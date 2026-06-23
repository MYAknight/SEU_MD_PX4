#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <algorithm>

namespace gazebo
{
  class DualArmRosControlPlugin : public ModelPlugin
  {
  public:
    physics::ModelPtr model;
    physics::JointPtr right_joint;
    physics::JointPtr left_joint;
    event::ConnectionPtr updateConnection;

    // ROS
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    ros::Subscriber fixSub;
    ros::Publisher statusPub;
    ros::Publisher armAnglePub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    std::unique_ptr<ros::AsyncSpinner> asyncSpinner;

    // Control params
    double targetAngle = 0.0;
    double kp = 5.0;
    double torqueLimit = 2.0;

    // Perching fix mode
    bool fixMode = false;
    bool autoFixEnabled = true;
    ignition::math::Pose3d fixedPose;
    uint64_t fixFrameCount = 0;
    uint64_t updateCount = 0;

    // Auto-fix thresholds
    double fixThresholdX = 4.85;
    double fixThresholdArm = -0.15;
    double fixThresholdVel = 0.05;
    double fixThresholdAngVel = 0.05;
    double fixHoldTime = 1.5;
    double fixStartTime = 0.0;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      model = _model;
      right_joint = model->GetJoint("right_arm_joint");
      left_joint  = model->GetJoint("left_arm_joint");

      if (!right_joint || !left_joint)
      {
        gzerr << "[DualArmRosControl] Joint not found! right=" << right_joint
              << " left=" << left_joint << std::endl;
        return;
      }

      if (_sdf->HasElement("kp"))
        kp = _sdf->Get<double>("kp");
      if (_sdf->HasElement("torqueLimit"))
        torqueLimit = _sdf->Get<double>("torqueLimit");
      if (_sdf->HasElement("initialAngle"))
        targetAngle = _sdf->Get<double>("initialAngle");

      if (_sdf->HasElement("autoFix"))
        autoFixEnabled = _sdf->Get<bool>("autoFix");
      if (_sdf->HasElement("fixThresholdX"))
        fixThresholdX = _sdf->Get<double>("fixThresholdX");
      if (_sdf->HasElement("fixThresholdArm"))
        fixThresholdArm = _sdf->Get<double>("fixThresholdArm");
      if (_sdf->HasElement("fixThresholdVel"))
        fixThresholdVel = _sdf->Get<double>("fixThresholdVel");
      if (_sdf->HasElement("fixThresholdAngVel"))
        fixThresholdAngVel = _sdf->Get<double>("fixThresholdAngVel");
      if (_sdf->HasElement("fixHoldTime"))
        fixHoldTime = _sdf->Get<double>("fixHoldTime");

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "huaqiccc_arm_controller",
                  ros::init_options::NoSigintHandler);
      }
      rosNode.reset(new ros::NodeHandle("huaqiccc"));

      // Bind subscribers to our custom queue so callbacks run in RosQueueThread
      ros::SubscribeOptions arm_so = ros::SubscribeOptions::create<std_msgs::Float64>(
        "/huaqiccc/arm_angle", 1,
        boost::bind(&DualArmRosControlPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &rosQueue);
      rosSub = rosNode->subscribe(arm_so);

      ros::SubscribeOptions fix_so = ros::SubscribeOptions::create<std_msgs::Bool>(
        "/huaqiccc/fix_perching", 1,
        boost::bind(&DualArmRosControlPlugin::OnFixMsg, this, _1),
        ros::VoidPtr(), &rosQueue);
      fixSub = rosNode->subscribe(fix_so);

      statusPub = rosNode->advertise<std_msgs::Bool>("/huaqiccc/perching_status", 1, true);
      armAnglePub = rosNode->advertise<std_msgs::Float64>("/huaqiccc/arm_actual_angle", 1, true);

      rosQueueThread = std::thread(std::bind(&DualArmRosControlPlugin::RosQueueThread, this));

      updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DualArmRosControlPlugin::OnUpdate, this));

      gzlog << "[DualArmRosControl] Loaded. kp=" << kp
            << " limit=" << torqueLimit << " initialAngle=" << targetAngle
            << " autoFix=" << autoFixEnabled << std::endl;
    }

    void OnRosMsg(const std_msgs::Float64::ConstPtr &_msg)
    {
      if (fixMode)
        return;
      targetAngle = _msg->data;
      gzlog << "[DualArmRosControl] Arm target: " << targetAngle << " rad" << std::endl;
    }

    void OnFixMsg(const std_msgs::Bool::ConstPtr &_msg)
    {
      if (_msg->data && !fixMode)
      {
        fixMode = true;
        fixedPose = model->WorldPose();
        fixFrameCount = 0;
        gzlog << "[DualArmRosControl] FIX ENABLED (manual) pose="
              << fixedPose.Pos().X() << "," << fixedPose.Pos().Y() << "," << fixedPose.Pos().Z()
              << std::endl;
        std_msgs::Bool out;
        out.data = true;
        statusPub.publish(out);
      }
      else if (!_msg->data && fixMode)
      {
        fixMode = false;
        fixStartTime = 0.0;
        gzlog << "[DualArmRosControl] FIX DISABLED" << std::endl;
        std_msgs::Bool out;
        out.data = false;
        statusPub.publish(out);
      }
    }

    void RosQueueThread()
    {
      ros::Rate rate(100);
      while (rosNode->ok())
      {
        rosQueue.callAvailable(ros::WallDuration(0.01));
        rate.sleep();
      }
    }

    void OnUpdate()
    {
      // ---- FIX MODE: freeze model pose and zero all velocities ----
      if (fixMode)
      {
        model->SetWorldPose(fixedPose);
        model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
        // Also zero velocity on every link to prevent internal physics drift
        for (auto &link : model->GetLinks())
        {
          link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
          link->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
          // Reset accumulated forces
          link->ResetPhysicsStates();
        }
        fixFrameCount++;
        if (fixFrameCount == 1 || fixFrameCount % 600 == 0)
        {
          gzlog << "[DualArmRosControl] FIX active frame=" << fixFrameCount
                << " pose=" << fixedPose.Pos().X() << "," << fixedPose.Pos().Y() << "," << fixedPose.Pos().Z()
                << std::endl;
        }
        return;
      }

      // ---- AUTO FIX detection ----
      if (autoFixEnabled)
      {
        ignition::math::Pose3d pose = model->WorldPose();
        ignition::math::Vector3d vel = model->WorldLinearVel();
        ignition::math::Vector3d angVel = model->WorldAngularVel();

        double x = pose.Pos().X();
        double rightAngle = right_joint->Position(0);
        double leftAngle  = left_joint->Position(0);

        bool nearPole = (x > fixThresholdX);
        bool armsClosed = (rightAngle > fixThresholdArm) && (leftAngle > fixThresholdArm);
        bool lowVel = vel.Length() < fixThresholdVel;
        bool lowAngVel = angVel.Length() < fixThresholdAngVel;

        if (nearPole && armsClosed && lowVel && lowAngVel)
        {
          if (fixStartTime == 0.0)
          {
            fixStartTime = model->GetWorld()->SimTime().Double();
            gzlog << "[DualArmRosControl] Auto-fix conditions met, waiting..."
                  << " x=" << x << " armR=" << rightAngle << " armL=" << leftAngle
                  << " vel=" << vel.Length() << std::endl;
          }
          else
          {
            double elapsed = model->GetWorld()->SimTime().Double() - fixStartTime;
            if (elapsed > fixHoldTime)
            {
              fixMode = true;
              fixedPose = pose;
              fixFrameCount = 0;
              gzlog << "[DualArmRosControl] AUTO FIX triggered!"
                    << " x=" << x << " armR=" << rightAngle << " armL=" << leftAngle
                    << std::endl;
              std_msgs::Bool out;
              out.data = true;
              statusPub.publish(out);
              return;
            }
          }
        }
        else
        {
          if (fixStartTime != 0.0)
          {
            gzlog << "[DualArmRosControl] Auto-fix conditions broken"
                  << " near=" << nearPole << " closed=" << armsClosed
                  << " lowVel=" << lowVel << " lowAngVel=" << lowAngVel << std::endl;
          }
          fixStartTime = 0.0;
        }
      }

      // ---- Normal arm control ----
      if (!right_joint || !left_joint)
        return;

      double rightAngle = right_joint->Position(0);
      double leftAngle  = left_joint->Position(0);

      double errorRight = targetAngle - rightAngle;
      double errorLeft  = targetAngle - leftAngle;

      double torqueRight = std::clamp(kp * errorRight, -torqueLimit, torqueLimit);
      double torqueLeft  = std::clamp(kp * errorLeft,  -torqueLimit, torqueLimit);

      right_joint->SetForce(0, torqueRight);
      left_joint->SetForce(0, torqueLeft);

      // Publish actual arm angle at ~10 Hz for SITL monitoring (matches ground-station morph_angle_rad).
      updateCount++;
      if (armAnglePub && (updateCount % 100 == 0))
      {
        std_msgs::Float64 msg;
        msg.data = 0.5 * (rightAngle + leftAngle);
        armAnglePub.publish(msg);
      }
    }

    ~DualArmRosControlPlugin()
    {
      rosNode->shutdown();
      if (rosQueueThread.joinable())
        rosQueueThread.join();
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(DualArmRosControlPlugin)
}
