#!/usr/bin/env python3
"""
ROS 接口封装 - 订阅/发布地面站所需的所有 topic
"""
import os
import sys
import math
import rospy
from std_msgs.msg import Bool, Float32, Int32, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamSet
from mavros_msgs.msg import ParamValue
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 将配置管理器加入路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config_manager import ConfigManager


class ROSInterface:
    """地面站 ROS 接口"""

    def __init__(self):
        self.cfg = ConfigManager()
        self.bridge = CvBridge()

        # ---- 状态缓存 ----
        self.fc_connected = False
        self.fc_armed = False
        self.fc_mode = "UNKNOWN"
        self.local_pose = None
        self.yaw_aligned = False
        self.pixel_error = 0.0
        self.locked_target_id = -1  # -1 = 自动
        self.detection_image = None
        self.detections = []  # 当前检测到的目标列表
        self.efo_mag = 0.0
        self.contact_state = -1

        # ---- 发布者 ----
        self.pub_lock_target = None
        self.pub_cmd_vel = None
        self.pub_setpoint_raw = None
        self.pub_arm_angle = None

        # ---- 服务代理 ----
        self.srv_arming = None
        self.srv_set_mode = None
        self.srv_cmd_long = None
        self.srv_param_set = None

        self._init_ros()

    def _init_ros(self):
        """初始化 ROS 节点和 topic"""
        rospy.init_node('huaqiccc_ground_station', anonymous=True)

        t = self.cfg.config.get('topics', {})

        # 订阅者
        rospy.Subscriber(t.get('mavros_state', '/mavros/state'), State, self._cb_state)
        rospy.Subscriber(t.get('local_pose', '/mavros/local_position/pose'), PoseStamped, self._cb_pose)
        rospy.Subscriber(self.cfg.get('vision.aligned_topic', '/yolo/yaw_aligned'), Bool, self._cb_aligned)
        rospy.Subscriber(self.cfg.get('vision.error_topic', '/yolo/pixel_error'), Float32, self._cb_pixel_error)
        rospy.Subscriber(self.cfg.get('vision.image_topic', '/yolo/detection_image'), Image, self._cb_image)

        # 检测信息 (JSON)
        rospy.Subscriber('/yolo/detections_info', String, self._cb_detections)

        # GMO topic (可选)
        try:
            rospy.Subscriber(t.get('external_force', '/fmu/external_force_estimate/out'), Float32, self._cb_efo)
        except Exception:
            pass
        try:
            rospy.Subscriber(t.get('contact_state', '/fmu/contact_state/out'), Int32, self._cb_contact)
        except Exception:
            pass

        # 发布者
        self.pub_lock_target = rospy.Publisher(
            self.cfg.get('vision.lock_topic', '/yolo/lock_target'), Int32, queue_size=1, latch=True)
        self.pub_cmd_vel = rospy.Publisher(
            t.get('cmd_vel', '/mavros/setpoint_velocity/cmd_vel'), TwistStamped, queue_size=10)
        self.pub_setpoint_raw = rospy.Publisher(
            t.get('setpoint_raw', '/mavros/setpoint_raw/local'), PositionTarget, queue_size=10)
        self.pub_arm_angle = rospy.Publisher(
            t.get('arm_angle_pub', '/huaqiccc/arm_angle'), Float32, queue_size=1)

        # 等待服务
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=5.0)
            self.srv_arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        except Exception as e:
            rospy.logwarn(f"Arming service not available: {e}")

        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5.0)
            self.srv_set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        except Exception as e:
            rospy.logwarn(f"SetMode service not available: {e}")

        try:
            rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
            self.srv_cmd_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        except Exception as e:
            rospy.logwarn(f"CommandLong service not available: {e}")

        rospy.loginfo("[ROSInterface] 初始化完成")

    # ---------- 回调 ----------
    def _cb_state(self, msg):
        self.fc_connected = msg.connected
        self.fc_armed = msg.armed
        self.fc_mode = msg.mode

    def _cb_pose(self, msg):
        self.local_pose = msg.pose

    def _cb_aligned(self, msg):
        self.yaw_aligned = msg.data

    def _cb_pixel_error(self, msg):
        self.pixel_error = msg.data

    def _cb_image(self, msg):
        try:
            self.detection_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"图像转换失败: {e}")

    def _cb_efo(self, msg):
        self.efo_mag = msg.data

    def _cb_contact(self, msg):
        self.contact_state = msg.data

    def _cb_detections(self, msg):
        """缓存检测信息 JSON"""
        try:
            self.detections_info = json.loads(msg.data)
        except Exception:
            self.detections_info = {}

    # ---------- 控制指令 ----------
    def lock_target(self, target_id):
        """锁定指定跟踪 ID，-1 表示自动选择"""
        self.locked_target_id = target_id
        self.pub_lock_target.publish(Int32(data=target_id))
        rospy.loginfo(f"[GroundStation] 锁定目标 ID: {target_id}")

    def send_yaw_rate(self, yaw_rate):
        """发送 YAW 角速度指令"""
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "body"
        twist.twist.angular.z = yaw_rate
        self.pub_cmd_vel.publish(twist)

    def send_position_target(self, x, y, z, yaw=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        """发送位置/速度混合 setpoint"""
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        has_vel = abs(vx) > 1e-6 or abs(vy) > 1e-6
        has_yaw_rate = abs(yaw_rate) > 1e-6

        if has_vel and has_yaw_rate:
            msg.type_mask = (
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            )
            msg.velocity.x = vx
            msg.velocity.y = vy
            msg.yaw_rate = yaw_rate
        elif has_vel:
            msg.type_mask = (
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            msg.velocity.x = vx
            msg.velocity.y = vy
        elif has_yaw_rate:
            msg.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            )
            msg.yaw_rate = yaw_rate
        else:
            msg.type_mask = (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        self.pub_setpoint_raw.publish(msg)

    def set_mode(self, mode_name):
        """设置飞行模式"""
        if self.srv_set_mode is None:
            rospy.logerr("SetMode service not available")
            return False
        try:
            resp = self.srv_set_mode(base_mode=0, custom_mode=mode_name)
            return resp.mode_sent
        except Exception as e:
            rospy.logerr(f"SetMode failed: {e}")
            return False

    def arm(self, arm=True):
        """解锁/锁定"""
        if self.srv_arming is None:
            rospy.logerr("Arming service not available")
            return False
        try:
            resp = self.srv_arming(arm)
            return resp.success
        except Exception as e:
            rospy.logerr(f"Arming failed: {e}")
            return False

    def send_morph_command_31440(self, angle):
        """通过 MAVLink CommandLong 发送 31440 变形命令"""
        if self.srv_cmd_long is None:
            rospy.logerr("CommandLong service not available")
            return False
        try:
            from mavros_msgs.srv import CommandLongRequest
            req = CommandLongRequest()
            req.broadcast = False
            req.command = 31440
            req.confirmation = 0
            req.param1 = float(angle)
            resp = self.srv_cmd_long(req)
            return getattr(resp, 'success', False)
        except Exception as e:
            rospy.logerr(f"31440 command failed: {e}")
            return False

    def emergency_stop(self):
        """紧急停止: 切到 stabilize + disarm"""
        rospy.logerr("[EMERGENCY] 紧急停止触发!")
        self.set_mode("STABILIZED")
        rospy.sleep(0.2)
        self.arm(False)

    def is_connected(self):
        return self.fc_connected

    def get_current_yaw(self):
        """从四元数提取当前 YAW (rad)"""
        if self.local_pose is None:
            return 0.0
        q = self.local_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
