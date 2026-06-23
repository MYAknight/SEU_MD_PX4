#!/usr/bin/env python3
"""
NOKOV/VRPN Pose -> MAVROS vision_pose/pose_cov

将 VRPN 发布的 geometry_msgs/PoseStamped 转换为
geometry_msgs/PoseWithCovarianceStamped 并发布到
/mavros/vision_pose/pose_cov，同时设置合理的协方差。

EKF2 需要非零协方差来正确评估视觉观测的不确定性。
NOKOV 动捕系统精度通常在亚毫米级，这里使用 1mm 位置标准差
和 0.05rad 姿态标准差作为保守估计。

用法:
    rosrun optimize mocap_to_vision_pose.py \
        _input:=/vrpn_client_node/Tracker1/pose \
        _output:=/mavros/vision_pose/pose_cov \
        _rate:=40.0 \
        _position_std:="0.001,0.001,0.001" \
        _orientation_std:="0.05,0.05,0.05"
"""

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class MocapToVisionPose:
    def __init__(self):
        rospy.init_node("mocap_to_vision_pose")

        self.input_topic = rospy.get_param("~input", "/vrpn_client_node/Tracker1/pose")
        self.output_topic = rospy.get_param("~output", "/mavros/vision_pose/pose_cov")
        self.rate_hz = rospy.get_param("~rate", 40.0)
        # 标准差 (m, rad)
        pos_std = rospy.get_param("~position_std", [0.001, 0.001, 0.001])
        ori_std = rospy.get_param("~orientation_std", [0.05, 0.05, 0.05])

        if len(pos_std) != 3 or len(ori_std) != 3:
            rospy.logerr("position_std 和 orientation_std 必须为 3 元素列表")
            raise ValueError("协方差标准差格式错误")

        # 协方差矩阵对角线 = 标准差平方
        self.covariance = [0.0] * 36
        for i in range(3):
            self.covariance[i * 6 + i] = pos_std[i] ** 2
        for i in range(3):
            self.covariance[(i + 3) * 6 + (i + 3)] = ori_std[i] ** 2

        self._last_msg = None
        self._last_stamp = rospy.Time(0)

        self.pub = rospy.Publisher(self.output_topic, PoseWithCovarianceStamped, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, PoseStamped, self._cb)

        rospy.loginfo(
            f"[MocapToVisionPose] {self.input_topic} -> {self.output_topic}, "
            f"rate={self.rate_hz}Hz, position_std={pos_std}, orientation_std={ori_std}"
        )

    def _cb(self, msg: PoseStamped):
        self._last_msg = msg

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self._last_msg is not None:
                # 时间戳使用当前 ROS 时间，避免 VRPN 时间戳与 ROS 不同步
                out = PoseWithCovarianceStamped()
                out.header.stamp = rospy.Time.now()
                out.header.frame_id = self._last_msg.header.frame_id
                out.pose.pose = self._last_msg.pose
                out.pose.covariance = self.covariance
                self.pub.publish(out)
            rate.sleep()


def main():
    node = MocapToVisionPose()
    node.run()


if __name__ == "__main__":
    main()
