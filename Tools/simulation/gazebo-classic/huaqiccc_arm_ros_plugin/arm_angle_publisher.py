#!/usr/bin/env python3
"""
 arm_angle_publisher.py
 ======================
 用于向 huaqiccc ROS Gazebo 插件发送目标机臂角度的简单节点。

 用法：
   rosrun huaqiccc_arm_ros_plugin arm_angle_publisher.py _angle:=0.2

 或动态发送：
   rostopic pub /huaqiccc/arm_angle std_msgs/Float64 "data: 0.2" --once
"""

import rospy
from std_msgs.msg import Float64
import sys


def main():
    rospy.init_node('huaqiccc_arm_angle_publisher', anonymous=True)

    # 参数
    angle = rospy.get_param('~angle', 0.0)
    rate_hz = rospy.get_param('~rate', 10.0)

    pub = rospy.Publisher('/huaqiccc/arm_angle', Float64, queue_size=1)
    rospy.sleep(0.5)  # 等待连接建立

    msg = Float64()
    msg.data = angle

    pub.publish(msg)
    rospy.loginfo("Published target arm angle: %.3f rad (%.1f deg)", angle, angle * 57.2958)

    # 持续发布保持角度（插件只会在收到新消息时更新，但持续发布可防止丢包）
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
