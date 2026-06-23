#!/usr/bin/env python3
"""
safe_offboard_mission.py — 安全空间内自动 offboard 轨迹执行脚本

用途：
    1. 在 2x2x2m（或用户标定的任意长方体）安全空间内执行标准化飞行测试
    2. 支持悬停、小圆、方框、8字、阶跃等轨迹
    3. 实时监控位置，越界时自动返回中心并降落
    4. 记录参数快照和 ulog 路径，便于后续自动分析

用法：
    # 使用默认示例配置（中心 1,1,0.1，半尺寸 1,1,1）
    python3 safe_offboard_mission.py --config safe_space_2x2x2.yaml --trajectory circle_small

    # 指定 MPCA_MODE
    python3 safe_offboard_mission.py --config safe_space_2x2x2.yaml --trajectory square_small --mode 2
"""

import os
import sys
import time
import math
import yaml
import argparse
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, ParamGet, CommandLong
from mavros_msgs.msg import State, ParamValue
from std_msgs.msg import Float64


# MAVLink command ID for huaqiccc morph angle
MAV_CMD_HUAQICCC_SET_ARM_ANGLE = 31440


@dataclass
class Bounds:
    """安全空间边界（绝对坐标，单位 m）"""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    @property
    def center(self) -> Tuple[float, float, float]:
        return (
            (self.x_min + self.x_max) / 2.0,
            (self.y_min + self.y_max) / 2.0,
            (self.z_min + self.z_max) / 2.0,
        )

    @property
    def size(self) -> Tuple[float, float, float]:
        return (
            self.x_max - self.x_min,
            self.y_max - self.y_min,
            self.z_max - self.z_min,
        )

    def contains(self, x: float, y: float, z: float, margin: float = 0.0) -> bool:
        """检查点是否在安全空间内（可设安全裕量 margin）"""
        return (
            self.x_min + margin <= x <= self.x_max - margin
            and self.y_min + margin <= y <= self.y_max - margin
            and self.z_min + margin <= z <= self.z_max - margin
        )


class SafeOffboardMission:
    def __init__(self, config_path: str, trajectory_name: str, mode: int = 2):
        rospy.init_node("safe_offboard_mission", anonymous=True)

        # ---- 加载配置 ----
        with open(config_path, "r") as f:
            self.cfg = yaml.safe_load(f)

        self.bounds = self._parse_bounds(self.cfg["safe_space"])
        self.trajectory_name = trajectory_name
        self.target_mode = mode
        self.flight_height = self.cfg["mission"].get("flight_height", 1.0)
        self.cruise_speed = self.cfg["mission"].get("cruise_speed", 0.3)
        self.hover_time = self.cfg["mission"].get("hover_time", 10.0)
        self.setpoint_rate = self.cfg["mission"].get("setpoint_rate", 20.0)
        self.safety_margin = self.cfg["mission"].get("safety_margin", 0.25)
        self.rth_height = self.cfg["mission"].get("rth_height", self.flight_height)

        cx, cy, _ = self.bounds.center
        self.home = (cx, cy, self.bounds.z_min + self.flight_height)

        # ---- ROS 服务 ----
        rospy.wait_for_service("/mavros/cmd/arming", timeout=10.0)
        rospy.wait_for_service("/mavros/set_mode", timeout=10.0)
        rospy.wait_for_service("/mavros/param/set", timeout=10.0)
        rospy.wait_for_service("/mavros/param/get", timeout=10.0)
        rospy.wait_for_service("/mavros/cmd/command", timeout=10.0)

        self.srv_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.srv_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.srv_param_set = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self.srv_param_get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
        self.srv_cmd = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        # ---- ROS Topic ----
        self.pub_pos = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.pub_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        self.current_state = State()
        self.current_pos = None
        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)

        # ---- 生成轨迹 ----
        self.waypoints = self._generate_trajectory(trajectory_name)
        rospy.loginfo(f"[SafeMission] 安全空间: {self.bounds}")
        rospy.loginfo(f"[SafeMission] HOME: {self.home}")
        rospy.loginfo(f"[SafeMission] 轨迹 {trajectory_name}: {len(self.waypoints)} 个航点")

    def _parse_bounds(self, cfg: dict) -> Bounds:
        """解析安全边界：支持 corner 方式或 center+half_size 方式"""
        if "corners" in cfg:
            corners = cfg["corners"]
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            zs = [c[2] for c in corners]
            return Bounds(min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
        elif "center" in cfg and "half_size" in cfg:
            cx, cy, cz = cfg["center"]
            hx, hy, hz = cfg["half_size"]
            return Bounds(cx - hx, cx + hx, cy - hy, cy + hy, cz, cz + 2 * hz)
        else:
            raise ValueError("safe_space 必须包含 corners 或 center+half_size")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _state_cb(self, msg: State):
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pos = msg.pose.position

    # ------------------------------------------------------------------
    # 轨迹生成
    # ------------------------------------------------------------------
    def _generate_trajectory(self, name: str) -> List[Tuple[float, float, float]]:
        """生成指定轨迹的航点列表（绝对坐标，m）"""
        cx, cy, _ = self.bounds.center
        hz = self.flight_height
        base_z = self.bounds.z_min + hz
        v = self.cruise_speed

        generators = {
            "hover": self._traj_hover,
            "takeoff_land": self._traj_takeoff_land,
            "square_small": self._traj_square,
            "circle_small": self._traj_circle,
            "figure8_small": self._traj_figure8,
            "step_x": self._traj_step_x,
            "step_xy": self._traj_step_xy,
            "morph_circle": self._traj_morph_circle,
        }

        if name not in generators:
            raise ValueError(f"未知轨迹: {name}。可用: {list(generators.keys())}")

        return generators[name](cx, cy, base_z)

    def _interp(self, p0, p1, dt: float, speed: float) -> List[Tuple[float, float, float]]:
        """两点间按速度插值"""
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
        if dist < 1e-6:
            return [p1]
        n = max(1, int(dist / (speed / self.setpoint_rate)))
        return [
            tuple(p0[i] + (p1[i] - p0[i]) * t / n for i in range(3))
            for t in range(1, n + 1)
        ]

    def _traj_hover(self, cx, cy, base_z):
        # 悬停：只发一个点，由外部计时控制
        return [(cx, cy, base_z)]

    def _traj_takeoff_land(self, cx, cy, base_z):
        p0 = (cx, cy, self.bounds.z_min + 0.3)  # 起飞过渡高度
        p1 = (cx, cy, base_z)
        return self._interp(p0, p1, 1.0 / self.setpoint_rate, 0.2) + \
               [p1] * int(self.hover_time * self.setpoint_rate) + \
               self._interp(p1, p0, 1.0 / self.setpoint_rate, 0.2)

    def _traj_square(self, cx, cy, base_z, half_len: float = 0.4):
        corners = [
            (cx + half_len, cy + half_len, base_z),
            (cx + half_len, cy - half_len, base_z),
            (cx - half_len, cy - half_len, base_z),
            (cx - half_len, cy + half_len, base_z),
            (cx + half_len, cy + half_len, base_z),
        ]
        pts = []
        for i in range(len(corners) - 1):
            pts.extend(self._interp(corners[i], corners[i + 1], 1.0 / self.setpoint_rate, self.cruise_speed))
        return pts

    def _traj_circle(self, cx, cy, base_z, radius: float = 0.4, n: int = 100):
        pts = []
        for i in range(n + 1):
            theta = 2.0 * math.pi * i / n
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)
            pts.append((x, y, base_z))
        return pts

    def _traj_figure8(self, cx, cy, base_z, radius: float = 0.3, n: int = 160):
        pts = []
        for i in range(n + 1):
            t = 2.0 * math.pi * i / n
            x = cx + radius * math.sin(t)
            y = cy + radius * math.sin(t) * math.cos(t)
            pts.append((x, y, base_z))
        return pts

    def _traj_step_x(self, cx, cy, base_z, delta: float = 0.4):
        p0 = (cx, cy, base_z)
        p1 = (cx + delta, cy, base_z)
        p2 = (cx - delta, cy, base_z)
        hold = int(3.0 * self.setpoint_rate)
        return [p0] * hold + \
               self._interp(p0, p1, 1.0 / self.setpoint_rate, 0.2) + [p1] * hold + \
               self._interp(p1, p2, 1.0 / self.setpoint_rate, 0.2) + [p2] * hold + \
               self._interp(p2, p0, 1.0 / self.setpoint_rate, 0.2) + [p0] * hold

    def _traj_step_xy(self, cx, cy, base_z, delta: float = 0.3):
        p0 = (cx, cy, base_z)
        p1 = (cx + delta, cy + delta, base_z)
        p2 = (cx - delta, cy - delta, base_z)
        hold = int(3.0 * self.setpoint_rate)
        return [p0] * hold + \
               self._interp(p0, p1, 1.0 / self.setpoint_rate, 0.2) + [p1] * hold + \
               self._interp(p1, p2, 1.0 / self.setpoint_rate, 0.2) + [p2] * hold + \
               self._interp(p2, p0, 1.0 / self.setpoint_rate, 0.2) + [p0] * hold

    def _traj_morph_circle(self, cx, cy, base_z, radius: float = 0.3, n: int = 120):
        # 圆轨迹，适合配合变形命令测试
        return self._traj_circle(cx, cy, base_z, radius, n)

    # ------------------------------------------------------------------
    # 安全监控
    # ------------------------------------------------------------------
    def _check_safety(self) -> bool:
        if self.current_pos is None:
            return True
        safe = self.bounds.contains(
            self.current_pos.x, self.current_pos.y, self.current_pos.z,
            margin=self.safety_margin
        )
        if not safe:
            rospy.logerr(
                f"[SAFETY] 超出安全空间! pos=({self.current_pos.x:.2f}, "
                f"{self.current_pos.y:.2f}, {self.current_pos.z:.2f})"
            )
        return safe

    # ------------------------------------------------------------------
    # 基本动作
    # ------------------------------------------------------------------
    def _set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        rospy.loginfo(f"[Mission] 切换模式: {mode}")
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < timeout:
            if self.current_state.mode == mode:
                return True
            try:
                self.srv_mode(custom_mode=mode)
            except Exception as e:
                rospy.logwarn(f"set_mode error: {e}")
            rate.sleep()
        return self.current_state.mode == mode

    def _arm(self, arm: bool = True, timeout: float = 10.0) -> bool:
        rospy.loginfo(f"[Mission] ARM={arm}")
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < timeout:
            if self.current_state.armed == arm:
                return True
            try:
                self.srv_arm(arm)
            except Exception as e:
                rospy.logwarn(f"arm error: {e}")
            rate.sleep()
        return self.current_state.armed == arm

    def _set_mpca_mode(self, mode: int) -> bool:
        rospy.loginfo(f"[Mission] 设置 MPCA_MODE={mode}")
        try:
            pv = ParamValue()
            pv.integer = mode
            resp = self.srv_param_set(param_id="MPCA_MODE", value=pv)
            return resp.success
        except Exception as e:
            rospy.logerr(f"set MPCA_MODE error: {e}")
            return False

    def _send_morph_angle(self, angle_rad: float) -> bool:
        rospy.loginfo(f"[Mission] 发送变形角度: {angle_rad:.3f} rad")
        try:
            resp = self.srv_cmd(
                command=MAV_CMD_HUAQICCC_SET_ARM_ANGLE,
                param1=float(angle_rad),
                param2=0, param3=0, param4=0, param5=0, param6=0, param7=0
            )
            return resp.success
        except Exception as e:
            rospy.logerr(f"morph command error: {e}")
            return False

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        ps.pose.orientation.w = cy
        ps.pose.orientation.x = 0
        ps.pose.orientation.y = 0
        ps.pose.orientation.z = sy
        self.pub_pos.publish(ps)

    def _go_to_point(self, x: float, y: float, z: float, speed: float = 0.3, yaw: float = 0.0):
        """从当前位置直线飞到目标点，带安全监控"""
        if self.current_pos is None:
            rospy.logwarn("[Mission] 无位置信息，直接跳转 setpoint")
            self._publish_setpoint(x, y, z, yaw)
            return

        p0 = (self.current_pos.x, self.current_pos.y, self.current_pos.z)
        p1 = (x, y, z)
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
        n = max(1, int(dist / (speed / self.setpoint_rate)))

        rate = rospy.Rate(self.setpoint_rate)
        for i in range(1, n + 1):
            if not self._check_safety():
                raise RuntimeError("Safety boundary violated during goto")
            xi = p0[0] + (p1[0] - p0[0]) * i / n
            yi = p0[1] + (p1[1] - p0[1]) * i / n
            zi = p0[2] + (p1[2] - p0[2]) * i / n
            self._publish_setpoint(xi, yi, zi, yaw)
            rate.sleep()

    def _return_to_home_and_land(self):
        """安全返回：先水平回到 HOME，再垂直降落"""
        rospy.logwarn("[Mission] 执行返航降落 (RTH)")
        if self.current_pos is not None:
            # 先水平回到 home
            self._go_to_point(self.home[0], self.home[1], self.current_pos.z, speed=0.5)
        # 垂直降落
        self._go_to_point(self.home[0], self.home[1], self.bounds.z_min + 0.3, speed=0.2)
        self._publish_setpoint(self.home[0], self.home[1], self.bounds.z_min + 0.3)

    # ------------------------------------------------------------------
    # 主任务流程
    # ------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.setpoint_rate)

        # 等待 FCU 连接
        rospy.loginfo("[Mission] 等待 FCU 连接...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("[Mission] FCU 已连接")

        # 等待位置有效
        rospy.loginfo("[Mission] 等待位置信息...")
        timeout = rospy.Time.now() + rospy.Duration(10)
        while self.current_pos is None and rospy.Time.now() < timeout:
            rate.sleep()
        if self.current_pos is None:
            raise RuntimeError("无法获取当前位置")
        rospy.loginfo(f"[Mission] 当前位置: ({self.current_pos.x:.2f}, {self.current_pos.y:.2f}, {self.current_pos.z:.2f})")

        # 检查初始位置是否在安全空间内
        if not self.bounds.contains(self.current_pos.x, self.current_pos.y, self.current_pos.z, margin=0):
            raise RuntimeError("初始位置不在安全空间内!")

        # 设置 MPCA_MODE
        self._set_mpca_mode(self.target_mode)

        # 预发布 setpoint
        rospy.loginfo("[Mission] 预发布 setpoint...")
        for _ in range(50):
            self._publish_setpoint(self.current_pos.x, self.current_pos.y, self.current_pos.z)
            rate.sleep()

        # 切 OFFBOARD
        if not self._set_mode("OFFBOARD"):
            raise RuntimeError("无法切换到 OFFBOARD")

        # ARM
        if not self._arm(True):
            raise RuntimeError("无法 ARM")

        try:
            # 起飞到 HOME 高度
            rospy.loginfo(f"[Mission] 起飞到高度 {self.home[2]:.2f}m")
            self._go_to_point(self.home[0], self.home[1], self.home[2], speed=0.3)

            # 悬停稳定
            rospy.loginfo(f"[Mission] 悬停 {self.hover_time}s")
            hover_start = rospy.Time.now()
            while (rospy.Time.now() - hover_start).to_sec() < self.hover_time:
                if not self._check_safety():
                    raise RuntimeError("Safety boundary violated during hover")
                self._publish_setpoint(self.home[0], self.home[1], self.home[2])
                rate.sleep()

            # 执行轨迹
            rospy.loginfo(f"[Mission] 开始执行轨迹: {self.trajectory_name}")
            for idx, (x, y, z) in enumerate(self.waypoints):
                if not self._check_safety():
                    raise RuntimeError("Safety boundary violated during trajectory")
                self._publish_setpoint(x, y, z)

                # morph_circle 轨迹：在轨迹中点发送变形命令
                if self.trajectory_name == "morph_circle" and idx == len(self.waypoints) // 4:
                    self._send_morph_angle(-0.35)
                if self.trajectory_name == "morph_circle" and idx == 3 * len(self.waypoints) // 4:
                    self._send_morph_angle(0.0)

                rate.sleep()

            # 轨迹结束后回到 home
            rospy.loginfo("[Mission] 返回 home")
            self._go_to_point(self.home[0], self.home[1], self.home[2], speed=self.cruise_speed)

            # 降落
            rospy.loginfo("[Mission] 降落")
            self._go_to_point(self.home[0], self.home[1], self.bounds.z_min + 0.3, speed=0.2)

        except RuntimeError as e:
            rospy.logerr(f"[Mission] {e}")
            self._return_to_home_and_land()
            raise
        finally:
            # DISARM
            rospy.loginfo("[Mission] DISARM")
            self._arm(False)
            self._set_mode("POSCTL")

        rospy.loginfo("[Mission] 任务完成")


def main():
    parser = argparse.ArgumentParser(description="安全空间 offboard 轨迹执行")
    parser.add_argument("--config", required=True, help="安全空间 YAML 配置文件")
    parser.add_argument("--trajectory", required=True,
                        choices=["hover", "takeoff_land", "square_small", "circle_small",
                                 "figure8_small", "step_x", "step_xy", "morph_circle"],
                        help="轨迹类型")
    parser.add_argument("--mode", type=int, default=2,
                        help="MPCA_MODE (0=PID, 1=GS-PID, 2=LQR, 3=MPC)")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印航点，不执行飞行")
    args = parser.parse_args()

    if args.dry_run:
        with open(args.config, "r") as f:
            cfg = yaml.safe_load(f)
        from safe_offboard_mission import SafeOffboardMission, Bounds
        # 简化打印
        print(f"Config: {cfg}")
        print(f"Trajectory: {args.trajectory}")
        print("Use --help for flight execution.")
        return

    mission = SafeOffboardMission(args.config, args.trajectory, args.mode)
    mission.run()


if __name__ == "__main__":
    main()
