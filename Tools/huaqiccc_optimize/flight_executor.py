#!/usr/bin/env python3
"""
flight_executor.py — 基于实际测量安全多边形的一键飞行执行器

作为 ROS 节点运行，提供以下 Service：
    /optimize/start_flight  (Trigger)
    /optimize/land          (Trigger)
    /optimize/emergency_stop (Trigger)

用法：
    rosrun optimize flight_executor.py
    或
    python3 flight_executor.py
"""

import os
import sys
import time
import math
import yaml
import signal
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rospy
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, ParamGet, CommandLong, CommandTOL
from mavros_msgs.msg import State, ParamValue


MAV_CMD_HUAQICCC_SET_ARM_ANGLE = 31440


class SafePolygon:
    """
    凸多边形安全空间。

    支持：
      - 点是否在多边形内部判断
      - 点到多边形边界的距离计算
      - 将外部/过近的点沿中心方向裁剪到安全区域内
    """

    def __init__(self, corners: List[List[float]], z_min: float, z_max: float):
        # corners: [[x0,y0,z0], [x1,y1,z1], ...] 按顺序闭合，不需要重复首点
        self.corners = [(float(c[0]), float(c[1])) for c in corners]
        self.n = len(self.corners)
        if self.n < 3:
            raise ValueError("安全多边形至少需要 3 个角点")

        self.z_min = float(z_min)
        self.z_max = float(z_max)

        # 计算多边形重心作为内部参考点（比 home 更通用）
        self.centroid = self._compute_centroid()

        # 记录多边形方向：True 为逆时针，False 为顺时针
        self.ccw = self._is_ccw()

    # ------------------------------------------------------------------
    # 基础几何工具
    # ------------------------------------------------------------------
    @staticmethod
    def _cross(ax: float, ay: float, bx: float, by: float) -> float:
        return ax * by - ay * bx

    def _compute_centroid(self) -> Tuple[float, float]:
        cx = sum(p[0] for p in self.corners) / self.n
        cy = sum(p[1] for p in self.corners) / self.n
        return (cx, cy)

    def _is_ccw(self) -> bool:
        """使用标准鞋带公式判断多边形是否为逆时针。"""
        area2 = 0.0
        for i in range(self.n):
            x1, y1 = self.corners[i]
            x2, y2 = self.corners[(i + 1) % self.n]
            area2 += x1 * y2 - x2 * y1
        return area2 > 0

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------
    def contains(self, x: float, y: float, z: float, margin: float = 0.0) -> bool:
        """判断点 (x,y,z) 是否在安全空间内（水平方向带 margin，垂直方向只要求在地平面范围内）。"""
        # 垂直方向：允许略高于/低于 z_min/z_max（地面放置和降落缓冲）
        if not (self.z_min - 0.05 <= z <= self.z_max):
            return False
        return self._contains_2d(x, y, margin)

    def _contains_2d(self, x: float, y: float, margin: float = 0.0) -> bool:
        """判断水平点 (x,y) 是否在带 margin 的多边形内。"""
        # 对于 ccw 多边形，内部点满足 cross(edge, p-a) > 0
        # 对于 cw 多边形，内部点满足 cross(edge, p-a) < 0
        sign = 1.0 if self.ccw else -1.0
        for i in range(self.n):
            x1, y1 = self.corners[i]
            x2, y2 = self.corners[(i + 1) % self.n]
            ex, ey = x2 - x1, y2 - y1
            px, py = x - x1, y - y1
            cross = ex * py - ey * px
            # 点到边的有向距离 = sign * cross / |edge|
            # 内部要求该距离 >= margin
            if sign * cross < margin * math.hypot(ex, ey):
                return False
        return True

    def distance_to_boundary(self, x: float, y: float) -> float:
        """计算水平点 (x,y) 到多边形边界的最短距离（负值表示在外部）。"""
        min_dist = float("inf")
        inside = True
        sign = 1.0 if self.ccw else -1.0
        for i in range(self.n):
            x1, y1 = self.corners[i]
            x2, y2 = self.corners[(i + 1) % self.n]
            ex, ey = x2 - x1, y2 - y1
            px, py = x - x1, y - y1
            cross = ex * py - ey * px
            signed = sign * cross / math.hypot(ex, ey)
            # 记录有符号距离（内部为正，外部为负）
            if signed < 0:
                inside = False

            # 同时考虑线段端点的绝对距离
            len2 = ex * ex + ey * ey
            t = max(0.0, min(1.0, (px * ex + py * ey) / len2)) if len2 > 1e-12 else 0.0
            cx = x1 + t * ex
            cy = y1 + t * ey
            dist = math.hypot(x - cx, y - cy)
            min_dist = min(min_dist, dist)

        if inside:
            # 内部最短距离 = 各边有向距离最小值
            min_signed = float("inf")
            for i in range(self.n):
                x1, y1 = self.corners[i]
                x2, y2 = self.corners[(i + 1) % self.n]
                ex, ey = x2 - x1, y2 - y1
                px, py = x - x1, y - y1
                cross = ex * py - ey * px
                signed = sign * cross / math.hypot(ex, ey)
                min_signed = min(min_signed, signed)
            return min_signed
        return -min_dist

    def clip_point(self, x: float, y: float, center: Tuple[float, float],
                   margin: float = 0.0) -> Tuple[float, float]:
        """
        将点 (x,y) 沿 center 方向裁剪，使其位于多边形内部且距离边界 >= margin。
        若点本身已满足条件，则原样返回。
        """
        cx, cy = center
        dx, dy = x - cx, y - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            return (x, y)

        ux, uy = dx / dist, dy / dist

        # 求射线 center + t * (ux,uy) (t >= 0) 与多边形边的交点，取最小正 t
        t_min = float("inf")
        for i in range(self.n):
            x1, y1 = self.corners[i]
            x2, y2 = self.corners[(i + 1) % self.n]
            ex, ey = x2 - x1, y2 - y1
            wx, wy = x1 - cx, y1 - cy

            denom = self._cross(ux, uy, ex, ey)
            if abs(denom) < 1e-12:
                continue

            t = self._cross(wx, wy, ex, ey) / denom
            s = self._cross(wx, wy, ux, uy) / denom
            if t >= -1e-9 and 0.0 <= s <= 1.0:
                t_min = min(t_min, max(t, 0.0))

        if not math.isfinite(t_min) or t_min <= 1e-9:
            # 未找到交点（中心在多边形外等异常情况），返回中心
            rospy.logwarn_throttle(1.0, "[SafePolygon] clip 无法找到边界交点，返回中心")
            return (cx, cy)

        # 沿射线最多可到达的距离 = 交点距离 - margin
        max_allowed = max(0.0, t_min - margin)

        if dist > max_allowed:
            return (cx + ux * max_allowed, cy + uy * max_allowed)
        return (x, y)

    def __repr__(self):
        pts = ", ".join(f"({x:.2f},{y:.2f})" for x, y in self.corners)
        return f"SafePolygon([{pts}], z=[{self.z_min},{self.z_max}])"


class FlightExecutor:
    def __init__(self):
        rospy.init_node("flight_executor", anonymous=True)

        # ---- 加载配置 ----
        cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "safe_space.yaml")
        with open(cfg_path, "r") as f:
            self.cfg = yaml.safe_load(f)

        ss_cfg = self.cfg["safe_space"]
        self.safe_space = SafePolygon(
            corners=ss_cfg["corners"],
            z_min=ss_cfg.get("z_min", 0.1),
            z_max=ss_cfg.get("z_max", 2.0),
        )
        self.home_xy = (float(ss_cfg["home"][0]), float(ss_cfg["home"][1]))
        self.ground_z = float(ss_cfg["home"][2])

        self.mission_cfg = self.cfg["mission"]
        self.flight_height = self.mission_cfg.get("flight_height", 1.0)
        self.cruise_speed = self.mission_cfg.get("cruise_speed", 0.3)
        self.hover_time = self.mission_cfg.get("hover_time", 10.0)
        self.setpoint_rate = self.mission_cfg.get("setpoint_rate", 20.0)
        self.safety_margin = self.mission_cfg.get("safety_margin", 0.30)
        self.takeoff_speed = self.mission_cfg.get("takeoff_speed", 0.2)
        self.land_speed = self.mission_cfg.get("land_speed", 0.2)
        self.land_final_height = self.mission_cfg.get("land_final_height", 0.3)
        self.default_mode = self.mission_cfg.get("default_mode", 2)

        # 计算安全飞行的最大半径（home 到边界距离 - margin）
        self.max_safe_radius = self._compute_max_safe_radius()
        rospy.loginfo(f"[FlightExecutor] home 到边界最短距离: {self.max_safe_radius + self.safety_margin:.3f} m")
        rospy.loginfo(f"[FlightExecutor] 扣除 {self.safety_margin} m 安全裕量后可用半径: {self.max_safe_radius:.3f} m")

        self.home = (self.home_xy[0], self.home_xy[1], self.ground_z + self.flight_height)
        self.land_home = (self.home_xy[0], self.home_xy[1], self.ground_z + self.land_final_height)

        # ---- ROS 服务 ----
        rospy.wait_for_service("/mavros/cmd/arming", timeout=10.0)
        rospy.wait_for_service("/mavros/set_mode", timeout=10.0)
        rospy.wait_for_service("/mavros/param/set", timeout=10.0)
        rospy.wait_for_service("/mavros/cmd/command", timeout=10.0)
        rospy.wait_for_service("/mavros/cmd/land", timeout=10.0)

        self.srv_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.srv_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.srv_param_set = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self.srv_cmd = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        self.srv_land = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)

        # ---- ROS Topic ----
        self.pub_pos = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.pub_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        self.current_state = State()
        self.current_pos = None
        self.current_yaw = 0.0
        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)

        # ---- Services ----
        rospy.Service("/optimize/start_flight", Trigger, self._handle_start_flight)
        rospy.Service("/optimize/land", Trigger, self._handle_land)
        rospy.Service("/optimize/emergency_stop", Trigger, self._handle_emergency_stop)

        # ---- 状态 ----
        self.is_running = False
        self.should_stop = False
        self.current_trajectory = None
        self.current_mode = self.default_mode
        self.flight_thread = None
        self.current_target = (0.0, 0.0, 0.0)

        rospy.loginfo(f"[FlightExecutor] 安全空间: {self.safe_space}")
        rospy.loginfo(f"[FlightExecutor] HOME: {self.home}")
        rospy.loginfo("[FlightExecutor] 服务已就绪")
        rospy.loginfo("  /optimize/start_flight")
        rospy.loginfo("  /optimize/land")
        rospy.loginfo("  /optimize/emergency_stop")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _state_cb(self, msg: State):
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pos = msg.pose.position
        q = msg.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ------------------------------------------------------------------
    # 配置解析 / 安全半径
    # ------------------------------------------------------------------
    def _compute_max_safe_radius(self) -> float:
        """计算 home 点到多边形各边的最短距离，再扣除 margin。"""
        min_dist = float("inf")
        for i in range(self.safe_space.n):
            x1, y1 = self.safe_space.corners[i]
            x2, y2 = self.safe_space.corners[(i + 1) % self.safe_space.n]
            dist, _ = self._point_to_segment_distance(self.home_xy[0], self.home_xy[1], x1, y1, x2, y2)
            min_dist = min(min_dist, dist)
        return max(0.0, min_dist - self.safety_margin)

    @staticmethod
    def _point_to_segment_distance(px: float, py: float, x1: float, y1: float,
                                   x2: float, y2: float) -> Tuple[float, Tuple[float, float]]:
        """点 (px,py) 到线段 (x1,y1)-(x2,y2) 的距离及最近点。"""
        ex, ey = x2 - x1, y2 - y1
        len2 = ex * ex + ey * ey
        if len2 < 1e-12:
            return math.hypot(px - x1, py - y1), (x1, y1)
        t = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / len2))
        cx = x1 + t * ex
        cy = y1 + t * ey
        return math.hypot(px - cx, py - cy), (cx, cy)

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------
    def _set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        rospy.loginfo(f"[FlightExecutor] 切换模式: {mode}")
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < timeout:
            if self.current_state.mode == mode:
                return True
            # 持续发布当前目标 setpoint，避免 OFFBOARD 因超时退出
            self._publish_setpoint(self.current_target[0], self.current_target[1], self.current_target[2])
            try:
                self.srv_mode(custom_mode=mode)
            except Exception as e:
                rospy.logwarn(f"set_mode error: {e}")
            rate.sleep()
        return self.current_state.mode == mode

    def _arm(self, arm: bool = True, timeout: float = 10.0) -> bool:
        rospy.loginfo(f"[FlightExecutor] ARM={arm}")
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < timeout:
            if self.current_state.armed == arm:
                return True
            # 持续发布当前目标 setpoint
            self._publish_setpoint(self.current_target[0], self.current_target[1], self.current_target[2])
            try:
                self.srv_arm(arm)
            except Exception as e:
                rospy.logwarn(f"arm error: {e}")
            rate.sleep()
        return self.current_state.armed == arm

    def _set_mpca_mode(self, mode: int) -> bool:
        rospy.loginfo(f"[FlightExecutor] 设置 MPCA_MODE={mode}")
        try:
            pv = ParamValue()
            pv.integer = mode
            resp = self.srv_param_set(param_id="MPCA_MODE", value=pv)
            return resp.success
        except Exception as e:
            rospy.logerr(f"set MPCA_MODE error: {e}")
            return False

    def _send_morph_angle(self, angle_rad: float) -> bool:
        rospy.loginfo(f"[FlightExecutor] 发送变形角度: {angle_rad:.3f} rad")
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

    def _check_safety(self) -> bool:
        if self.current_pos is None:
            return True
        safe = self.safe_space.contains(
            self.current_pos.x, self.current_pos.y, self.current_pos.z,
            margin=self.safety_margin
        )
        if not safe:
            rospy.logerr(
                f"[SAFETY] 超出安全空间! pos=({self.current_pos.x:.2f}, "
                f"{self.current_pos.y:.2f}, {self.current_pos.z:.2f})"
            )
        return safe

    def _wait_pos(self, timeout: float = 10.0):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while self.current_pos is None and (rospy.Time.now() - start).to_sec() < timeout:
            rate.sleep()
        if self.current_pos is None:
            raise RuntimeError("无法获取当前位置")

    def _interp(self, p0, p1, speed: float) -> List[Tuple[float, float, float]]:
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(p0, p1)))
        if dist < 1e-6:
            return [p1]
        n = max(1, int(dist / (speed / self.setpoint_rate)))
        return [
            tuple(p0[i] + (p1[i] - p0[i]) * t / n for i in range(3))
            for t in range(1, n + 1)
        ]

    def _clip_xy(self, x: float, y: float) -> Tuple[float, float]:
        """将水平坐标裁剪到安全区域内。"""
        return self.safe_space.clip_point(x, y, self.home_xy, margin=self.safety_margin)

    def _go_to_point(self, x: float, y: float, z: float, speed: float = 0.3, yaw: float = 0.0,
                     timeout: float = 15.0, pos_tol: float = 0.10, z_tol: float = 0.10):
        """
        平滑移动到目标点，并等待到达指定容差范围内。
        """
        # 目标点也做安全裁剪（仅在水平面）
        x, y = self._clip_xy(x, y)
        self.current_target = (x, y, z)

        if self.current_pos is None:
            self._publish_setpoint(x, y, z, yaw)
            return

        p0 = (self.current_pos.x, self.current_pos.y, self.current_pos.z)
        p1 = (x, y, z)
        pts = self._interp(p0, p1, speed)

        rate = rospy.Rate(self.setpoint_rate)
        for pt in pts:
            if self.should_stop:
                return
            if not self._check_safety():
                raise RuntimeError("Safety boundary violated during goto")
            self.current_target = (pt[0], pt[1], pt[2])
            self._publish_setpoint(pt[0], pt[1], pt[2], yaw)
            rate.sleep()

        # 到达目标点后继续发布 setpoint，直到进入容差范围或超时
        self.current_target = (x, y, z)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < timeout:
            if self.should_stop:
                return
            if not self._check_safety():
                raise RuntimeError("Safety boundary violated during goto")
            if self.current_pos is not None:
                dx = self.current_pos.x - x
                dy = self.current_pos.y - y
                dz = self.current_pos.z - z
                horiz_ok = math.hypot(dx, dy) <= pos_tol
                vert_ok = abs(dz) <= z_tol
                if horiz_ok and vert_ok:
                    rospy.loginfo(
                        f"[FlightExecutor] 到达目标点 ({x:.2f}, {y:.2f}, {z:.2f}), "
                        f"误差=({math.hypot(dx,dy):.3f}, {abs(dz):.3f})"
                    )
                    return
            self._publish_setpoint(x, y, z, yaw)
            rate.sleep()

        rospy.logwarn(
            f"[FlightExecutor] 到达目标点超时 ({timeout}s): ({x:.2f}, {y:.2f}, {z:.2f}), "
            f"当前位置=({self.current_pos.x:.2f}, {self.current_pos.y:.2f}, {self.current_pos.z:.2f})"
        )

    def _takeoff(self, x: float, y: float, z: float):
        """
        垂直起飞到目标高度。
        水平位置保持在 home，不引入水平移动；垂直缓慢上升，给足 Z 轴响应时间。
        """
        rospy.loginfo(f"[FlightExecutor] 起飞到 ({x:.2f}, {y:.2f}, {z:.2f})")
        # 安全裁剪水平目标
        x, y = self._clip_xy(x, y)
        self.current_target = (x, y, z)

        rate = rospy.Rate(self.setpoint_rate)

        start_z = self.current_pos.z if self.current_pos is not None else self.ground_z
        dz_total = z - start_z
        if abs(dz_total) < 0.05:
            rospy.loginfo("[FlightExecutor] 已经在目标高度附近")
            return

        # 给足上升时间：按 takeoff_speed 计算，且至少 8s（考虑 PX4 起飞 ramp 较慢）
        ascent_time = max(8.0, abs(dz_total) / max(0.05, self.takeoff_speed))
        n_steps = max(1, int(ascent_time * self.setpoint_rate))
        rospy.loginfo(
            f"[FlightExecutor] 起飞距离 {dz_total:.2f}m, 预计时间 {ascent_time:.1f}s"
        )

        for i in range(1, n_steps + 1):
            if self.should_stop:
                return
            if not self._check_safety():
                raise RuntimeError("Safety boundary violated during takeoff")
            zi = start_z + dz_total * i / n_steps
            self.current_target = (x, y, zi)
            self._publish_setpoint(x, y, zi)
            rate.sleep()

        # 到达目标高度后继续发布 setpoint，等待一段时间让 Z 轴收敛（不阻塞失败）
        self.current_target = (x, y, z)
        rospy.loginfo("[FlightExecutor] 起飞段结束，在目标高度稳定 3s")
        settle_start = rospy.Time.now()
        while (rospy.Time.now() - settle_start).to_sec() < 3.0:
            if self.should_stop:
                return
            if not self._check_safety():
                raise RuntimeError("Safety boundary violated during takeoff")
            if self.current_pos is not None:
                err = abs(self.current_pos.z - z)
                rospy.loginfo_throttle(1.0, f"[FlightExecutor] 起飞高度误差 {err:.3f}m")
            self._publish_setpoint(x, y, z)
            rate.sleep()

    # ------------------------------------------------------------------
    # 轨迹生成
    # ------------------------------------------------------------------
    def generate_trajectory(self, name: str) -> List[Tuple[float, float, float]]:
        """生成指定轨迹的所有航点（已裁剪到安全区内）。"""
        cx, cy = self.home_xy
        base_z = self.ground_z + self.flight_height
        traj = self._generate_raw_trajectory(name, cx, cy, base_z)

        # 安全裁剪并验证
        clipped = []
        for x, y, z in traj:
            sx, sy = self._clip_xy(x, y)
            # z 方向也做限制
            sz = max(self.ground_z, min(self.ground_z + self.flight_height, z))
            clipped.append((sx, sy, sz))

        self._verify_trajectory(clipped)
        return clipped

    def _verify_trajectory(self, pts: List[Tuple[float, float, float]]):
        """检查所有航点均在安全区内，否则报错。"""
        for x, y, z in pts:
            if not self.safe_space.contains(x, y, z, margin=self.safety_margin):
                d = self.safe_space.distance_to_boundary(x, y)
                raise RuntimeError(
                    f"轨迹航点 ({x:.3f}, {y:.3f}, {z:.3f}) 超出安全空间，到边界距离 {d:.3f} m"
                )

    def _generate_raw_trajectory(self, name: str, cx: float, cy: float,
                                 base_z: float) -> List[Tuple[float, float, float]]:
        """生成原始轨迹点（未裁剪）。"""

        # 轨迹尺寸上限：不超过可用半径，并留出额外余量
        R = min(0.30, self.max_safe_radius)
        square_half = min(0.20, self.max_safe_radius / math.sqrt(2))
        step_delta = min(0.22, self.max_safe_radius / math.sqrt(2))

        if name == "hover":
            return [(cx, cy, base_z)]

        elif name == "takeoff_land":
            p0 = (cx, cy, self.ground_z + 0.3)
            p1 = (cx, cy, base_z)
            return self._interp(p0, p1, self.takeoff_speed) + \
                   [p1] * int(self.hover_time * self.setpoint_rate) + \
                   self._interp(p1, p0, self.land_speed)

        elif name == "square_small":
            corners = [
                (cx + square_half, cy + square_half, base_z),
                (cx + square_half, cy - square_half, base_z),
                (cx - square_half, cy - square_half, base_z),
                (cx - square_half, cy + square_half, base_z),
                (cx + square_half, cy + square_half, base_z),
            ]
            pts = []
            for i in range(len(corners) - 1):
                pts.extend(self._interp(corners[i], corners[i + 1], self.cruise_speed))
            return pts

        elif name == "circle_small":
            n = 100
            pts = []
            for i in range(n + 1):
                theta = 2.0 * math.pi * i / n
                x = cx + R * math.cos(theta)
                y = cy + R * math.sin(theta)
                pts.append((x, y, base_z))
            return pts

        elif name == "figure8_small":
            n = 160
            pts = []
            for i in range(n + 1):
                t = 2.0 * math.pi * i / n
                x = cx + R * math.sin(t)
                y = cy + R * math.sin(t) * math.cos(t)
                pts.append((x, y, base_z))
            return pts

        elif name == "step_x":
            p0 = (cx, cy, base_z)
            p1 = (cx + step_delta, cy, base_z)
            p2 = (cx - step_delta, cy, base_z)
            hold = int(3.0 * self.setpoint_rate)
            return [p0] * hold + \
                   self._interp(p0, p1, self.cruise_speed) + [p1] * hold + \
                   self._interp(p1, p2, self.cruise_speed) + [p2] * hold + \
                   self._interp(p2, p0, self.cruise_speed) + [p0] * hold

        elif name == "step_xy":
            p0 = (cx, cy, base_z)
            p1 = (cx + step_delta, cy + step_delta, base_z)
            p2 = (cx - step_delta, cy - step_delta, base_z)
            hold = int(3.0 * self.setpoint_rate)
            return [p0] * hold + \
                   self._interp(p0, p1, self.cruise_speed) + [p1] * hold + \
                   self._interp(p1, p2, self.cruise_speed) + [p2] * hold + \
                   self._interp(p2, p0, self.cruise_speed) + [p0] * hold

        elif name == "morph_circle":
            return self._generate_raw_trajectory("circle_small", cx, cy, base_z)

        else:
            raise ValueError(f"未知轨迹: {name}")

    # ------------------------------------------------------------------
    # 服务处理函数
    # ------------------------------------------------------------------
    def _handle_start_flight(self, req: Trigger) -> TriggerResponse:
        if self.is_running:
            return TriggerResponse(success=False, message="已有飞行任务在执行")

        traj = rospy.get_param("/optimize/trajectory", "circle_small")
        mode = rospy.get_param("/optimize/mode", self.default_mode)

        self.current_trajectory = traj
        self.current_mode = mode
        self.should_stop = False
        self.is_running = True

        self.flight_thread = threading.Thread(target=self._run_flight, args=(traj, mode))
        self.flight_thread.start()

        return TriggerResponse(success=True, message=f"启动飞行: {traj}, mode={mode}")

    def _handle_land(self, req: Trigger) -> TriggerResponse:
        if self.is_running:
            self.should_stop = True
            if self.flight_thread:
                self.flight_thread.join(timeout=2.0)

        threading.Thread(target=self._run_land).start()
        return TriggerResponse(success=True, message="启动降落")

    def _handle_emergency_stop(self, req: Trigger) -> TriggerResponse:
        self.should_stop = True
        if self.is_running and self.flight_thread:
            self.flight_thread.join(timeout=2.0)

        try:
            self.srv_mode(custom_mode="STABILIZED")
            self.srv_arm(False)
        except Exception as e:
            rospy.logerr(f"急停错误: {e}")
            return TriggerResponse(success=False, message=f"急停错误: {e}")

        return TriggerResponse(success=True, message="已执行急停: STABILIZED + DISARM")

    # ------------------------------------------------------------------
    # 主飞行流程
    # ------------------------------------------------------------------
    def _run_flight(self, trajectory_name: str, mode: int):
        rospy.loginfo(f"[FlightExecutor] 开始执行任务: {trajectory_name}, MPCA_MODE={mode}")
        rate = rospy.Rate(self.setpoint_rate)

        try:
            # 等待连接
            rospy.loginfo("[FlightExecutor] 等待 FCU 连接...")
            timeout = rospy.Time.now() + rospy.Duration(10)
            while not rospy.is_shutdown() and not self.current_state.connected and rospy.Time.now() < timeout:
                rate.sleep()
            if not self.current_state.connected:
                raise RuntimeError("FCU 未连接")

            # 等待位置
            self._wait_pos()
            rospy.loginfo(f"[FlightExecutor] 当前位置: ({self.current_pos.x:.2f}, {self.current_pos.y:.2f}, {self.current_pos.z:.2f})")

            # 检查初始位置安全（失败时直接返回，不切模式/不解锁，保留用户手动控制权）
            if not self.safe_space.contains(self.current_pos.x, self.current_pos.y, self.current_pos.z, margin=0):
                rospy.logerr(f"[FlightExecutor] 初始位置不在安全空间内: ({self.current_pos.x:.2f}, {self.current_pos.y:.2f}, {self.current_pos.z:.2f})")
                rospy.logerr(f"[FlightExecutor] 请确认无人机已放置在 home 点 {self.home_xy} 附近，且高度在地面 ±5cm 范围内")
                self.is_running = False
                return

            # 设置当前目标为当前位置
            self.current_target = (self.current_pos.x, self.current_pos.y, self.current_pos.z)

            # 设置 MPCA_MODE
            self._set_mpca_mode(mode)

            # 预发布 setpoint 2秒，让飞控稳定接收
            rospy.loginfo("[FlightExecutor] 预发布 setpoint 2秒...")
            pre_pub_start = rospy.Time.now()
            while (rospy.Time.now() - pre_pub_start).to_sec() < 2.0:
                self._publish_setpoint(self.current_target[0], self.current_target[1], self.current_target[2])
                rate.sleep()

            # 切 OFFBOARD（setpoint 持续发布，避免超时退出）
            if not self._set_mode("OFFBOARD"):
                raise RuntimeError("无法切换到 OFFBOARD")

            # OFFBOARD 激活后继续稳定发布当前位置 setpoint，确保飞控认可 setpoint
            rospy.loginfo("[FlightExecutor] OFFBOARD 已激活，继续稳定 setpoint 1s...")
            for _ in range(int(1.0 * self.setpoint_rate)):
                self._publish_setpoint(self.current_target[0], self.current_target[1], self.current_target[2])
                rate.sleep()

            # ARM
            if not self._arm(True):
                raise RuntimeError("无法 ARM")

            # ARM 后继续稳定发布当前位置 setpoint，避免解锁后因 setpoint 突变导致 OFFBOARD 退出
            rospy.loginfo("[FlightExecutor] ARM 完成，继续稳定 setpoint 1s...")
            for _ in range(int(1.0 * self.setpoint_rate)):
                self._publish_setpoint(self.current_target[0], self.current_target[1], self.current_target[2])
                rate.sleep()

            # 起飞
            self._takeoff(self.home[0], self.home[1], self.home[2])

            # 悬停
            rospy.loginfo(f"[FlightExecutor] 悬停 {self.hover_time}s")
            hover_start = rospy.Time.now()
            self.current_target = (self.home[0], self.home[1], self.home[2])
            while (rospy.Time.now() - hover_start).to_sec() < self.hover_time:
                if self.should_stop:
                    break
                if not self._check_safety():
                    raise RuntimeError("Safety boundary violated during hover")
                self._publish_setpoint(self.home[0], self.home[1], self.home[2])
                rate.sleep()

            # 生成并执行轨迹
            waypoints = self.generate_trajectory(trajectory_name)
            rospy.loginfo(f"[FlightExecutor] 执行轨迹: {len(waypoints)} 个航点")

            for idx, (x, y, z) in enumerate(waypoints):
                if self.should_stop:
                    rospy.loginfo("[FlightExecutor] 收到停止指令")
                    break
                if not self._check_safety():
                    raise RuntimeError("Safety boundary violated during trajectory")

                self.current_target = (x, y, z)
                self._publish_setpoint(x, y, z)

                # morph_circle 轨迹：在 1/4 和 3/4 处变形
                if trajectory_name == "morph_circle":
                    if idx == len(waypoints) // 4:
                        self._send_morph_angle(-0.35)
                    elif idx == 3 * len(waypoints) // 4:
                        self._send_morph_angle(0.0)

                rate.sleep()

            # 返回 home
            rospy.loginfo("[FlightExecutor] 返回 home")
            self._go_to_point(self.home[0], self.home[1], self.home[2], speed=self.cruise_speed)

            # 自动降落
            self._run_land()

        except RuntimeError as e:
            rospy.logerr(f"[FlightExecutor] 任务错误: {e}")
            self._emergency_land()
        except Exception as e:
            rospy.logerr(f"[FlightExecutor] 未知错误: {e}")
            self._emergency_land()
        finally:
            self.is_running = False
            self.should_stop = False

    # ------------------------------------------------------------------
    # 降落流程（重点解决 LAND 模式失败问题）
    # ------------------------------------------------------------------
    def _run_land(self):
        """
        可靠的自动降落流程：
        1. 先水平回到 home 上方
        2. 通过 setpoint 缓慢下降到离地约 0.05m
        3. 尝试使用 mavros/cmd/land 触发 PX4 LAND
        4. 如果 LAND 不可用，继续用 setpoint 控制落地并 DISARM
        """
        rospy.loginfo("[FlightExecutor] 开始降落")
        rate = rospy.Rate(self.setpoint_rate)

        # 检查当前位置安全
        if self.current_pos is not None and not self.safe_space.contains(self.current_pos.x, self.current_pos.y, self.current_pos.z, margin=0):
            rospy.logerr(f"[FlightExecutor] 当前位置不在安全空间内，中止自动降落: ({self.current_pos.x:.2f}, {self.current_pos.y:.2f}, {self.current_pos.z:.2f})")
            self.is_running = False
            return

        try:
            # 步骤 1：水平回到 home 上方
            if self.current_pos is not None:
                self._go_to_point(self.home[0], self.home[1], self.current_pos.z, speed=self.cruise_speed, yaw=self.current_yaw)

            # 步骤 2：缓慢下降到接近地面 (ground_z + 0.05m)
            final_land_z = self.ground_z + 0.05
            rospy.loginfo(f"[FlightExecutor] 下降到最终高度 {final_land_z:.2f}m")
            self._go_to_point(self.home[0], self.home[1], final_land_z, speed=self.land_speed, z_tol=0.08, yaw=self.current_yaw)

            # 在最终高度悬停一小段时间，让飞机稳定
            self.current_target = (self.home[0], self.home[1], final_land_z)
            stable_start = rospy.Time.now()
            while (rospy.Time.now() - stable_start).to_sec() < 2.0:
                if self.should_stop:
                    return
                self._publish_setpoint(self.home[0], self.home[1], final_land_z, yaw=self.current_yaw)
                rate.sleep()

            # 步骤 3：尝试使用 mavros/cmd/land 触发 PX4 LAND
            rospy.loginfo("[FlightExecutor] 尝试触发 PX4 LAND...")
            land_ok = False
            try:
                resp = self.srv_land(min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=0.0)
                if resp.success:
                    rospy.loginfo("[FlightExecutor] LAND 命令已发送，等待降落...")
                    land_ok = True
                else:
                    rospy.logwarn(f"[FlightExecutor] LAND 命令被拒绝: result={resp.result}")
            except Exception as e:
                rospy.logwarn(f"[FlightExecutor] LAND 服务调用失败: {e}")

            if land_ok:
                # 等待降落完成（已 disarm 或高度接近地面）
                land_start = rospy.Time.now()
                while (rospy.Time.now() - land_start).to_sec() < 30:
                    if not self.current_state.armed:
                        rospy.loginfo("[FlightExecutor] 已自动 DISARM")
                        return
                    if self.current_pos is not None and self.current_pos.z < self.ground_z + 0.12:
                        rospy.loginfo("[FlightExecutor] 高度接近地面，执行 DISARM")
                        self._arm(False)
                        return
                    rate.sleep()

                rospy.logwarn("[FlightExecutor] LAND 30s 后仍未降落，强制 DISARM")
                self._arm(False)
            else:
                # Fallback: 继续用 setpoint 控制，缓慢降低高度直到触地
                rospy.logwarn("[FlightExecutor] LAND 不可用，使用 setpoint 继续下降")
                self.current_target = (self.home[0], self.home[1], self.ground_z + 0.02)
                touch_start = rospy.Time.now()
                while (rospy.Time.now() - touch_start).to_sec() < 10.0:
                    if not self.current_state.armed:
                        rospy.loginfo("[FlightExecutor] 已自动 DISARM")
                        return
                    if self.current_pos is not None:
                        # 如果高度基本不再变化且接近地面，认为已着陆
                        if self.current_pos.z < self.ground_z + 0.10:
                            rospy.loginfo("[FlightExecutor] 检测到着陆，执行 DISARM")
                            self._arm(False)
                            return
                    self._publish_setpoint(self.home[0], self.home[1], self.ground_z + 0.02, yaw=self.current_yaw)
                    rate.sleep()

                rospy.logwarn("[FlightExecutor] setpoint 降落超时，切 POSCTL 并 DISARM")
                self._set_mode("POSCTL")
                self._arm(False)

        except Exception as e:
            rospy.logerr(f"[FlightExecutor] 降落错误: {e}")
            self._emergency_land()

    def _emergency_land(self):
        """紧急降落：切 POSCTL，DISARM"""
        rospy.logerr("[FlightExecutor] 执行紧急降落")
        try:
            self._set_mode("POSCTL")
            self._arm(False)
        except Exception as e:
            rospy.logerr(f"[FlightExecutor] 紧急降落错误: {e}")

    def run(self):
        rospy.spin()


def main():
    executor = FlightExecutor()
    executor.run()


if __name__ == "__main__":
    main()
