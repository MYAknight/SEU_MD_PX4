#!/usr/bin/env python3
"""
vision_approach_test.py — 视觉引导接近 + 接触检测 SITL 测试

流程：
    1. 起飞并悬停到 HOVER_Z
    2. 纯视觉 YAW 对齐：根据 /yolo/pixel_error 控制 yaw_rate，直到 /yolo/yaw_aligned == True
    3. 展开机臂到 ARM_EXPAND_ANGLE
    4. 前向接近 pole，同时持续根据 pixel_error 修正 YAW
    5. 若视觉丢失超过阈值，进入 BLIND_PUSH：沿前进方向继续推送，不再依赖视觉
    6. 当 PX4 报告 CONTACT（debug_float_array phase >= 2）或超时/距离过远时停止
    6. 保存 CSV 日志

用法：
    python3 vision_approach_test.py
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import rospy
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, DebugValue, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, ParamSet
from mavros_msgs.msg import ParamValue
from std_msgs.msg import Bool, Float32, String, Float64

try:
    from gazebo_msgs.msg import ModelStates
    GAZEBO_AVAILABLE = True
except Exception:
    GAZEBO_AVAILABLE = False


class YawAlignController:
    """带低通滤波、微分阻尼和相机水平偏移补偿的 YAW 对齐 PD 控制器。"""

    def __init__(self, Kp=1.0, Kd=0.15, max_yaw_rate=0.25, deadzone=0.08, alpha=0.35, offset_norm=0.0):
        self.Kp = Kp
        self.Kd = Kd
        self.max_yaw_rate = max_yaw_rate
        self.deadzone = deadzone
        self.alpha = alpha
        self.offset_norm = offset_norm  # 相机中心偏移量（已归一化到 [-1,1]）
        self.err_filt = 0.0
        self.err_prev = 0.0
        self.last_t = None

    def set_offset(self, offset_norm):
        self.offset_norm = offset_norm

    def compute(self, pixel_error):
        """输入 pixel_error ∈ [-1,1]，输出 yaw_rate (rad/s) 与 aligned 标志。

        若相机安装位置相对机体中心有水平偏移（例如实机相机偏右），
        通过 offset_norm 进行补偿：corrected_error = pixel_error + offset_norm。
        """
        corrected_error = pixel_error + self.offset_norm
        # 低通滤波（无检测时 pixel_error 可能为 0，此时不过度响应）
        self.err_filt = self.alpha * corrected_error + (1.0 - self.alpha) * self.err_filt

        if abs(self.err_filt) < self.deadzone:
            self.err_prev = self.err_filt
            return 0.0, True

        now = time.time()
        dt = 0.05 if self.last_t is None else max(now - self.last_t, 0.001)
        derr = (self.err_filt - self.err_prev) / dt
        self.last_t = now
        self.err_prev = self.err_filt

        # PD 控制：误差大时降低增益，抑制震荡
        yaw_rate = -(self.Kp * self.err_filt + self.Kd * derr) * self.max_yaw_rate
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        return yaw_rate, False


class VisionApproachTest:

    # ---------- 默认参数（可通过 ROS params 覆盖） ----------
    HOVER_Z = 2.5
    HOVER_TIME = 5.0
    ALIGN_START_DELAY = 3.0       # s，起飞悬停后稳定一段时间再开始对齐
    ALIGN_TIMEOUT = 40.0
    Kp = 2.0                      # YAW 对齐比例增益（配合 max_yaw_rate 做限速）
    MAX_YAW_RATE = 0.12           # rad/s，YOLO 对齐最大 yaw 速度（实机安全）
    CAMERA_OFFSET_X = 0.0         # px，相机光心相对图像中心的水平偏移（正=偏右）
    IMAGE_WIDTH = 640.0           # px，与 YOLO 输入图像宽度一致
    ARM_EXPAND_ANGLE = -0.30      # rad, 与 pole_collision.py 一致
    ARM_EXPAND_DURATION = 3.0     # s
    ARM_RETRACT_DURATION = 3.0    # s
    APPROACH_SPEED = 0.05
    APPROACH_TIMEOUT = 150.0
    MAX_APPROACH_DIST = 7.0
    BLIND_PUSH_LOST_THR = 0.6   # s，连续无检测即进入盲推
    CONTACT_HOLD_DURATION = 5.0   # s，CONTACT 后先锁定位置
    POST_RETRACT_HOLD_DURATION = 3.0  # s，机臂收回后继续锁定位置
    MONITOR_TIME = 5.0                # s，disarm 后监测是否稳定挂在柱上
    CONTACT_WAIT_TIMEOUT = 30.0       # s，等待机臂收拢/稳定（地面站 CONTACT_WAIT）
    ARM_RETRACTED_THRESHOLD = -0.05   # rad，机臂视为已收拢
    THROTTLE_RAMP_DURATION = 5.0      # s，油门下降阶段时长（与地面站一致）
    THROTTLE_RAMP_DELTA_Z = 0.5       # m，油门下降阶段降低高度（用于降低推力）
    POLE_X = 5.0
    POLE_Y = 0.0
    RATE_HZ = 20.0

    def __init__(self, output_prefix='vision_approach'):
        self.output_prefix = output_prefix
        self.records = []

        rospy.init_node('vision_approach_test', anonymous=True)

        # 参数
        self.hover_z = rospy.get_param('~hover_z', self.HOVER_Z)
        self.hover_time = rospy.get_param('~hover_time', self.HOVER_TIME)
        self.align_start_delay = rospy.get_param('~align_start_delay', self.ALIGN_START_DELAY)
        self.align_timeout = rospy.get_param('~align_timeout', self.ALIGN_TIMEOUT)
        self.arm_expand_angle = rospy.get_param('~arm_expand_angle', self.ARM_EXPAND_ANGLE)
        self.arm_expand_duration = rospy.get_param('~arm_expand_duration', self.ARM_EXPAND_DURATION)
        self.arm_retract_duration = rospy.get_param('~arm_retract_duration', self.ARM_RETRACT_DURATION)
        self.approach_speed = rospy.get_param('~approach_speed', self.APPROACH_SPEED)
        self.approach_timeout = rospy.get_param('~approach_timeout', self.APPROACH_TIMEOUT)
        self.max_approach_dist = rospy.get_param('~max_approach_dist', self.MAX_APPROACH_DIST)
        self.blind_push_lost_thr = rospy.get_param('~blind_push_lost_thr', self.BLIND_PUSH_LOST_THR)
        self.contact_hold_duration = rospy.get_param('~contact_hold_duration', self.CONTACT_HOLD_DURATION)
        self.post_retract_hold_duration = rospy.get_param('~post_retract_hold_duration', self.POST_RETRACT_HOLD_DURATION)
        self.monitor_time = rospy.get_param('~monitor_time', self.MONITOR_TIME)
        self.contact_wait_timeout = rospy.get_param('~contact_wait_timeout', self.CONTACT_WAIT_TIMEOUT)
        self.arm_retracted_threshold = rospy.get_param('~arm_retracted_threshold', self.ARM_RETRACTED_THRESHOLD)
        self.throttle_ramp_duration = rospy.get_param('~throttle_ramp_duration', self.THROTTLE_RAMP_DURATION)
        self.throttle_ramp_delta_z = rospy.get_param('~throttle_ramp_delta_z', self.THROTTLE_RAMP_DELTA_Z)
        self.pole_x = rospy.get_param('~pole_x', self.POLE_X)
        self.pole_y = rospy.get_param('~pole_y', self.POLE_Y)
        self.rate_hz = rospy.get_param('~rate_hz', self.RATE_HZ)

        # YAW 控制参数
        self.Kp = rospy.get_param('~Kp', self.Kp)
        self.Kd = rospy.get_param('~Kd', 0.15)
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', self.MAX_YAW_RATE)
        self.deadzone = rospy.get_param('~deadzone', 0.08)
        self.yaw_alpha = rospy.get_param('~yaw_alpha', 0.35)
        self.camera_offset_x = rospy.get_param('~camera_offset_x', self.CAMERA_OFFSET_X)
        self.image_width = rospy.get_param('~image_width', self.IMAGE_WIDTH)
        offset_norm = self.camera_offset_x / (self.image_width / 2.0)

        self.yaw_controller = YawAlignController(self.Kp, self.Kd, self.max_yaw_rate, self.deadzone, self.yaw_alpha, offset_norm)

        # 状态
        self.current_state = None
        self.current_pose = None
        self.yaw_aligned = False
        self.pixel_error = 0.0
        self.detections_info = None
        self.det_count = 0
        self.px4_perching_phase = 0
        self.contact_detected = False
        self.actual_arm_angle = None  # 来自 Gazebo 插件的实际臂角
        self.perching_status = False  # Gazebo 插件的自动固定状态

        # ROS 订阅
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/yolo/yaw_aligned', Bool, self._aligned_cb)
        rospy.Subscriber('/yolo/pixel_error', Float32, self._error_cb)
        rospy.Subscriber('/yolo/detections_info', String, self._detections_cb)
        rospy.Subscriber('/mavros/debug_value/debug_float_array', DebugValue, self._debug_cb)
        rospy.Subscriber('/huaqiccc/arm_actual_angle', Float64, self._arm_actual_cb)
        rospy.Subscriber('/huaqiccc/perching_status', Bool, self._perching_status_cb)

        # ROS 发布：统一使用 setpoint_raw/local
        self.sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.arm_pub = rospy.Publisher('/huaqiccc/arm_angle', Float64, queue_size=1)
        self.fix_pub = rospy.Publisher('/huaqiccc/fix_perching', Bool, queue_size=1, latch=True)

        # Gazebo 真值位姿（disarm 后 EKF 会漂移，用真值判断栖落稳定性）
        self._gazebo_pose = None
        if GAZEBO_AVAILABLE:
            try:
                rospy.Subscriber('/gazebo/model_states', ModelStates, self._gazebo_state_cb)
            except Exception as e:
                rospy.logwarn(f'[VisionApproachTest] 无法订阅 /gazebo/model_states: {e}')

        # ROS 服务
        rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
        rospy.wait_for_service('/mavros/set_mode', timeout=10.0)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # MAVROS 参数设置（用于 MPCA_PC_EN 等）
        self.param_set_client = None
        try:
            rospy.wait_for_service('/mavros/param/set', timeout=5.0)
            self.param_set_client = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            rospy.loginfo('[VisionApproachTest] /mavros/param/set 已连接')
        except Exception as e:
            rospy.logwarn(f'[VisionApproachTest] /mavros/param/set 不可用: {e}')

        # 31440 morph command（实机方式；SITL 中可选）
        self.cmd_long_srv = None
        try:
            rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
            self.cmd_long_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
            rospy.loginfo('[VisionApproachTest] /mavros/cmd/command 已连接，将发送 31440')
        except Exception as e:
            rospy.logwarn(f'[VisionApproachTest] /mavros/cmd/command 不可用: {e}')

        rospy.loginfo('[VisionApproachTest] 初始化完成')
        rospy.loginfo(f'  hover_z={self.hover_z}, approach_speed={self.approach_speed}')
        rospy.loginfo(f'  YAW Kp={self.Kp}, Kd={self.Kd}, max_rate={self.max_yaw_rate}, deadzone={self.deadzone}, alpha={self.yaw_alpha}')
        rospy.loginfo(f'  camera_offset_x={self.camera_offset_x:.1f} px (norm={offset_norm:.4f}), image_width={self.image_width}')
        rospy.loginfo(f'  arm_expand_angle={self.arm_expand_angle} rad, expand={self.arm_expand_duration}s, retract={self.arm_retract_duration}s')
        rospy.loginfo(f'  align_start_delay={self.align_start_delay}s, blind_push_lost_thr={self.blind_push_lost_thr}s')
        rospy.loginfo(f'  contact_hold={self.contact_hold_duration}s, post_retract_hold={self.post_retract_hold_duration}s')

    # ---------- 回调 ----------

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg.pose

    def _aligned_cb(self, msg):
        self.yaw_aligned = msg.data

    def _error_cb(self, msg):
        self.pixel_error = msg.data

    def _gazebo_state_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'huaqiccc':
                self._gazebo_pose = msg.pose[i]
                break

    def _detections_cb(self, msg):
        self.detections_info = msg
        try:
            self.det_count = len(json.loads(msg.data).get('targets', []))
        except Exception:
            self.det_count = 0

    def _arm_actual_cb(self, msg):
        self.actual_arm_angle = float(msg.data)

    def _perching_status_cb(self, msg):
        self.perching_status = bool(msg.data)

    def _debug_cb(self, msg):
        if msg.name == 'perch' and len(msg.data) >= 6:
            self.px4_perching_phase = int(msg.data[1])
            if self.px4_perching_phase >= 2 and not self.contact_detected:
                self.contact_detected = True
                rospy.loginfo(f'[TELEM] PX4 CONTACT detected, phase={self.px4_perching_phase}')

    # ---------- 工具函数 ----------

    def _make_setpoint(self, x, y, z, yaw=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        """Build PositionTarget for /mavros/setpoint_raw/local."""
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        has_vel = abs(vx) > 1e-6 or abs(vy) > 1e-6
        has_yaw_rate = abs(yaw_rate) > 1e-6

        if has_vel and has_yaw_rate:
            msg.type_mask = (
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW
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
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW
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
        if not has_yaw_rate:
            msg.yaw = yaw
        return msg

    def _send_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.sp_pub.publish(self._make_setpoint(x, y, z, yaw, vx, vy, yaw_rate))

    def _send_arm_angle_ros(self, angle):
        if self.arm_pub is None:
            return
        msg = Float64()
        msg.data = float(angle)
        self.arm_pub.publish(msg)

    def _send_morph_31440(self, angle):
        if self.cmd_long_srv is None:
            return False
        try:
            angle_f = float(angle)
            resp = self.cmd_long_srv(
                broadcast=False,
                command=31440,
                confirmation=0,
                param1=angle_f,
                param2=0.0,
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )
            return getattr(resp, 'success', False)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f'[31440] 发送失败: {e}')
            return False

    def _record(self, phase, t, sp_vx, sp_vy, sp_yaw_rate, note=''):
        act = self.current_pose
        yaw_deg = round(math.degrees(self._quat_to_yaw(act.orientation)), 2) if act else 0.0
        det_count = 0
        if self.detections_info is not None:
            try:
                det_count = len(json.loads(self.detections_info.data).get('targets', []))
            except Exception:
                det_count = 0
        self.records.append({
            'phase': phase,
            'time': round(t, 3),
            'sp_vx': round(sp_vx, 4),
            'sp_vy': round(sp_vy, 4),
            'sp_yaw_rate': round(sp_yaw_rate, 4),
            'pixel_error': round(self.pixel_error, 4),
            'yaw_aligned': 1 if self.yaw_aligned else 0,
            'act_x': round(act.position.x, 5) if act else 0.0,
            'act_y': round(act.position.y, 5) if act else 0.0,
            'act_z': round(act.position.z, 5) if act else 0.0,
            'yaw_deg': yaw_deg,
            'det_count': det_count,
            'px4_phase': self.px4_perching_phase,
            'contact': 1 if self.contact_detected else 0,
            'note': note,
        })

    def _save_csv(self, suffix=''):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(os.path.expanduser('~'), 'huaqiccc_logs')
        os.makedirs(out_dir, exist_ok=True)
        name = f'{self.output_prefix}{suffix}_{ts}.csv'
        out_path = os.path.join(out_dir, name)
        if not self.records:
            rospy.logwarn('[VisionApproachTest] 无记录可保存')
            return None
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.records[0].keys())
            writer.writeheader()
            writer.writerows(self.records)
        rospy.loginfo(f'[SAVE] {out_path}')
        return out_path

    def _set_px4_param_int(self, param_id, value):
        """通过 MAVROS 设置 PX4 整型参数，返回是否成功。"""
        if self.param_set_client is None:
            rospy.logwarn(f'[PARAM] {param_id} 未设置：param/set 不可用')
            return False
        try:
            pv = ParamValue()
            pv.integer = int(value)
            resp = self.param_set_client(param_id=param_id, value=pv)
            success = getattr(resp, 'success', False)
            rospy.loginfo(f'[PARAM] {param_id}={value} success={success}')
            return success
        except Exception as e:
            rospy.logwarn(f'[PARAM] 设置 {param_id} 失败: {e}')
            return False

    def _set_pc_en(self, value):
        return self._set_px4_param_int('MPCA_PC_EN', value)

    @staticmethod
    def _quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _current_yaw(self):
        """获取当前航向；无位姿时返回 0。"""
        if self.current_pose is None:
            return 0.0
        return self._quat_to_yaw(self.current_pose.orientation)

    # ---------- 飞行阶段 ----------

    def _takeoff_and_hover(self):
        rospy.loginfo('[PRE] 预发位置 setpoint...')
        rate = rospy.Rate(self.rate_hz)
        for _ in range(100):
            if rospy.is_shutdown():
                return False
            # 起飞前不指定朝向柱子的 yaw，而是保持当前航向
            self._send_setpoint(z=self.hover_z, yaw=self._current_yaw())
            rate.sleep()

        rospy.loginfo('[MODE] 切换到 OFFBOARD')
        self.set_mode_client(base_mode=0, custom_mode='OFFBOARD')

        rospy.loginfo('[ARM] 解锁')
        self.arming_client(True)

        rospy.loginfo(f'[TAKEOFF] 起飞到 z={self.hover_z}')
        for _ in range(100):
            if rospy.is_shutdown():
                return False
            self._send_setpoint(z=self.hover_z, yaw=self._current_yaw())
            rate.sleep()

        rospy.loginfo(f'[HOVER] 悬停 {self.hover_time}s')
        dt = 1.0 / self.rate_hz
        start = time.time()
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > self.hover_time:
                break
            self._send_setpoint(z=self.hover_z, yaw=self._current_yaw())
            self._record('hover', t, 0.0, 0.0, 0.0)
            time.sleep(dt)

        # 悬停结束后再稳定一段时间，期间保持当前位置/航向，不启动 YOLO 对齐
        if self.align_start_delay > 0.0:
            rospy.loginfo(f'[STABLE] 起飞后稳定 {self.align_start_delay}s，再启动 YAW 对齐')
            start = time.time()
            hold_x = self.current_pose.position.x if self.current_pose else 0.0
            hold_y = self.current_pose.position.y if self.current_pose else 0.0
            hold_yaw = self._current_yaw()
            while not rospy.is_shutdown():
                t = time.time() - start
                if t > self.align_start_delay:
                    break
                self._send_setpoint(x=hold_x, y=hold_y, z=self.hover_z, yaw=hold_yaw)
                self._record('stable', t, 0.0, 0.0, 0.0)
                time.sleep(dt)

        return not rospy.is_shutdown()

    def _wait_yaw_alignment(self):
        """纯视觉 YAW 对齐：完全依赖 /yolo/pixel_error。
        若长时间无检测，则执行慢速扫视（yaw search）直到发现目标。
        """
        rospy.loginfo(f'[ALIGN] 纯视觉 YAW 对齐，超时 {self.align_timeout}s')
        dt = 1.0 / self.rate_hz
        start = time.time()
        aligned_once = False
        align_time = None
        no_det_start = None
        search_rate = self.max_yaw_rate  # rad/s，扫视速度上限与对齐限速一致

        # 对齐开始的位置保持点
        hold_x = self.current_pose.position.x if self.current_pose else 0.0
        hold_y = self.current_pose.position.y if self.current_pose else 0.0

        def _normalize_angle(a):
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            return a

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > self.align_timeout:
                rospy.logwarn('[ALIGN] YAW 对齐超时')
                return False

            # 无检测时先等待，超时则朝 pole 几何方向扫视
            if self.det_count == 0:
                if no_det_start is None:
                    no_det_start = t
                    rospy.logwarn_throttle(2.0, '[ALIGN] 未检测到 pole，等待中...')
                if t - no_det_start > 1.0:
                    yaw_cur = self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else 0.0
                    yaw_to_pole = math.atan2(self.pole_y - hold_y, self.pole_x - hold_x)
                    yaw_err = _normalize_angle(yaw_to_pole - yaw_cur)
                    yaw_rate = max(-search_rate, min(search_rate, yaw_err))
                    note = 'search_geo'
                else:
                    yaw_rate = 0.0
                    note = 'wait_det'
                aligned_now = False
            else:
                no_det_start = None
                yaw_rate, aligned_now = self.yaw_controller.compute(self.pixel_error)
                note = ''

            self._send_setpoint(x=hold_x, y=hold_y, z=self.hover_z, yaw_rate=yaw_rate)
            self._record('align', t, 0.0, 0.0, yaw_rate, note=note)

            if aligned_now and self.yaw_aligned:
                if not aligned_once:
                    rospy.loginfo(f'[ALIGN] ✅ 纯视觉 YAW 对齐完成 (t={t:.1f}s, pixel_error={self.pixel_error:.3f})')
                    aligned_once = True
                    align_time = t
                elif t - align_time > 0.5:
                    break

            time.sleep(dt)

        # 对齐完成后保持当前 yaw（避免后续阶段把机头掰回 0°）
        hold_yaw = self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else 0.0
        self._send_setpoint(x=hold_x, y=hold_y, z=self.hover_z, yaw=hold_yaw)
        return aligned_once

    def _expand_arms_before_approach(self):
        """对齐完成后、接近前，缓慢展开机臂。"""
        rospy.loginfo(f'[ARM] 展开机臂到 {self.arm_expand_angle:.2f} rad，耗时 {self.arm_expand_duration}s')
        dt = 1.0 / self.rate_hz
        start = time.time()
        hold_x = self.current_pose.position.x if self.current_pose else 0.0
        hold_y = self.current_pose.position.y if self.current_pose else 0.0
        hold_yaw = self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else 0.0
        start_angle = 0.0
        last_31440 = start

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > self.arm_expand_duration:
                break
            s = t / self.arm_expand_duration
            s2 = s * s * (3.0 - 2.0 * s)  # smoothstep
            angle = start_angle + (self.arm_expand_angle - start_angle) * s2

            # Gazebo 视觉机臂（高频）
            self._send_arm_angle_ros(angle)

            # 实机 31440（1 Hz）
            if time.time() - last_31440 >= 0.95:
                self._send_morph_31440(angle)
                last_31440 = time.time()

            # 位置保持，防止 OFFBOARD 超时；保持当前 yaw，不要掰回 0°
            self._send_setpoint(x=hold_x, y=hold_y, z=self.hover_z, yaw=hold_yaw)
            self._record('arm_expand', t, 0.0, 0.0, 0.0, note=f'arm_angle={angle:.3f}')
            time.sleep(dt)

        # 最终到位
        self._send_arm_angle_ros(self.arm_expand_angle)
        self._send_morph_31440(self.arm_expand_angle)
        rospy.loginfo('[ARM] 机臂展开完成')

        # 与地面站一致：展开后启用 PX4 接触检测 + setpoint 接管（MPCA_PC_EN=2）
        self._set_pc_en(2)
        return True

    def _approach_until_contact(self):
        rospy.loginfo(f'[APPROACH] 开始接近 pole，速度 {self.approach_speed} m/s')
        dt = 1.0 / self.rate_hz
        start = time.time()
        start_pos = self.current_pose
        if start_pos is None:
            rospy.logerr('[APPROACH] 缺少起始位姿')
            return False, 'no_pose'

        x0 = start_pos.position.x
        y0 = start_pos.position.y
        # 目标：pole 前方 0.15m，便于 PX4 CONTACT 触发
        x1 = self.pole_x + 0.15
        y1 = self.pole_y
        total_dist = math.hypot(x1 - x0, y1 - y0)
        duration = total_dist / self.approach_speed
        duration = min(duration, self.approach_timeout)

        # 盲推状态
        lost_start = None
        in_blind = False
        blind_yaw = self._quat_to_yaw(start_pos.orientation)
        self.push_yaw = blind_yaw

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > self.approach_timeout:
                rospy.logwarn('[APPROACH] 接近超时')
                return False, 'timeout'

            s = min(1.0, t / duration)
            x_sp = x0 + (x1 - x0) * s
            y_sp = y0 + (y1 - y0) * s
            vx = (x1 - x0) / duration
            vy = (y1 - y0) / duration

            # 检测丢失判定：一旦进入盲推即保持（与地面站设计一致），不再因短暂恢复退出
            if self.det_count == 0:
                if lost_start is None:
                    lost_start = t
                elif t - lost_start > self.blind_push_lost_thr and not in_blind:
                    in_blind = True
                    blind_yaw = self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else blind_yaw
                    rospy.logwarn(f'[BLIND_PUSH] 视觉丢失超过 {self.blind_push_lost_thr}s，进入盲推 (yaw={math.degrees(blind_yaw):.1f}°)')

            if in_blind:
                # 盲推：沿 A->B 方向继续位置推送，不再依赖视觉，保持进入盲推时的航向
                self._send_setpoint(x=x_sp, y=y_sp, z=self.hover_z, yaw=blind_yaw)
                note = 'blind_push'
                yaw_rate = 0.0
            else:
                # 纯视觉 YAW 修正：根据当前 pixel_error 持续调整 yaw_rate
                yaw_rate, _ = self.yaw_controller.compute(self.pixel_error)
                # 位置 setpoint + yaw_rate；不再同时发送 vx/vy，避免 PX4 位置/速度混合控制引入耦合震荡
                self._send_setpoint(x=x_sp, y=y_sp, z=self.hover_z, yaw_rate=yaw_rate)
                note = ''

            self._record('approach', t, vx, vy, yaw_rate, note=note)

            # 距离保护
            pose = self.current_pose
            if pose and start_pos:
                dist = math.hypot(
                    pose.position.x - start_pos.position.x,
                    pose.position.y - start_pos.position.y
                )
                if dist > self.max_approach_dist:
                    rospy.logwarn(f'[APPROACH] 距离超过上限 {self.max_approach_dist}m')
                    return False, 'max_dist'

            if self.contact_detected:
                self.push_yaw = blind_yaw if in_blind else self._current_yaw()
                rospy.loginfo(f'[APPROACH] ✅ 接触检测到 (t={t:.1f}s, push_yaw={math.degrees(self.push_yaw):.1f}°)')
                return True, 'contact'

            time.sleep(dt)

        return False, 'shutdown'

    def _hold_zero_velocity(self, duration=2.0, phase='hold'):
        rospy.loginfo(f'[{phase.upper()}] 保持位置 {duration}s')
        dt = 1.0 / self.rate_hz
        start = time.time()
        hold_x = self.current_pose.position.x if self.current_pose else 0.0
        hold_y = self.current_pose.position.y if self.current_pose else 0.0
        hold_yaw = self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else 0.0
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > duration:
                break
            self._send_setpoint(x=hold_x, y=hold_y, z=self.hover_z, yaw=hold_yaw)
            self._record(phase, t, 0.0, 0.0, 0.0)
            time.sleep(dt)

    def _retract_arms_after_contact(self):
        """CONTACT 后收回机臂到 0.0 rad，同时锁定当前位置。"""
        rospy.loginfo(f'[ARM_RETRACT] CONTACT 后收回机臂到 0.0 rad，耗时 {self.arm_retract_duration}s')
        dt = 1.0 / self.rate_hz
        start = time.time()
        hold_x = self.current_pose.position.x if self.current_pose else 0.0
        hold_y = self.current_pose.position.y if self.current_pose else 0.0
        hold_yaw = self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else 0.0
        start_angle = self.arm_expand_angle
        target_angle = 0.0
        last_31440 = start

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > self.arm_retract_duration:
                break
            s = t / self.arm_retract_duration
            s2 = s * s * (3.0 - 2.0 * s)  # smoothstep
            angle = start_angle + (target_angle - start_angle) * s2

            # Gazebo 视觉机臂（高频）
            self._send_arm_angle_ros(angle)

            # 实机 31440（1 Hz）
            if time.time() - last_31440 >= 0.95:
                self._send_morph_31440(angle)
                last_31440 = time.time()

            # CONTACT 后 PX4 已接管，但我们继续发位置 setpoint 保持 OFFBOARD 活跃；锁定当前位置
            self._send_setpoint(x=hold_x, y=hold_y, z=self.hover_z, yaw=hold_yaw)
            self._record('arm_retract', t, 0.0, 0.0, 0.0, note=f'arm_angle={angle:.3f}')
            time.sleep(dt)

        # 最终到位
        self._send_arm_angle_ros(target_angle)
        self._send_morph_31440(target_angle)
        rospy.loginfo('[ARM_RETRACT] 机臂收回完成')
        return True

    def _contact_wait(self, push_yaw=None):
        """与地面站 CONTACT_WAIT 对齐：CONTACT 后等待机臂收拢，期间 PX4 三轴 setpoint 接管生效。"""
        rospy.loginfo('[CONTACT_WAIT] 等待机臂收拢，PX4 三轴 setpoint 接管中...')
        dt = 1.0 / self.rate_hz
        start = time.time()
        hold_x = self.current_pose.position.x if self.current_pose else 0.0
        hold_y = self.current_pose.position.y if self.current_pose else 0.0
        hold_yaw = push_yaw if push_yaw is not None else (self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else 0.0)
        # 机臂收拢指令已执行 3s，再留 1s 稳定余量；无反馈时按时间兜底
        time_fallback = self.arm_retract_duration + 1.0

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > self.contact_wait_timeout:
                rospy.logwarn('[CONTACT_WAIT] 等待机臂收拢超时')
                return False

            # 持续发布当前位置 setpoint 保持 OFFBOARD 链路，PX4 会覆盖为接触点+预载
            self._send_setpoint(x=hold_x, y=hold_y, z=self.hover_z, yaw=hold_yaw)
            note = f'px4_phase={self.px4_perching_phase}'
            if self.actual_arm_angle is not None:
                note += f' arm_actual={self.actual_arm_angle:.3f}'
            note += f' fixed={int(self.perching_status)}'
            self._record('contact_wait', t, 0.0, 0.0, 0.0, note=note)

            if self.perching_status:
                rospy.loginfo('[CONTACT_WAIT] Gazebo 自动固定已触发（机臂收拢 + 低速），视为收拢完成')
                return True
            if self.actual_arm_angle is not None and self.actual_arm_angle >= self.arm_retracted_threshold:
                rospy.loginfo(f'[CONTACT_WAIT] 机臂已收拢 (actual={self.actual_arm_angle:.3f} rad)')
                return True
            if t > time_fallback:
                rospy.loginfo('[CONTACT_WAIT] 已超出收拢指令时间，按时间兜底视为收拢完成')
                return True

            time.sleep(dt)

        return False

    def _throttle_ramp(self):
        """与地面站 THROTTLE_RAMP 对齐：MPCA_PC_EN 2->1 打断 PX4 接管后缓慢降低 z setpoint。"""
        rospy.loginfo('[THROTTLE_RAMP] 打断 PX4 接管 (MPCA_PC_EN=1) 并缓慢降低推力...')
        self._set_pc_en(1)
        dt = 1.0 / self.rate_hz
        start = time.time()
        hold_x = self.current_pose.position.x if self.current_pose else 0.0
        hold_y = self.current_pose.position.y if self.current_pose else 0.0
        hold_yaw = self._quat_to_yaw(self.current_pose.orientation) if self.current_pose else 0.0
        start_z = self.current_pose.position.z if self.current_pose else self.hover_z
        end_z = start_z - self.throttle_ramp_delta_z

        while not rospy.is_shutdown():
            t = time.time() - start
            if t > self.throttle_ramp_duration:
                break
            ratio = t / self.throttle_ramp_duration
            z_sp = start_z + (end_z - start_z) * ratio
            self._send_setpoint(x=hold_x, y=hold_y, z=z_sp, yaw=hold_yaw)
            self._record('throttle_ramp', t, 0.0, 0.0, 0.0, note=f'z_sp={z_sp:.3f}')
            time.sleep(dt)
        return True

    def _send_fix(self, enable=True):
        if self.fix_pub is None:
            return
        msg = Bool()
        msg.data = bool(enable)
        self.fix_pub.publish(msg)
        rospy.loginfo(f'[FIX] fix_perching={enable}')

    def _force_disarm(self):
        if self.cmd_long_srv is None:
            return False
        try:
            resp = self.cmd_long_srv(
                broadcast=False,
                command=400,              # MAV_CMD_COMPONENT_ARM_DISARM
                confirmation=0,
                param1=0.0,               # disarm
                param2=21196.0,           # force
                param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0,
            )
            success = getattr(resp, 'success', False)
            if success:
                rospy.loginfo('[OK] Force disarm accepted')
            else:
                rospy.logwarn(f'[WARN] Force disarm result={getattr(resp, "result", "unknown")}')
            return success
        except Exception as e:
            rospy.logerr(f'[ERROR] Force disarm: {e}')
            return False

    def _monitor_after_disarm(self, duration):
        """force disarm 后监测真值位姿，验证是否稳定栖落。"""
        rospy.loginfo(f'[MONITOR] 电机停止后监测真值位姿 {duration}s')
        dt = 1.0 / self.rate_hz
        start = time.time()
        xs, ys, zs = [], [], []
        while not rospy.is_shutdown():
            t = time.time() - start
            if t > duration:
                break
            pose = self._gazebo_pose
            if pose is not None:
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                xs.append(x)
                ys.append(y)
                zs.append(z)
                self._record('monitor', t, 0.0, 0.0, 0.0, note='gazebo_true')
            else:
                #  fallback：用 MAVROS 位姿（EKF 漂移后不准，仅作记录）
                if self.current_pose:
                    self._record('monitor', t, 0.0, 0.0, 0.0, note='mavros_fallback')
            time.sleep(dt)

        if len(zs) >= 5:
            z_mean = sum(zs) / len(zs)
            z_var = sum((z - z_mean) ** 2 for z in zs) / len(zs)
            z_std = math.sqrt(z_var)
            x_std = 0.0
            if len(xs) >= 5:
                x_mean = sum(xs) / len(xs)
                x_var = sum((x - x_mean) ** 2 for x in xs) / len(xs)
                x_std = math.sqrt(x_var)
            success = z_mean > 1.5 and z_std < 0.05 and x_std < 0.05
            rospy.loginfo(f'[MONITOR] z_mean={z_mean:.2f}m, z_std={z_std:.3f}m, x_std={x_std:.3f}m, n={len(zs)}')
            rospy.loginfo(f'[RESULT] {"SUCCESS" if success else "FAILURE"} - {"remains perched" if success else "fell or oscillates"}')
            return success
        else:
            rospy.logwarn('[MONITOR] 真值位姿样本不足')
            return None

    def run(self):
        if not self._takeoff_and_hover():
            return self._save_csv(suffix='_takeoff_abort')

        # 1) 纯视觉 YAW 对齐
        aligned = self._wait_yaw_alignment()
        if not aligned:
            rospy.logwarn('[RESULT] YAW 对齐失败，尝试降落')
            self.set_mode_client(base_mode=0, custom_mode='AUTO.LAND')
            rospy.sleep(8.0)
            self.arming_client(False)
            return self._save_csv(suffix='_align_fail')

        # 2) 展开机臂
        self._expand_arms_before_approach()

        # 3) 视觉引导下接近
        success, reason = self._approach_until_contact()
        rospy.loginfo(f'[RESULT] 接近结果: success={success}, reason={reason}')

        if success:
            # CONTACT 后流程（与地面站一致）：
            # 1) 收回机臂（SITL 中仍由脚本驱动 Gazebo 插件；实机由 PX4 自动发布 huaqiccc_morph_cmd）
            # 2) CONTACT_WAIT：等待机臂收拢，期间 PX4 三轴 setpoint 接管生效
            # 3) THROTTLE_RAMP：MPCA_PC_EN 2->1 打断 PX4 接管，缓慢降低 z setpoint
            # 4) FIX + FORCE DISARM：SITL 中冻结位姿并停桨，验证稳定栖落
            self._retract_arms_after_contact()
            contact_wait_ok = self._contact_wait(push_yaw=self.push_yaw)
            if not contact_wait_ok:
                rospy.logwarn('[RESULT] CONTACT_WAIT 未确认机臂收拢，继续执行后续安全流程')

            self._throttle_ramp()

            rospy.loginfo('[FINAL] 触发 fix_perching 固定并停止电机')
            self._send_fix(True)
            rospy.sleep(1.0)
            if not self._force_disarm():
                rospy.logwarn('[WARN] Force disarm 失败，尝试普通 disarm')
                self.arming_client(False)
            rospy.sleep(2.0)

            # 监测栖落稳定性
            self._monitor_after_disarm(self.monitor_time)
            rospy.loginfo('[DONE] CONTACT 后流程完成（PX4 接管、机臂收回、油门下降、固定、电机停止）')
        else:
            rospy.loginfo('[LAND] 降落')
            self.set_mode_client(base_mode=0, custom_mode='AUTO.LAND')
            rospy.sleep(8.0)
            self.arming_client(False)

        suffix = '_contact' if success else f'_{reason}'
        return self._save_csv(suffix=suffix)


def main():
    parser = argparse.ArgumentParser(description='Vision-guided approach test for SITL perching')
    parser.add_argument('--output', default='vision_approach', help='Output CSV prefix')
    # 过滤 ROS remapping 参数（如 _camera_offset_x:=20.0），使脚本可直接接收 private param
    args = parser.parse_args(rospy.myargv()[1:])

    print('\n' + '=' * 60)
    print('  Vision-Guided Approach Test (SITL)')
    print('  Flow: takeoff -> stable -> pure vision yaw align -> arm expand -> MPCA_PC_EN=2 -> approach -> blind_push -> contact -> arm_retract -> CONTACT_WAIT -> throttle_ramp -> fix -> disarm -> monitor')
    print('=' * 60 + '\n')

    test = VisionApproachTest(output_prefix=args.output)
    csv_path = test.run()
    if csv_path:
        print(f'\n[RESULT] Log saved to: {csv_path}')
    else:
        print('\n[ERROR] Test did not complete or no log saved')


if __name__ == '__main__':
    main()
