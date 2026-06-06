#!/usr/bin/env python3
"""
huaqiccc_dynamic_param_adjust.py
=================================
huaqiccc 机臂变形动态飞行参数调整脚本

功能:
- 实时订阅机臂角度 (/huaqiccc/arm_angle 或 /huaqiccc/arm_status)
- 根据当前机臂角度，重新计算整机质心和四个电机相对质心的位置
- 通过 MAVROS /mavros/cmd/command 发送 MAV_CMD_SET_ROTOR_CONFIG (31000)
    动态更新 PX4 的电机空间位置，使控制分配器适应新的几何构型

坐标系:
- SDF: X+前, Y+左, Z+上
- PX4 FRD (北东地): X+前, Y+右, Z+下
- 脚本内部自动完成 SDF → PX4 FRD 转换

MAV_CMD_SET_ROTOR_CONFIG (31000) 参数格式:
    param1: PX  (前后, 前正)
    param2: PY  (左右, 右正)
    param3: PZ  (上下, 下正 — FRD)
    param4: thrust_axis_x (通常为 0)
    param5: thrust_axis_y (通常为 0)
    param6: thrust_axis_z (通常为 -1, 推力向上)
    param7: rotor_index (0=lb, 1=lf, 2=rb, 3=rf)

用法:
    # 终端 1: 启动仿真
    roslaunch ~/PX4-Autopilot/launch/mavros_posix_sitl.launch

    # 终端 2: 启动本脚本
    source /opt/ros/noetic/setup.bash
    python3 huaqiccc_dynamic_param_adjust.py

更新策略（保守安全）:
- 角度变化超过 0.05 rad 时触发重新计算
- 参数更新频率限制: 最大 0.5 Hz (每 2 秒最多一次)
- 四个电机依次发送，间隔 0.3 秒，避免 PX4 过载
- 启用平滑过渡: 每次角度变化分 3 步插值发送
"""

import numpy as np
import math
import time
import json
import xml.etree.ElementTree as ET

try:
    import rospy
    from std_msgs.msg import Float64, String
    from mavros_msgs.srv import CommandLong
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARN] ROS / MAVROS 未安装，脚本将以离线演示模式运行。")


# ===================== SDF 运动学解析 =====================

class SDFKinematics:
    """解析 SDF，建立完整的运动学/动力学模型"""

    def __init__(self, sdf_path):
        self.tree = ET.parse(sdf_path)
        self.root = self.tree.getroot()
        self.model = self.root.find('model')
        self.links = {}
        self.joints = {}
        self._parse_links()
        self._parse_joints()

    def _parse_links(self):
        for link in self.model.findall('link'):
            name = link.get('name')
            pose_str = link.get('pose', '0 0 0 0 0 0')
            pose = np.array([float(x) for x in pose_str.split()])
            inertial = link.find('inertial')
            mass = 0.0
            inertial_pose = np.zeros(6)
            if inertial is not None:
                m = inertial.find('mass')
                mass = float(m.text) if m is not None else 0.0
                p = inertial.find('pose')
                if p is not None and p.text:
                    vals = [float(x) for x in p.text.strip().split()]
                    inertial_pose[:len(vals)] = vals
            self.links[name] = {'pose': pose, 'mass': mass, 'inertial_pose': inertial_pose}

    def _parse_joints(self):
        for joint in self.model.findall('joint'):
            name = joint.get('name')
            jtype = joint.get('type', 'fixed')
            parent = joint.find('parent')
            child = joint.find('child')
            p = joint.find('pose')
            pose = np.zeros(6)
            if p is not None and p.text:
                vals = [float(x) for x in p.text.strip().split()]
                pose[:len(vals)] = vals
            axis = joint.find('axis')
            axis_xyz = None
            if axis is not None:
                xyz = axis.find('xyz')
                if xyz is not None and xyz.text:
                    axis_xyz = np.array([float(x) for x in xyz.text.strip().split()])
            self.joints[name] = {
                'type': jtype,
                'parent': parent.text if parent is not None else '',
                'child': child.text if child is not None else '',
                'pose': pose,
                'axis': axis_xyz
            }

    @staticmethod
    def rotate_vector(v, axis, angle):
        axis = np.array(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        return v * cos_a + np.cross(axis, v) * sin_a + axis * np.dot(axis, v) * (1 - cos_a)

    def compute_system(self, arm_angle=0.0):
        base_frame = np.zeros(3)
        links, joints = self.links, self.joints

        # 左臂
        ljf = joints['left_arm_joint']['pose'][:3]
        lax = joints['left_arm_joint']['axis']
        laf = base_frame + ljf
        lac = laf + self.rotate_vector(links['left_arm_link']['inertial_pose'][:3], lax, arm_angle)

        lb_j = self.rotate_vector(joints['lb_motor_joint']['pose'][:3], lax, arm_angle)
        lb_c = laf + lb_j + self.rotate_vector(links['lb_motor_link']['inertial_pose'][:3], lax, arm_angle)

        lf_j = self.rotate_vector(joints['lf_motor_joint']['pose'][:3], lax, arm_angle)
        lf_c = laf + lf_j + self.rotate_vector(links['lf_motor_link']['inertial_pose'][:3], lax, arm_angle)

        # 右臂
        rjf = joints['right_arm_joint']['pose'][:3]
        rax = joints['right_arm_joint']['axis']
        raf = base_frame + rjf
        rac = raf + self.rotate_vector(links['right_arm_link']['inertial_pose'][:3], rax, arm_angle)

        rb_j = self.rotate_vector(joints['rb_motor_joint']['pose'][:3], rax, arm_angle)
        rb_c = raf + rb_j + self.rotate_vector(links['rb_motor_link']['inertial_pose'][:3], rax, arm_angle)

        rf_j = self.rotate_vector(joints['rf_motor_joint']['pose'][:3], rax, arm_angle)
        rf_c = raf + rf_j + self.rotate_vector(links['rf_motor_link']['inertial_pose'][:3], rax, arm_angle)

        # 质心
        items = [
            ('base_link', base_frame + links['base_link']['inertial_pose'][:3]),
            ('/imu_link', base_frame + joints['/imu_joint']['pose'][:3] + links['/imu_link']['inertial_pose'][:3]),
            ('left_arm_link', lac), ('right_arm_link', rac),
            ('lb_motor_link', lb_c), ('lf_motor_link', lf_c),
            ('rb_motor_link', rb_c), ('rf_motor_link', rf_c),
        ]
        tm = sum(links[n]['mass'] for n, _ in items)
        com = sum(links[n]['mass'] * c for n, c in items) / tm

        return {
            'total_mass': tm,
            'com_total': com,
            'motor_rel_sdf': {'lb': lb_c - com, 'lf': lf_c - com, 'rb': rb_c - com, 'rf': rf_c - com},
        }

    def get_px4_rotor_params(self, arm_angle=0.0):
        r = self.compute_system(arm_angle)
        com = r['com_total']
        params = {}
        for num, name in {0: 'lb', 1: 'lf', 2: 'rb', 3: 'rf'}.items():
            pos = r['motor_rel_sdf'][name]
            params[num] = {
                'PX': round(float(pos[0]), 5),           # X 同向
                'PY': round(float(-pos[1]), 5),          # SDF Y+左 → PX4 Y+右
                'PZ': round(float(-pos[2]), 5),          # SDF Z+上 → PX4 Z+下
            }
        return params, r['total_mass'], com


# ===================== 动态参数更新器 =====================

class DynamicParamUpdater:
    """
    订阅机臂角度 → 重新计算电机位置 → 通过 31000 命令更新 PX4
    """

    # 电机编号 → 混控器 rotor index 映射
    MOTOR_INDEX = {'lb': 0, 'lf': 1, 'rb': 2, 'rf': 3}

    def __init__(self, sdf_path,
                 angle_threshold=0.05,    # rad，角度变化超过此值才更新
                 max_rate_hz=0.5,          # Hz，最大更新频率
                 motor_send_interval=0.3,  # s，四个电机依次发送间隔
                 smooth_steps=3):          # 平滑插值步数
        """
        保守默认：
        - 角度阈值 0.05 rad (~3 deg)
        - 最大 0.5 Hz (每 2 秒一次)
        - 电机间发送间隔 0.3 秒
        - 平滑过渡 3 步
        """
        self.kin = SDFKinematics(sdf_path)
        self.angle_threshold = angle_threshold
        self.min_interval = 1.0 / max_rate_hz
        self.motor_interval = motor_send_interval
        self.smooth_steps = smooth_steps

        self.last_angle = None
        self.last_params = None
        self.last_update_time = 0.0
        self.current_angle = 0.0
        self.running = False

        # ROS
        self.ros_connected = False
        self.cmd_long_srv = None
        self.angle_sub = None
        self.status_sub = None

        if not ROS_AVAILABLE:
            print("[WARN] ROS 不可用，将以 print-only 模式运行")
            return

        try:
            if not rospy.core.is_initialized():
                rospy.init_node('huaqiccc_param_adjuster', anonymous=True)
            self.ros_connected = True

            self.angle_sub = rospy.Subscriber('/huaqiccc/arm_angle', Float64, self._on_angle)
            self.status_sub = rospy.Subscriber('/huaqiccc/arm_status', String, self._on_status)

            # MAVROS command_long 服务
            try:
                rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
                self.cmd_long_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
                print("[OK] MAVROS /mavros/cmd/command 服务已连接")
            except Exception as e:
                print(f"[WARN] MAVROS cmd/command 服务未连接: {e}")
                print("       将以 print-only 模式运行（仅打印命令，不实际发送）")

            print("[OK] 动态参数调整器初始化完成，等待机臂角度...")
        except Exception as e:
            print(f"[WARN] ROS 初始化失败: {e}")
            self.ros_connected = False

    def _on_angle(self, msg):
        self.current_angle = msg.data

    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
            if 'right_angle' in data and data['right_angle'] is not None:
                self.current_angle = float(data['right_angle'])
        except Exception:
            pass

    # ----------------- 参数发送方法 -----------------

    def _send_rotor_31000(self, rotor_index, px, py, pz,
                          thrust_axis_x=0.0, thrust_axis_y=0.0, thrust_axis_z=-1.0):
        """
        发送单个电机的 MAV_CMD_SET_ROTOR_CONFIG (31000) 命令
        rotor_index: 0=lb, 1=lf, 2=rb, 3=rf
        """
        if not self.cmd_long_srv:
            print(f"  [PRINT-ONLY] motor{rotor_index}: PX={px:.4f} PY={py:.4f} PZ={pz:.4f}")
            return True

        try:
            from mavros_msgs.srv import CommandLongRequest
            req = CommandLongRequest()
            req.broadcast = False
            req.command = 31000  # MAV_CMD_SET_ROTOR_CONFIG
            req.confirmation = 0
            req.param1 = float(px)
            req.param2 = float(py)
            req.param3 = float(pz)
            req.param4 = float(thrust_axis_x)
            req.param5 = float(thrust_axis_y)
            req.param6 = float(thrust_axis_z)
            req.param7 = float(rotor_index)

            resp = self.cmd_long_srv(req)
            success = resp.success if hasattr(resp, 'success') else False
            status = "OK" if success else "FAIL"
            print(f"  [{status}] motor{rotor_index}: PX={px:.4f} PY={py:.4f} PZ={pz:.4f}")
            return success
        except Exception as e:
            print(f"  [ERROR] motor{rotor_index} 发送失败: {e}")
            return False

    def send_all_rotors(self, params_dict):
        """
        依次发送四个电机的 31000 命令
        params_dict: {0: {'PX':x,'PY':y,'PZ':z}, 1:...}
        """
        results = []
        for i in range(4):
            p = params_dict[i]
            ok = self._send_rotor_31000(i, p['PX'], p['PY'], p['PZ'])
            results.append(ok)
            if i < 3:  # 电机间延迟
                time.sleep(self.motor_interval)
        return all(results)

    def send_smooth_transition(self, from_params, to_params, steps=None):
        """
        平滑过渡：从 from_params 逐步插值到 to_params，分 steps 步发送
        避免参数跳动过大导致飞行器失控
        """
        steps = steps or self.smooth_steps
        print(f"  [SMOOTH] 分 {steps} 步平滑过渡...")

        for step in range(1, steps + 1):
            alpha = step / float(steps)
            interp = {}
            for i in range(4):
                interp[i] = {
                    'PX': from_params[i]['PX'] + alpha * (to_params[i]['PX'] - from_params[i]['PX']),
                    'PY': from_params[i]['PY'] + alpha * (to_params[i]['PY'] - from_params[i]['PY']),
                    'PZ': from_params[i]['PZ'] + alpha * (to_params[i]['PZ'] - from_params[i]['PZ']),
                }

            print(f"  Step {step}/{steps}:")
            self.send_all_rotors(interp)

            if step < steps:
                time.sleep(self.motor_interval * 2)

        return True

    # ----------------- 主更新逻辑 -----------------

    def update(self):
        """检查角度变化，如超过阈值则重新计算并发送参数"""
        now = time.time()

        # 频率限制
        if now - self.last_update_time < self.min_interval:
            return False, {}

        # 首次运行
        if self.last_angle is None:
            self.last_angle = self.current_angle
            params, mass, com = self.kin.get_px4_rotor_params(self.current_angle)
            self.last_params = params
            self.last_update_time = now
            print(f"\n[INIT] 机臂={self.current_angle:.3f}rad 质量={mass:.3f}kg "
                  f"COM=({com[0]:.3f},{com[1]:.3f},{com[2]:.3f})")
            self.send_all_rotors(params)
            return True, params

        # 角度变化检查
        delta = abs(self.current_angle - self.last_angle)
        if delta < self.angle_threshold:
            return False, {}

        # 计算新参数
        new_params, mass, com = self.kin.get_px4_rotor_params(self.current_angle)
        self.last_angle = self.current_angle
        self.last_update_time = now

        print(f"\n[UPDATE] 角度变化 {delta:.3f}rad → 新角度={self.current_angle:.3f}rad")
        print(f"         COM=({com[0]:.4f},{com[1]:.4f},{com[2]:.4f})")

        # 平滑过渡发送
        if self.last_params and self.smooth_steps > 1:
            self.send_smooth_transition(self.last_params, new_params)
        else:
            self.send_all_rotors(new_params)

        self.last_params = new_params
        return True, new_params

    # ----------------- 运行模式 -----------------

    def run_loop(self):
        """持续监听角度变化并更新参数（主动轮询模式）"""
        self.running = True
        print(f"\n{'='*60}")
        print("  huaqiccc 动态参数调整器 运行中")
        print(f"{'='*60}")
        print(f"  触发阈值 : {self.angle_threshold:.2f} rad ({math.degrees(self.angle_threshold):.1f} deg)")
        print(f"  最大频率 : {1.0/self.min_interval:.1f} Hz")
        print(f"  平滑步数 : {self.smooth_steps}")
        print(f"  电机间隔 : {self.motor_interval}s")
        print(f"  发送方式 : {'MAVROS 31000' if self.cmd_long_srv else 'PRINT-ONLY'}")
        print(f"{'='*60}\n")

        # 先执行一次初始化
        self.update()

        rate = rospy.Rate(2) if ROS_AVAILABLE else None
        try:
            while self.running and ROS_AVAILABLE:
                self.update()
                if rate:
                    rate.sleep()
                else:
                    time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[STOP] 用户中断")
        except Exception as e:
            print(f"\n[ERROR] {e}")

    def run_with_callback(self):
        """ROS spin 回调模式（推荐：只在角度变化时触发）"""
        self.running = True
        print(f"\n{'='*60}")
        print("  huaqiccc 动态参数调整器 运行中 [回调模式]")
        print(f"{'='*60}")
        print(f"  此模式下角度变化自动触发更新，无需轮询")
        print(f"{'='*60}\n")

        # 初始化一次
        rospy.sleep(1.0)
        self.update()

        # 主循环只做频率控制，实际由 ROS 回调驱动
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("\n[STOP] 用户中断")

    def run_once(self, arm_angle=0.0):
        """单次计算并发送，用于初始化或测试"""
        self.current_angle = arm_angle
        params, mass, com = self.kin.get_px4_rotor_params(arm_angle)
        print(f"\n[ONCE] 机臂={arm_angle:.3f}rad 质量={mass:.3f}kg")
        print(f"       COM=({com[0]:.4f},{com[1]:.4f},{com[2]:.4f})")
        for i in range(4):
            p = params[i]
            print(f"       motor{i}: PX={p['PX']:.4f} PY={p['PY']:.4f} PZ={p['PZ']:.4f}")
        self.send_all_rotors(params)
        self.last_angle = arm_angle
        self.last_params = params
        return params


def main():
    import sys
    sdf_path = sys.argv[1] if len(sys.argv) > 1 else '/mnt/agents/upload/现在的huaqiccc.txt'

    updater = DynamicParamUpdater(
        sdf_path=sdf_path,
        angle_threshold=0.05,
        max_rate_hz=0.5,
        motor_send_interval=0.3,
        smooth_steps=3
    )

    # 先执行一次初始化
    updater.run_once(arm_angle=0.0)

    # ROS 可用时进入持续监听
    if ROS_AVAILABLE:
        updater.run_with_callback()


if __name__ == '__main__':
    main()
