#!/usr/bin/env python3
"""
Huaqiccc 变形无人机地面站主窗口
"""
import os
import sys
import json
import subprocess
import signal
from datetime import datetime

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QLineEdit,
    QGroupBox, QPlainTextEdit, QSplitter, QFrame,
    QMessageBox, QSpinBox, QDoubleSpinBox, QCheckBox,
    QGridLayout, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QColor, QFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config_manager import ConfigManager
from video_widget import VideoWidget

# ROS 相关延迟导入（避免无 ROS 环境时崩溃）
ROS_AVAILABLE = False
ros_interface = None


def init_ros_interface():
    global ROS_AVAILABLE, ros_interface
    try:
        import rospy
        from ros_interface import ROSInterface
        ros_interface = ROSInterface()
        ROS_AVAILABLE = True
        return True
    except Exception as e:
        print(f"[WARN] ROS 未初始化或不可用: {e}")
        return False


class GroundStation(QMainWindow):
    """地面站主窗口"""

    def __init__(self):
        super().__init__()
        self.cfg = ConfigManager()
        self.setWindowTitle(self.cfg.get('ui.window_title', 'Huaqiccc 变形无人机地面站'))
        self.resize(self.cfg.get('ui.window_width', 1400), self.cfg.get('ui.window_height', 900))

        # 进程管理
        self.procs = {}
        self.ros_initialized = False

        # 状态缓存
        self._fc_status = "未连接"
        self._vision_status = "等待"
        self._mocap_status = "等待"
        self._current_detections = []

        self._build_ui()
        self._init_timers()
        self._apply_config_to_ui()

    # ================================================================
    # UI 构建
    # ================================================================
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(8, 8, 8, 8)

        # ---- 左侧：视频 + 日志 ----
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # 视频显示
        self.video_widget = VideoWidget()
        self.video_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_widget.target_selected.connect(self._on_video_click)
        left_layout.addWidget(self.video_widget, stretch=3)

        # 日志区
        log_group = QGroupBox("系统日志")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(500)
        self.log_text.setStyleSheet("background-color: #1a1a1a; color: #00ff00; font-family: Consolas;")
        log_layout.addWidget(self.log_text)
        left_layout.addWidget(log_group, stretch=1)

        # ---- 右侧：控制面板 ----
        right_scroll = QWidget()
        right_layout = QVBoxLayout(right_scroll)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        # 1. 连接控制
        right_layout.addWidget(self._build_connection_panel())
        # 2. 状态监控
        right_layout.addWidget(self._build_status_panel())
        # 3. 手动控制
        right_layout.addWidget(self._build_manual_panel())
        # 4. 任务控制
        right_layout.addWidget(self._build_mission_panel())
        # 5. 紧急停止
        right_layout.addWidget(self._build_emergency_panel())

        right_layout.addStretch()

        # 整体分割
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_scroll)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1000, 400])
        main_layout.addWidget(splitter)

    def _build_connection_panel(self):
        group = QGroupBox("连接控制")
        layout = QGridLayout(group)

        # 串口
        layout.addWidget(QLabel("飞控串口:"), 0, 0)
        self.combo_serial = QComboBox()
        self.combo_serial.setEditable(True)
        self.combo_serial.addItem("/dev/ttyUSB0")
        self.combo_serial.addItem("/dev/ttyUSB1")
        self.combo_serial.addItem("/dev/ttyACM0")
        self.combo_serial.addItem("/dev/ttyACM1")
        layout.addWidget(self.combo_serial, 0, 1)

        layout.addWidget(QLabel("波特率:"), 0, 2)
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["57600", "115200", "921600"])
        self.combo_baud.setCurrentText("115200")
        layout.addWidget(self.combo_baud, 0, 3)

        # 动捕 IP
        layout.addWidget(QLabel("动捕 IP:"), 1, 0)
        self.edit_mocap_ip = QLineEdit("192.168.1.5")
        layout.addWidget(self.edit_mocap_ip, 1, 1)

        layout.addWidget(QLabel("追踪名:"), 1, 2)
        self.edit_tracker_name = QLineEdit("Tracker1")
        layout.addWidget(self.edit_tracker_name, 1, 3)

        # RTSP
        layout.addWidget(QLabel("RTSP 地址:"), 2, 0)
        self.edit_rtsp = QLineEdit("rtsp://192.168.144.25:8554/main.264")
        layout.addWidget(self.edit_rtsp, 2, 1, 1, 3)

        # 按钮行
        btn_layout = QHBoxLayout()
        self.btn_ros_init = QPushButton("初始化 ROS")
        self.btn_ros_init.setStyleSheet("background-color: #2d5a27; color: white;")
        self.btn_ros_init.clicked.connect(self._on_init_ros)
        btn_layout.addWidget(self.btn_ros_init)

        self.btn_vrpn = QPushButton("启动 VRPN")
        self.btn_vrpn.clicked.connect(self._on_start_vrpn)
        btn_layout.addWidget(self.btn_vrpn)

        self.btn_mavros = QPushButton("启动 MAVROS")
        self.btn_mavros.clicked.connect(self._on_start_mavros)
        btn_layout.addWidget(self.btn_mavros)

        self.btn_yolo = QPushButton("启动 YOLO")
        self.btn_yolo.clicked.connect(self._on_start_yolo)
        btn_layout.addWidget(self.btn_yolo)

        layout.addLayout(btn_layout, 3, 0, 1, 4)
        return group

    def _build_status_panel(self):
        group = QGroupBox("状态监控")
        layout = QGridLayout(group)

        # 飞控状态
        layout.addWidget(QLabel("飞控连接:"), 0, 0)
        self.lbl_fc = QLabel("未连接")
        self.lbl_fc.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.lbl_fc, 0, 1)

        layout.addWidget(QLabel("解锁状态:"), 0, 2)
        self.lbl_arm = QLabel("DISARMED")
        self.lbl_arm.setStyleSheet("color: gray;")
        layout.addWidget(self.lbl_arm, 0, 3)

        layout.addWidget(QLabel("飞行模式:"), 1, 0)
        self.lbl_mode = QLabel("UNKNOWN")
        layout.addWidget(self.lbl_mode, 1, 1)

        layout.addWidget(QLabel("动捕状态:"), 1, 2)
        self.lbl_mocap = QLabel("等待")
        self.lbl_mocap.setStyleSheet("color: gray;")
        layout.addWidget(self.lbl_mocap, 1, 3)

        # 视觉状态
        layout.addWidget(QLabel("视觉状态:"), 2, 0)
        self.lbl_vision = QLabel("等待")
        self.lbl_vision.setStyleSheet("color: gray;")
        layout.addWidget(self.lbl_vision, 2, 1)

        layout.addWidget(QLabel("YAW 对齐:"), 2, 2)
        self.lbl_aligned = QLabel("未对齐")
        self.lbl_aligned.setStyleSheet("color: gray;")
        layout.addWidget(self.lbl_aligned, 2, 3)

        # 锁定目标
        layout.addWidget(QLabel("锁定目标:"), 3, 0)
        self.lbl_lock = QLabel("自动")
        self.lbl_lock.setStyleSheet("color: cyan;")
        layout.addWidget(self.lbl_lock, 3, 1)

        layout.addWidget(QLabel("像素误差:"), 3, 2)
        self.lbl_pixel_err = QLabel("0.000")
        layout.addWidget(self.lbl_pixel_err, 3, 3)

        # 位置
        layout.addWidget(QLabel("当前位置:"), 4, 0)
        self.lbl_pos = QLabel("x: --  y: --  z: --")
        layout.addWidget(self.lbl_pos, 4, 1, 1, 3)

        return group

    def _build_manual_panel(self):
        group = QGroupBox("手动控制")
        layout = QGridLayout(group)

        # 变形控制
        layout.addWidget(QLabel("机臂角度 (rad):"), 0, 0)
        self.spin_morph_angle = QDoubleSpinBox()
        self.spin_morph_angle.setRange(-0.5, 0.0)
        self.spin_morph_angle.setSingleStep(0.05)
        self.spin_morph_angle.setValue(-0.35)
        self.spin_morph_angle.setDecimals(2)
        layout.addWidget(self.spin_morph_angle, 0, 1)

        self.btn_morph_send = QPushButton("发送变形指令")
        self.btn_morph_send.clicked.connect(self._on_send_morph)
        layout.addWidget(self.btn_morph_send, 0, 2)

        self.btn_morph_expand = QPushButton("一键展开")
        self.btn_morph_expand.setStyleSheet("background-color: #1a5c1a; color: white;")
        self.btn_morph_expand.clicked.connect(lambda: self._quick_morph(-0.35))
        layout.addWidget(self.btn_morph_expand, 1, 0)

        self.btn_morph_retract = QPushButton("一键收拢")
        self.btn_morph_retract.setStyleSheet("background-color: #5c1a1a; color: white;")
        self.btn_morph_retract.clicked.connect(lambda: self._quick_morph(0.0))
        layout.addWidget(self.btn_morph_retract, 1, 1)

        # YAW 对齐开关
        self.chk_yaw_align = QCheckBox("启用 YAW 自动对齐")
        self.chk_yaw_align.setChecked(True)
        layout.addWidget(self.chk_yaw_align, 2, 0, 1, 2)

        # OFFBOARD 模式
        self.btn_offboard = QPushButton("设置 OFFBOARD")
        self.btn_offboard.clicked.connect(self._on_set_offboard)
        layout.addWidget(self.btn_offboard, 2, 2)

        self.btn_arm = QPushButton("解锁 (ARM)")
        self.btn_arm.setStyleSheet("background-color: #1a5c1a; color: white;")
        self.btn_arm.clicked.connect(lambda: self._on_arm(True))
        layout.addWidget(self.btn_arm, 3, 0)

        self.btn_disarm = QPushButton("锁定 (DISARM)")
        self.btn_disarm.setStyleSheet("background-color: #5c1a1a; color: white;")
        self.btn_disarm.clicked.connect(lambda: self._on_arm(False))
        layout.addWidget(self.btn_disarm, 3, 1)

        return group

    def _build_mission_panel(self):
        group = QGroupBox("任务控制")
        layout = QVBoxLayout(group)

        self.btn_mission_start = QPushButton("▶ 一键栖停任务")
        self.btn_mission_start.setStyleSheet(
            "background-color: #1a5c1a; color: white; font-size: 14px; padding: 10px;"
        )
        self.btn_mission_start.clicked.connect(self._on_mission_start)
        layout.addWidget(self.btn_mission_start)

        self.btn_mission_abort = QPushButton("⏹ 中止任务")
        self.btn_mission_abort.setStyleSheet(
            "background-color: #5c5c1a; color: white; font-size: 12px; padding: 6px;"
        )
        self.btn_mission_abort.clicked.connect(self._on_mission_abort)
        layout.addWidget(self.btn_mission_abort)

        info = QLabel(
            "任务流程:\n"
            "1. 起飞 → 悬停\n"
            "2. YAW 对齐柱子\n"
            "3. 展开机臂\n"
            "4. 接近 → 接触\n"
            "5. 收拢夹持 → 栖停"
        )
        info.setStyleSheet("color: #888888; font-size: 10px;")
        layout.addWidget(info)
        return group

    def _build_emergency_panel(self):
        group = QGroupBox("安全")
        layout = QVBoxLayout(group)

        self.btn_estop = QPushButton("🚨 紧急停止")
        self.btn_estop.setStyleSheet(
            "background-color: #cc0000; color: white; font-size: 18px; font-weight: bold; padding: 15px;"
        )
        self.btn_estop.clicked.connect(self._on_emergency_stop)
        layout.addWidget(self.btn_estop)
        return group

    # ================================================================
    # 定时器
    # ================================================================
    def _init_timers(self):
        # 状态刷新定时器
        self.timer_status = QTimer(self)
        self.timer_status.timeout.connect(self._update_status)
        self.timer_status.start(100)  # 10 Hz

        # 视频刷新定时器
        self.timer_video = QTimer(self)
        self.timer_video.timeout.connect(self._update_video)
        self.timer_video.start(33)  # ~30 Hz

    # ================================================================
    # 配置应用
    # ================================================================
    def _apply_config_to_ui(self):
        cfg = self.cfg.config
        fc = cfg.get('flight_controller', {})
        self.combo_serial.setCurrentText(fc.get('serial_port', '/dev/ttyUSB0'))
        self.combo_baud.setCurrentText(str(fc.get('baudrate', 115200)))

        mc = cfg.get('motion_capture', {})
        self.edit_mocap_ip.setText(mc.get('server_ip', '192.168.1.5'))
        self.edit_tracker_name.setText(mc.get('tracker_name', 'Tracker1'))

        cam = cfg.get('camera', {})
        self.edit_rtsp.setText(cam.get('rtsp_url', 'rtsp://192.168.144.25:8554/main.264'))

    def _save_config_from_ui(self):
        """将 UI 上的值保存回配置文件"""
        self.cfg.set('flight_controller.serial_port', self.combo_serial.currentText())
        self.cfg.set('flight_controller.baudrate', int(self.combo_baud.currentText()))
        self.cfg.set('flight_controller.fcu_url',
                     f"serial://{self.combo_serial.currentText()}:{self.combo_baud.currentText()}")
        self.cfg.set('motion_capture.server_ip', self.edit_mocap_ip.text())
        self.cfg.set('motion_capture.tracker_name', self.edit_tracker_name.text())
        self.cfg.set('camera.rtsp_url', self.edit_rtsp.text())
        self.cfg.save()
        self.log("[CONFIG] 配置已保存到 ground_station.yaml")

    # ================================================================
    # 槽函数：连接控制
    # ================================================================
    @pyqtSlot()
    def _on_init_ros(self):
        if self.ros_initialized:
            self.log("[ROS] 已经初始化")
            return
        self.log("[ROS] 正在初始化...")
        if init_ros_interface():
            self.ros_initialized = True
            self.btn_ros_init.setText("ROS 已就绪")
            self.btn_ros_init.setStyleSheet("background-color: #3a7a33; color: white;")
            self.log("[ROS] 初始化成功")
        else:
            self.log("[ROS] 初始化失败，请确认 roscore 已启动")

    @pyqtSlot()
    def _on_start_vrpn(self):
        if "vrpn" in self.procs and self.procs["vrpn"].poll() is None:
            self.log("[VRPN] 已经在运行")
            return
        self._save_config_from_ui()
        ip = self.edit_mocap_ip.text()
        tracker = self.edit_tracker_name.text()
        cmd = [
            "roslaunch", "vrpn_client_ros", "sample.launch",
            f"server:={ip}"
        ]
        self.procs["vrpn"] = subprocess.Popen(cmd)
        self.log(f"[VRPN] 已启动: server={ip}")

        # 同时启动 topic relay
        relay_cmd = [
            "rosrun", "topic_tools", "relay",
            f"/vrpn_client_node/{tracker}/pose",
            "/mavros/vision_pose/pose"
        ]
        self.procs["relay"] = subprocess.Popen(relay_cmd)
        self.log(f"[RELAY] /vrpn_client_node/{tracker}/pose -> /mavros/vision_pose/pose")

    @pyqtSlot()
    def _on_start_mavros(self):
        if "mavros" in self.procs and self.procs["mavros"].poll() is None:
            self.log("[MAVROS] 已经在运行")
            return
        self._save_config_from_ui()
        fcu = self.cfg.get('flight_controller.fcu_url')
        gcs = self.cfg.get('flight_controller.gcs_url', 'udp://@127.0.0.1')
        cmd = [
            "roslaunch", "mavros", "px4.launch",
            f"fcu_url:={fcu}",
            f"gcs_url:={gcs}"
        ]
        self.procs["mavros"] = subprocess.Popen(cmd)
        self.log(f"[MAVROS] 已启动: fcu_url={fcu}")

    @pyqtSlot()
    def _on_start_yolo(self):
        if "yolo" in self.procs and self.procs["yolo"].poll() is None:
            self.log("[YOLO] 已经在运行")
            return
        self._save_config_from_ui()
        rtsp = self.edit_rtsp.text()
        yolo_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "..", "YOLO4SEU_MD")
        yolo_path = os.path.abspath(yolo_path)
        cmd = [
            "roslaunch", "YOLO4SEU_MD", "rtsp_pillar.launch",
            f"rtsp_url:={rtsp}"
        ]
        env = os.environ.copy()
        env["ROS_PACKAGE_PATH"] = f"{yolo_path}:{env.get('ROS_PACKAGE_PATH', '')}"
        self.procs["yolo"] = subprocess.Popen(cmd, env=env)
        self.log(f"[YOLO] 已启动: RTSP={rtsp}")

    # ================================================================
    # 槽函数：手动控制
    # ================================================================
    @pyqtSlot()
    def _on_send_morph(self):
        angle = self.spin_morph_angle.value()
        if ros_interface and ROS_AVAILABLE:
            ros_interface.send_morph_command_31440(angle)
            self.log(f"[MORPH] 发送变形指令: {angle:.2f} rad")
        else:
            self.log("[MORPH] ROS 未连接，无法发送")

    def _quick_morph(self, angle):
        self.spin_morph_angle.setValue(angle)
        if ros_interface and ROS_AVAILABLE:
            ros_interface.send_morph_command_31440(angle)
            self.log(f"[MORPH] 一键{'展开' if angle < 0 else '收拢'}: {angle:.2f} rad")
        else:
            self.log("[MORPH] ROS 未连接，无法发送")

    @pyqtSlot()
    def _on_set_offboard(self):
        if ros_interface and ROS_AVAILABLE:
            ok = ros_interface.set_mode("OFFBOARD")
            self.log(f"[MODE] 设置 OFFBOARD: {'成功' if ok else '失败'}")
        else:
            self.log("[MODE] ROS 未连接")

    @pyqtSlot()
    def _on_arm(self, arm):
        if ros_interface and ROS_AVAILABLE:
            ok = ros_interface.arm(arm)
            self.log(f"[ARM] {'解锁' if arm else '锁定'}: {'成功' if ok else '失败'}")
        else:
            self.log("[ARM] ROS 未连接")

    # ================================================================
    # 槽函数：任务控制
    # ================================================================
    @pyqtSlot()
    def _on_mission_start(self):
        self.log("[MISSION] 一键栖停任务启动（功能待集成）")
        QMessageBox.information(self, "任务", "一键栖停任务逻辑将在后续版本集成。\n当前请使用手动控制分步执行。")

    @pyqtSlot()
    def _on_mission_abort(self):
        self.log("[MISSION] 任务中止")
        if ros_interface and ROS_AVAILABLE:
            ros_interface.set_mode("HOLD")
            ros_interface.send_morph_command_31440(0.0)

    # ================================================================
    # 槽函数：紧急停止
    # ================================================================
    @pyqtSlot()
    def _on_emergency_stop(self):
        self.log("🚨 紧急停止触发!")
        if ros_interface and ROS_AVAILABLE:
            ros_interface.emergency_stop()
        # 停止所有外部进程
        for name, proc in self.procs.items():
            if proc.poll() is None:
                proc.terminate()
                self.log(f"[ESTOP] 终止进程: {name}")
        QMessageBox.critical(self, "紧急停止", "已触发紧急停止!\n飞控已切到 STABILIZED 并 DISARM。")

    # ================================================================
    # 槽函数：视频点击选中
    # ================================================================
    @pyqtSlot(int, int, int, int)
    def _on_video_click(self, img_x, img_y, img_w, img_h):
        """用户在视频上点击，选择最近的检测目标"""
        if not self._current_detections:
            self.log("[SELECT] 当前无检测目标")
            return

        # 找到点击位置最近的目标中心
        best_id = -1
        best_dist = float('inf')
        for det in self._current_detections:
            x1, y1, x2, y2 = det['bbox']
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist = ((cx - img_x) ** 2 + (cy - img_y) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_id = det['id']

        if best_id >= 0 and best_dist < 100:  # 100px 阈值
            if ros_interface and ROS_AVAILABLE:
                ros_interface.lock_target(best_id)
                self.log(f"[SELECT] 锁定目标 #{best_id} (距离 {best_dist:.1f}px)")
                self.lbl_lock.setText(f"#{best_id}")
        else:
            # 点击远离所有目标，则取消锁定
            if ros_interface and ROS_AVAILABLE:
                ros_interface.lock_target(-1)
                self.log("[SELECT] 取消锁定，回退自动模式")
                self.lbl_lock.setText("自动")

    # ================================================================
    # 定时更新
    # ================================================================
    @pyqtSlot()
    def _update_status(self):
        if not ROS_AVAILABLE or ros_interface is None:
            return

        # 飞控状态
        if ros_interface.fc_connected:
            self.lbl_fc.setText("已连接")
            self.lbl_fc.setStyleSheet("color: #00ff00; font-weight: bold;")
        else:
            self.lbl_fc.setText("未连接")
            self.lbl_fc.setStyleSheet("color: red; font-weight: bold;")

        self.lbl_arm.setText("ARMED" if ros_interface.fc_armed else "DISARMED")
        self.lbl_arm.setStyleSheet("color: #ff6600;" if ros_interface.fc_armed else "color: gray;")
        self.lbl_mode.setText(ros_interface.fc_mode)

        # 位置
        if ros_interface.local_pose:
            p = ros_interface.local_pose.position
            self.lbl_pos.setText(f"x: {p.x:.2f}  y: {p.y:.2f}  z: {p.z:.2f}")
        else:
            self.lbl_pos.setText("x: --  y: --  z: --")

        # 视觉对齐
        if ros_interface.yaw_aligned:
            self.lbl_aligned.setText("✅ 已对齐")
            self.lbl_aligned.setStyleSheet("color: #00ff00; font-weight: bold;")
        else:
            self.lbl_aligned.setText("对齐中...")
            self.lbl_aligned.setStyleSheet("color: #ffaa00;")

        self.lbl_pixel_err.setText(f"{ros_interface.pixel_error:+.3f}")

        # 锁定状态
        if ros_interface.locked_target_id >= 0:
            self.lbl_lock.setText(f"#{ros_interface.locked_target_id}")
            self.lbl_lock.setStyleSheet("color: #00ff00; font-weight: bold;")
        else:
            self.lbl_lock.setText("自动")
            self.lbl_lock.setStyleSheet("color: cyan;")

    @pyqtSlot()
    def _update_video(self):
        if not ROS_AVAILABLE or ros_interface is None:
            self.video_widget.set_status("等待 ROS 连接...")
            return

        # 从 ROS 接口获取图像
        frame = ros_interface.detection_image
        if frame is not None:
            self.video_widget.set_frame(frame)
            self.video_widget.set_status(
                f"对齐: {'YES' if ros_interface.yaw_aligned else 'NO'} | "
                f"err={ros_interface.pixel_error:+.3f} | "
                f"锁定: {ros_interface.locked_target_id}"
            )
        else:
            self.video_widget.set_status("等待视频流...")

        # 解析检测信息（如果有）
        if hasattr(ros_interface, 'detections_info') and ros_interface.detections_info:
            data = ros_interface.detections_info
            self._current_detections = data.get('targets', [])
            # 更新锁定框中心
            for det in self._current_detections:
                if det['id'] == ros_interface.locked_target_id:
                    x1, y1, x2, y2 = det['bbox']
                    self.video_widget.set_lock_info(
                        det['id'], ((x1+x2)//2, (y1+y2)//2)
                    )
                    break
            else:
                self.video_widget.set_lock_info(-1, None)

    # ================================================================
    # 日志
    # ================================================================
    def log(self, text):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.appendPlainText(f"[{ts}] {text}")

    # ================================================================
    # 退出清理
    # ================================================================
    def closeEvent(self, event):
        self.log("[EXIT] 正在关闭地面站...")
        for name, proc in self.procs.items():
            if proc.poll() is None:
                proc.terminate()
                self.log(f"[EXIT] 终止进程: {name}")
        event.accept()
