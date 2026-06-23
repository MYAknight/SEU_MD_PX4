#!/usr/bin/env python3
"""
auto_flight_gui.py — 简化的一键飞行测试 GUI

界面元素：
    - 状态显示：连接、模式、ARM、位置、电池
    - 轨迹选择下拉框
    - MPCA_MODE 选择下拉框
    - [一键起飞并执行轨迹] 按钮
    - [降落] 按钮
    - [急停] 按钮
    - [下载日志] 按钮
    - 日志显示窗口

用法：
    # 先启动环境
    ./launch_env.sh

    # 再启动 GUI
    python3 auto_flight_gui.py
"""

import os
import sys
import subprocess
import threading
import time

import rospy
import yaml
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QTextEdit, QGroupBox, QGridLayout,
    QMessageBox
)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont

from std_srvs.srv import Trigger
from mavros_msgs.msg import State, StatusText
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped


class RosWorker(QObject):
    """ROS 回调在后台线程运行，通过信号更新 GUI"""
    state_sig = pyqtSignal(dict)
    pos_sig = pyqtSignal(dict)
    batt_sig = pyqtSignal(dict)
    log_sig = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        rospy.init_node("auto_flight_gui", anonymous=True)

        self.current_state = {}
        self.current_pos = {"x": 0, "y": 0, "z": 0}

        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/mavros/battery", BatteryState, self._battery_cb)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self._statustext_cb)

        # 等待服务可用
        rospy.wait_for_service("/optimize/start_flight", timeout=10.0)
        rospy.wait_for_service("/optimize/land", timeout=10.0)
        rospy.wait_for_service("/optimize/emergency_stop", timeout=10.0)

        self.srv_start = rospy.ServiceProxy("/optimize/start_flight", Trigger)
        self.srv_land = rospy.ServiceProxy("/optimize/land", Trigger)
        self.srv_estop = rospy.ServiceProxy("/optimize/emergency_stop", Trigger)

    def _state_cb(self, msg):
        self.current_state = {
            "connected": msg.connected,
            "armed": msg.armed,
            "mode": msg.mode,
        }
        self.state_sig.emit(self.current_state)

    def _pose_cb(self, msg):
        p = msg.pose.position
        self.current_pos = {"x": p.x, "y": p.y, "z": p.z}
        self.pos_sig.emit(self.current_pos)

    def _battery_cb(self, msg):
        self.batt_sig.emit({
            "voltage": msg.voltage,
            "percentage": msg.percentage,
        })

    def _statustext_cb(self, msg):
        text = msg.text.strip()
        if text:
            self.log_sig.emit(f"[FCU] {text}")

    def start_flight(self, trajectory: str, mode: int):
        try:
            rospy.set_param("/optimize/trajectory", trajectory)
            rospy.set_param("/optimize/mode", mode)
            resp = self.srv_start()
            self.log_sig.emit(f"[MISSION] {resp.message}")
            return resp.success
        except Exception as e:
            self.log_sig.emit(f"[ERROR] 启动飞行失败: {e}")
            return False

    def land(self):
        try:
            resp = self.srv_land()
            self.log_sig.emit(f"[MISSION] {resp.message}")
            return resp.success
        except Exception as e:
            self.log_sig.emit(f"[ERROR] 降落失败: {e}")
            return False

    def emergency_stop(self):
        try:
            resp = self.srv_estop()
            self.log_sig.emit(f"[MISSION] {resp.message}")
            return resp.success
        except Exception as e:
            self.log_sig.emit(f"[ERROR] 急停失败: {e}")
            return False


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("huaqiccc 一键飞行测试")
        self.setMinimumSize(600, 500)

        self.ros = RosWorker()
        self.ros.state_sig.connect(self._on_state)
        self.ros.pos_sig.connect(self._on_pos)
        self.ros.batt_sig.connect(self._on_battery)
        self.ros.log_sig.connect(self._on_log)

        self._build_ui()

        # 定时刷新
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._update_ui)
        self.ui_timer.start(100)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)

        font_mono = QFont("Monospace", 10)
        font_mono.setStyleHint(QFont.Monospace)

        # ---- 状态显示 ----
        sg = QGroupBox("飞控状态")
        sl = QGridLayout()
        self.lbl_conn = QLabel("<span style='color:gray'>●</span> 未连接")
        self.lbl_mode = QLabel("模式: --")
        self.lbl_arm = QLabel("锁定")
        self.lbl_pos = QLabel("X:0.00 Y:0.00 Z:0.00")
        self.lbl_pos.setFont(font_mono)
        self.lbl_batt = QLabel("电池: --")

        sl.addWidget(self.lbl_conn, 0, 0)
        sl.addWidget(self.lbl_mode, 0, 1)
        sl.addWidget(self.lbl_arm, 0, 2)
        sl.addWidget(self.lbl_pos, 1, 0, 1, 2)
        sl.addWidget(self.lbl_batt, 1, 2)
        sg.setLayout(sl)
        layout.addWidget(sg)

        # ---- 任务配置 ----
        cg = QGroupBox("任务配置")
        cl = QGridLayout()

        cl.addWidget(QLabel("轨迹:"), 0, 0)
        self.combo_traj = QComboBox()
        self.combo_traj.addItems([
            "hover",
            "takeoff_land",
            "square_small",
            "circle_small",
            "figure8_small",
            "step_x",
            "step_xy",
            "morph_circle",
        ])
        self.combo_traj.setCurrentText("circle_small")
        cl.addWidget(self.combo_traj, 0, 1)

        cl.addWidget(QLabel("控制器:"), 1, 0)
        self.combo_mode = QComboBox()
        self.combo_mode.addItem("0: 原始 PID", 0)
        self.combo_mode.addItem("1: GS-PID", 1)
        self.combo_mode.addItem("2: LQR", 2)
        self.combo_mode.addItem("3: MPC", 3)
        self.combo_mode.setCurrentIndex(2)
        cl.addWidget(self.combo_mode, 1, 1)

        cg.setLayout(cl)
        layout.addWidget(cg)

        # ---- 控制按钮 ----
        bg = QGroupBox("控制")
        bl = QHBoxLayout()

        self.btn_start = QPushButton("🚀 一键起飞并执行轨迹")
        self.btn_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; font-size: 14px;")
        self.btn_start.setMinimumHeight(50)
        self.btn_start.clicked.connect(self._on_start)

        self.btn_land = QPushButton("🛬 降落")
        self.btn_land.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.btn_land.setMinimumHeight(50)
        self.btn_land.clicked.connect(self._on_land)

        self.btn_estop = QPushButton("⛔ 急停")
        self.btn_estop.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.btn_estop.setMinimumHeight(50)
        self.btn_estop.clicked.connect(self._on_estop)

        bl.addWidget(self.btn_start)
        bl.addWidget(self.btn_land)
        bl.addWidget(self.btn_estop)
        bg.setLayout(bl)
        layout.addWidget(bg)

        # ---- 日志下载 ----
        dg = QGroupBox("日志")
        dl = QHBoxLayout()
        self.btn_download = QPushButton("⬇ 下载最新日志")
        self.btn_download.clicked.connect(self._on_download)
        dl.addWidget(self.btn_download)
        dg.setLayout(dl)
        layout.addWidget(dg)

        # ---- 日志窗口 ----
        lg = QGroupBox("运行日志")
        ll = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(font_mono)
        self.log_text.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4;")
        ll.addWidget(self.log_text)
        lg.setLayout(ll)
        layout.addWidget(lg)

    def _on_state(self, state):
        color = "green" if state.get("connected") else "gray"
        self.lbl_conn.setText(f"<span style='color:{color}'>●</span> {'已连接' if state.get('connected') else '未连接'}")
        self.lbl_mode.setText(f"模式: {state.get('mode', '--')}")
        self.lbl_arm.setText("解锁" if state.get("armed") else "锁定")

    def _on_pos(self, pos):
        self.lbl_pos.setText(f"X:{pos['x']:.2f} Y:{pos['y']:.2f} Z:{pos['z']:.2f}")

    def _on_battery(self, batt):
        v = batt.get("voltage", 0)
        p = batt.get("percentage", 0)
        self.lbl_batt.setText(f"电池: {v:.1f}V ({p:.0f}%)")

    def _on_log(self, msg):
        self.log_text.append(msg)

    def _update_ui(self):
        pass

    def _on_start(self):
        traj = self.combo_traj.currentText()
        mode = self.combo_mode.currentData()

        reply = QMessageBox.question(
            self, "确认起飞",
            f"即将执行:\n轨迹: {traj}\n控制器: {self.combo_mode.currentText()}\n\n"
            f"请确认:\n1. 螺旋桨已安装且周围无人\n2. 遥控器在手边\n3. 当前位置在安全空间内",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            threading.Thread(target=self.ros.start_flight, args=(traj, mode), daemon=True).start()

    def _on_land(self):
        threading.Thread(target=self.ros.land, daemon=True).start()

    def _on_estop(self):
        reply = QMessageBox.warning(
            self, "确认急停",
            "急停将立即切 STABILIZED 并 DISARM!\n是否继续?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            threading.Thread(target=self.ros.emergency_stop, daemon=True).start()

    def _on_download(self):
        self.log_text.append("[LOG] 启动日志下载...")
        threading.Thread(target=self._download_logs_thread, daemon=True).start()

    def _download_logs_thread(self):
        try:
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "download_logs.py")
            result = subprocess.run(
                ["python3", script],
                capture_output=True, text=True, timeout=120
            )
            self.log_sig.emit(result.stdout)
            if result.stderr:
                self.log_sig.emit(f"[ERROR] {result.stderr}")
        except Exception as e:
            self.log_sig.emit(f"[ERROR] 下载日志失败: {e}")


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
