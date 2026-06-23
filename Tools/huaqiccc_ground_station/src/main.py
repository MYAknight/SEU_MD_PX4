#!/usr/bin/env python3
"""
Huaqiccc 变形无人机地面站入口

用法:
    python3 src/main.py

依赖:
    PyQt5, PyYAML, rospy (可选)
"""
import sys
import os

# 确保 src 目录在路径中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication
from ground_station import GroundStation


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # 设置全局暗色样式
    app.setStyleSheet("""
        QMainWindow, QWidget {
            background-color: #2b2b2b;
            color: #eeeeee;
        }
        QGroupBox {
            font-weight: bold;
            border: 1px solid #555555;
            border-radius: 4px;
            margin-top: 8px;
            padding-top: 8px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 4px;
            color: #88ccff;
        }
        QPushButton {
            background-color: #3d3d3d;
            border: 1px solid #555555;
            border-radius: 3px;
            padding: 4px 12px;
            color: #eeeeee;
        }
        QPushButton:hover {
            background-color: #4d4d4d;
        }
        QPushButton:pressed {
            background-color: #5d5d5d;
        }
        QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
            background-color: #1a1a1a;
            border: 1px solid #555555;
            color: #eeeeee;
            padding: 2px 4px;
        }
        QLabel {
            color: #cccccc;
        }
        QTextEdit {
            background-color: #1a1a1a;
            color: #00ff00;
            border: 1px solid #555555;
        }
    """)

    window = GroundStation()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
