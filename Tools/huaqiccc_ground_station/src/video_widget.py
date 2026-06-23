#!/usr/bin/env python3
"""
视频显示组件 - 支持鼠标点击选中柱子
"""
import math
from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont
import numpy as np


class VideoWidget(QWidget):
    """
    视频显示控件，支持：
    - 显示实时视频帧
    - 鼠标点击选中检测目标
    - 叠加锁定框和状态信息
    """

    # 信号：用户点击画面，发送点击的归一化坐标 (x, y) 和画面尺寸 (w, h)
    target_selected = pyqtSignal(int, int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 360)
        self.setStyleSheet("background-color: #1a1a1a;")

        self._frame = None          # 当前视频帧 (numpy BGR)
        self._overlay = None        # 叠加层
        self._lock_center = None    # 锁定目标的中心点 (px, py)
        self._lock_id = -1          # 锁定目标的 ID
        self._status_text = "等待视频..."
        self._status_color = QColor(255, 255, 0)

        # 定时刷新
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(33)  # ~30fps

    def set_frame(self, frame):
        """设置当前视频帧 (numpy BGR array)"""
        if frame is not None and frame.size > 0:
            self._frame = frame.copy()

    def set_lock_info(self, target_id, center_px=None):
        """设置锁定目标信息"""
        self._lock_id = target_id
        self._lock_center = center_px

    def set_status(self, text, color=QColor(255, 255, 0)):
        self._status_text = text
        self._status_color = color

    def paintEvent(self, event):
        painter = QPainter(self)
        w, h = self.width(), self.height()

        # 绘制背景
        painter.fillRect(self.rect(), QColor(26, 26, 26))

        if self._frame is not None:
            # 转换 numpy BGR -> QImage -> QPixmap
            img_h, img_w = self._frame.shape[:2]
            rgb = self._frame[:, :, ::-1]  # BGR -> RGB
            qimg = QImage(rgb.data, img_w, img_h, img_w * 3, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)

            # 等比例缩放适应控件
            scaled = pixmap.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            x_offset = (w - scaled.width()) // 2
            y_offset = (h - scaled.height()) // 2
            painter.drawPixmap(x_offset, y_offset, scaled)

            # 计算缩放比例
            scale_x = scaled.width() / img_w
            scale_y = scaled.height() / img_h

            # 绘制锁定框
            if self._lock_id >= 0 and self._lock_center is not None:
                cx = int(self._lock_center[0] * scale_x) + x_offset
                cy = int(self._lock_center[1] * scale_y) + y_offset
                pen = QPen(QColor(0, 255, 0), 3)
                painter.setPen(pen)
                size = 40
                painter.drawRect(cx - size, cy - size, size * 2, size * 2)
                painter.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
                painter.drawText(cx - size, cy - size - 5, f"LOCKED #{self._lock_id}")

            # 绘制画面中心十字线
            pen = QPen(QColor(128, 128, 128), 1, Qt.DashLine)
            painter.setPen(pen)
            cx_screen = w // 2
            cy_screen = h // 2
            painter.drawLine(cx_screen, 0, cx_screen, h)
            painter.drawLine(0, cy_screen, w, cy_screen)
        else:
            # 无画面时显示提示
            painter.setPen(QColor(200, 200, 200))
            painter.setFont(QFont("Microsoft YaHei", 16))
            painter.drawText(self.rect(), Qt.AlignCenter, self._status_text)

        # 绘制状态栏
        painter.fillRect(0, h - 28, w, 28, QColor(0, 0, 0, 180))
        painter.setPen(self._status_color)
        painter.setFont(QFont("Microsoft YaHei", 10))
        painter.drawText(10, h - 8, self._status_text)

    def mousePressEvent(self, event):
        """鼠标点击事件 - 选中柱子"""
        if self._frame is None:
            return

        img_h, img_w = self._frame.shape[:2]
        w, h = self.width(), self.height()

        # 计算缩放后的图像在控件中的位置
        scale = min(w / img_w, h / img_h)
        scaled_w = img_w * scale
        scaled_h = img_h * scale
        x_offset = (w - scaled_w) / 2
        y_offset = (h - scaled_h) / 2

        # 将鼠标坐标转换回图像坐标
        mx = event.x() - x_offset
        my = event.y() - y_offset
        img_x = int(mx / scale)
        img_y = int(my / scale)

        # 检查是否在图像范围内
        if 0 <= img_x < img_w and 0 <= img_y < img_h:
            self.target_selected.emit(img_x, img_y, img_w, img_h)
