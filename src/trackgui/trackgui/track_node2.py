#!/usr/bin/env python3
import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))

import cv2
from PySide6 import QtCore, QtGui
from PySide6.QtCore import QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import Qt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from trackgui.msg import Target  # 导入自定义消息

import kcf.tracker
import kcf.run

from enum import Enum

class MOVEMISSON(Enum):
    NONE_MOVE = 0
    CLICK_MOVE = 1
    TRACK_MOVE = 2

# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py

from .ui_form import Ui_MainWindow  # 注意这里的相对导入

class MainWindow(QMainWindow, Node):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent, node_name='fixed_wing_tracker_node')
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        Node.__init__(self, 'fixed_wing_tracker_node')
        # self.image_pub = self.create_publisher(Image, '/tracked_image', 10)
        self.target_pub = self.create_publisher(Target, '/target_info', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        self.cap_width = 640
        self.cap_height = 360
        self.fps = 0
        self.timer = QtCore.QTimer(self)
        self.timer.start(20)

        self.ui.imageLabel.mousePressEvent = self.monitor_mousePressEvent
        self.ui.imageLabel.mouseMoveEvent = self.monitor_mouseMoveEvent
        self.ui.imageLabel.mouseReleaseEvent = self.monitor_mouseReleaseEvent
        self.ui.imageLabel.wheelEvent = self.monitor_mouseWheelEvent

        self.track_window_timer = QtCore.QTimer(self)
        self.track_window_timer.timeout.connect(self.check_track_window)
        self.track_window_timer.start(200)

        self.debug_draw = False
        self.ui.imageLabel.start_pos = None
        self.ui.imageLabel.end_pos = None
        self.selection = None
        self.mouse_clicked = None
        self.click_window_size = 15
        self.mission_flag = MOVEMISSON.NONE_MOVE
        self.tracker = None
        self.track_window = None
        self.mission_initialized = False
        self.target = Target()

        self.show_wheel_box = False
        self.show_wheel_timer = QtCore.QTimer(self)
        self.show_wheel_timer.timeout.connect(self.hide_wheel_box)

        self.init_slot()

    def init_slot(self):
        self.timer.timeout.connect(self.showimg)
        self.ui.trackButton.clicked.connect(self.track_confirm)
        self.ui.clickButton.clicked.connect(self.click_confirm)

    def check_track_window(self):
        if self.mission_flag == MOVEMISSON.NONE_MOVE:
            self.ui.trackButton.setEnabled(self.track_window is not None)

    def hide_wheel_box(self):
        self.show_wheel_box = False
        self.show_wheel_timer.stop()

    def monitor_mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.mission_flag == MOVEMISSON.NONE_MOVE:
                self.ui.imageLabel.start_pos = event.position().toPoint()
                self.ui.imageLabel.end_pos = event.position().toPoint()
                self.track_window = None
            elif self.mission_flag == MOVEMISSON.CLICK_MOVE:
                self.mouse_clicked = event.position().toPoint()

    def monitor_mouseMoveEvent(self, event):
        if self.ui.imageLabel.start_pos is not None:
            if self.mission_flag == MOVEMISSON.NONE_MOVE:
                self.ui.imageLabel.end_pos = event.position().toPoint()
                xmin = min(self.ui.imageLabel.start_pos.x(), self.ui.imageLabel.end_pos.x())
                ymin = min(self.ui.imageLabel.start_pos.y(), self.ui.imageLabel.end_pos.y())
                xmax = max(self.ui.imageLabel.start_pos.x(), self.ui.imageLabel.end_pos.x())
                ymax = max(self.ui.imageLabel.start_pos.y(), self.ui.imageLabel.end_pos.y())
                self.selection = (xmin, ymin, xmax, ymax)
            elif self.mission_flag == MOVEMISSON.CLICK_MOVE:
                self.mouse_clicked = None

    def monitor_mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.mission_flag == MOVEMISSON.NONE_MOVE:
                self.track_window = self.selection
                self.ui.imageLabel.start_pos = None
                self.selection = None
                self.mission_initialized = False
            elif self.mission_flag == MOVEMISSON.CLICK_MOVE:
                if self.mouse_clicked is not None:
                    x = self.mouse_clicked.x()
                    y = self.mouse_clicked.y()
                    size = self.click_window_size
                    self.track_window = (x - size, y - size, x + size, y + size)
                    self.mouse_clicked = None
                    self.mission_initialized = False

    def monitor_mouseWheelEvent(self, event):
        if self.mission_flag == MOVEMISSON.CLICK_MOVE:
            delta = event.angleDelta().y()
            self.click_window_size = min(max(self.click_window_size + (5 if delta > 0 else -5), 10), 100)
            self.show_wheel_box = True
            self.show_wheel_timer.start(500)

    def track_confirm(self):
        if self.mission_flag == MOVEMISSON.TRACK_MOVE:
            self.mission_flag = MOVEMISSON.NONE_MOVE
            self.track_window = None
            self.selection = None
            self.ui.trackButton.setText("锁定")
            self.ui.trackButton.setStyleSheet("")
            self.ui.clickButton.setEnabled(True)
        else:
            self.mission_flag = MOVEMISSON.TRACK_MOVE
            self.ui.trackButton.setText("取消锁定")
            self.ui.trackButton.setStyleSheet("background-color: red;")
            self.ui.clickButton.setEnabled(False)

    def click_confirm(self):
        if self.mission_flag == MOVEMISSON.CLICK_MOVE:
            self.mission_flag = MOVEMISSON.NONE_MOVE
            self.track_window = None
            self.ui.clickButton.setText("漫游")
            self.ui.clickButton.setStyleSheet("")
            self.ui.trackButton.setEnabled(True)
        else:
            self.mission_flag = MOVEMISSON.CLICK_MOVE
            self.ui.clickButton.setText("取消漫游")
            self.ui.clickButton.setStyleSheet("background-color: red;")
            self.ui.trackButton.setEnabled(False)

    def showimg(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read from camera, exiting...")
            sys.exit(1)

        start = cv2.getTickCount()
        frame = cv2.resize(frame, (self.cap_width, self.cap_height))
        frame = cv2.flip(frame, 1)

        if self.mission_flag in (MOVEMISSON.TRACK_MOVE, MOVEMISSON.CLICK_MOVE) and self.track_window:
            if not self.mission_initialized:
                exact_track_window = list(self.track_window)
                exact_track_window[2] = exact_track_window[2] - exact_track_window[0]
                exact_track_window[3] = exact_track_window[3] - exact_track_window[1]
                self.tracker = kcf.tracker.KCFTracker(True, True, True)
                self.tracker.init(tuple(exact_track_window), frame)
                self.mission_initialized = True

            try:
                bbox = list(map(int, self.tracker.update(frame)))
                p1 = (bbox[0], bbox[1])
                p2 = (bbox[0] + bbox[2], bbox[1] + bbox[3])
                kcf.run.draw_military_lock(frame, p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1], False, (0, 0, 255))
                
                self.target.detect_flag = 1
                self.target.cx = int(((p2[0] - p1[0]) / 2 + p1[0]) / self.cap_width * 100)
                self.target.cy = int(((p2[1] - p1[1]) / 2 + p1[1]) / self.cap_height * 100)
                self.target.dx = int((p2[0] - p1[0]) / self.cap_width * 100)
                self.target.dy = int((p2[1] - p1[1]) / self.cap_height * 100)
            except Exception as e:
                self.get_logger().warn(f"Tracking failed: {str(e)}")
                self.target.detect_flag = 0 # 目标丢失，取消追踪
                self.mission_initialized = False
                self.track_window = None

        if (self.mission_flag == MOVEMISSON.NONE_MOVE or self.debug_draw):
            self.target.detect_flag = 0 # 取消跟踪
            if self.track_window is not None:
                cv2.rectangle(frame, (self.track_window[0], self.track_window[1]),
                            (self.track_window[2], self.track_window[3]), (0, 255, 255), 2)
            elif self.selection is not None:
                cv2.rectangle(frame, (self.selection[0], self.selection[1]),
                            (self.selection[2], self.selection[3]), (0, 255, 0), 2)

        if self.mission_flag == MOVEMISSON.CLICK_MOVE:
            if self.show_wheel_box:
                center_x, center_y = self.cap_width // 2, self.cap_height // 2
                p1 = (center_x - self.click_window_size, center_y - self.click_window_size)
                p2 = (center_x + self.click_window_size, center_y + self.click_window_size)
                kcf.run.draw_military_lock(frame, p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1], False, (0, 0, 255))

        # 发布消息
        self.target_pub.publish(self.target)

        self.fps = cv2.getTickFrequency() / (cv2.getTickCount() - start)
        cv2.putText(frame, f"FPS: {min(int(self.fps), 99)}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 170, 50), 1)
        cross_length = 20
        cv2.line(frame, (self.cap_width // 2 - cross_length, self.cap_height // 2),
                (self.cap_width // 2 + cross_length, self.cap_height // 2), (0, 255, 0), 1)
        cv2.line(frame, (self.cap_width // 2, self.cap_height // 2 - cross_length),
                (self.cap_width // 2, self.cap_height // 2 + cross_length), (0, 255, 0), 1)

        im = QImage(frame.data, frame.shape[1], frame.shape[0], frame.shape[2] * frame.shape[1],
                   QImage.Format_BGR888)
        self.ui.imageLabel.setPixmap(QPixmap.fromImage(im))

        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()

    from threading import Thread
    def spin():
        rclpy.spin(widget)
    
    spin_thread = Thread(target=spin)
    spin_thread.daemon = True
    spin_thread.start()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()