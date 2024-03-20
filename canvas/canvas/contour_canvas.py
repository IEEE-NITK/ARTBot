#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import std_msgs.msg as std

botcount = 10
drawing = False
x_prev, y_prev = -1, -1
target = []
target_publishers = []

def draw(event, x, y, flags, param):
    global drawing, x_prev, y_prev
    canvas = param
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            cv2.line(canvas, (x_prev, y_prev), (x, y), (255, 255, 255), 2)
        x_prev, y_prev = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

def get_target_pose(canvas):
    global target
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    _ , thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(canvas, contours, -1, (255, 255, 255), 3)
    contours = contours[0].reshape(-1, 2)
    target = []
    ind = int(contours.shape[0] / botcount)
    for i in range(0, contours.shape[0], ind):
        target.append(contours[i])
        cv2.circle(canvas, tuple(contours[i]), 2, (0, 0, 255), 2)
    publish_msg(canvas)

def publish_msg(canvas):
    global target, target_publishers
    height, width, _ = canvas.shape
    for i in range(botcount):
        x_normalized = target[i][0] / width
        y_normalized = target[i][1] / height
        target_publishers[i].publish(Point(x=x_normalized, y=y_normalized))

class Artbotcanvas(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        self.get_logger().info(f"Number of robots: {botcount}")
        self.get_logger().info("Press 'm' to get target pose from canvas")
        self.get_logger().info("Press 'c' to clear the canvas")
        self.get_logger().info("Press 'q' to quit")

        global target_publishers
        for i in range(botcount):
            target_publishers.append(self.create_publisher(Point, f'/artist{i}/target', 10))

        self.canvas = np.zeros((720, 1280, 3), np.uint8)
        cv2.namedWindow('ARTBOT Canvas')
        cv2.setMouseCallback('ARTBOT Canvas', draw, self.canvas)
        self.timer = self.create_timer(0.001, self.timer_callback)
        # cv2.putText(self.canvas, x_prev, y_prev,
        #             (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

    def timer_callback(self):
        cv2.imshow('ARTBOT Canvas', self.canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == ord('m'):
            get_target_pose(self.canvas)
        elif key == ord('c'):
            self.canvas = np.zeros((720, 1280, 3), np.uint8)
            cv2.namedWindow('ARTBOT Canvas')
            cv2.setMouseCallback('ARTBOT Canvas', draw, self.canvas)
            self.timer = self.create_timer(0.001, self.timer_callback)

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = Artbotcanvas()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()