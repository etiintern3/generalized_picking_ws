#!/usr/bin/env python3
"""Publish stamped bbox grid prompts over the tray ROI for Isaac ROS SAM."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# Tuned tray interior ROI (pixels)
#X_MIN, X_MAX = 540, 705
#Y_MIN, Y_MAX = 215, 475
X_MIN, X_MAX = 220, 1000
Y_MIN, Y_MAX = 500, 720

NX, NY = 6, 6
BOX_W, BOX_H = 100.0, 100.0

CELL_W = (X_MAX - X_MIN) / NX
CELL_H = (Y_MAX - Y_MIN) / NY
CENTERS = [
    (X_MIN + (i + 0.5) * CELL_W, Y_MIN + (j + 0.5) * CELL_H)
    for j in range(NY)
    for i in range(NX)
]


class TrayGridPrompts(Node):
    def __init__(self):
        super().__init__('tray_grid_prompts')
        self.pub = self.create_publisher(Detection2DArray, '/prompts', 10)
        self.create_subscription(Image, '/camera/realsense/rgb', self.on_image, 10)
        self.get_logger().info(
            f'ROI ({X_MIN},{Y_MIN})-({X_MAX},{Y_MAX})  '
            f'{len(CENTERS)} boxes {BOX_W}x{BOX_H}'
        )

    def on_image(self, img: Image):
        msg = Detection2DArray()
        msg.header = img.header
        for x, y in CENTERS:
            det = Detection2D()
            det.header = img.header
            det.bbox.center.position.x = float(x)
            det.bbox.center.position.y = float(y)
            det.bbox.size_x = BOX_W
            det.bbox.size_y = BOX_H
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'grid'
            hyp.hypothesis.score = 1.0
            det.results.append(hyp)
            msg.detections.append(det)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TrayGridPrompts()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
