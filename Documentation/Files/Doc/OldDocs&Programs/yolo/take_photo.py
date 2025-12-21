#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class TakePhoto(Node):
    def __init__(self, img_topic, image_dir):
        super().__init__('take_photo_node')
        self.bridge = CvBridge()
        self.image_received = False
        self.cv_image = None
        self.image_sub = self.create_subscription(
            Image,
            img_topic,
            self.callback,
            10  # QoS history depth
        )
        self.image_dir = image_dir
        self.image_count = 1

        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
            self.get_logger().info(f'Created directory: {self.image_dir}')

    def callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv.putText(self.cv_image, f"Image {self.image_count}", (100, 290),
                       cv.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 255), 1)
            self.image_received = True
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')

    def save_picture(self):
        if self.image_received:
            img_title = os.path.join(self.image_dir, f"Foto_{self.image_count}.jpg")
            cv.imwrite(img_title, self.cv_image)
            self.get_logger().info(f'Saved image: {img_title}')
            self.image_received = False # Reset flag after saving
            self.image_count += 1
        else:
            self.get_logger().info("No images received yet")

def main(args=None):
    rclpy.init(args=args)
    node = TakePhoto(
        img_topic=node.declare_parameter('image_topic', '/usb_cam/image_raw').value,
        image_dir=node.declare_parameter('image_dir', '/home/user/rUBot_mecanum_ws/src/rubot_projects/photos').value
    )
    rate = node.create_rate(1)  # 1 Hz
    try:
        while rclpy.ok():
            node.save_picture()
            rate.sleep()
    except KeyboardInterrupt:
        node.get_logger().info('Stopping take_photo node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()