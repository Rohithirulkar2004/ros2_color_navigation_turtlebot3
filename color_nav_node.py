#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2
import numpy as np


class ColorNavNode(Node):

    def __init__(self):
        super().__init__('color_nav_node')

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Controller parameters
        self.kp = 0.003
        self.linear_speed = 0.15
        self.angular_limit = 1.0

        # LiDAR distance
        self.front_distance = 999.0

        # Tracking memory
        self.last_direction = 1

        self.get_logger().info("Color Navigation Node Started")

    # =====================================
    # LiDAR Callback
    # =====================================
    def scan_callback(self, msg):

        ranges = np.array(msg.ranges)

        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            self.front_distance = np.min(valid_ranges)

    # =====================================
    # Camera Callback
    # =====================================
    def image_callback(self, msg):

        self.get_logger().info("Camera callback running")

        try:

            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            self.get_logger().info("Image converted")

            height, width, _ = frame.shape
            image_center_x = width // 2

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Green range
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])

            # Create mask
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Find contours
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            self.get_logger().info(
                f"Contours found: {len(contours)}"
            )

            twist = Twist()

            # =====================================
            # Object Found
            # =====================================
            if len(contours) > 0:

                largest_contour = max(
                    contours,
                    key=cv2.contourArea
                )

                area = cv2.contourArea(largest_contour)

                if area > 500:

                    M = cv2.moments(largest_contour)

                    if M["m00"] != 0:

                        cx = int(M["m10"] / M["m00"])

                        # Draw center
                        cv2.circle(
                            frame,
                            (cx, height // 2),
                            5,
                            (0, 0, 255),
                            -1
                        )

                        # Error
                        error = image_center_x - cx

                        # Save direction
                        if error > 0:
                            self.last_direction = 1
                        else:
                            self.last_direction = -1

                        # P controller
                        twist.angular.z = self.kp * error

                        # Limit angular speed
                        twist.angular.z = max(
                            min(
                                twist.angular.z,
                                self.angular_limit
                            ),
                            -self.angular_limit
                        )

                        # Move forward if aligned
                        if abs(error) < 30:

                            if self.front_distance > 0.35:
                                twist.linear.x = self.linear_speed
                            else:
                                twist.linear.x = 0.0
                                twist.angular.z = 0.0

                                self.get_logger().info(
                                    "Reached Green Object"
                                )

            # =====================================
            # Object Lost
            # =====================================
            else:

                twist.angular.z = 0.3 * self.last_direction

            # Publish motion
            self.cmd_pub.publish(twist)

            # Show OpenCV windows
            cv2.imshow("Camera View", frame)
            cv2.imshow("Green Mask", mask)

            cv2.waitKey(1)

        except Exception as e:

            self.get_logger().error(str(e))


# =====================================
# Main
# =====================================
def main(args=None):

    rclpy.init(args=args)

    node = ColorNavNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()