#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image as image_msg
from sensor_msgs.msg import CameraInfo as camerainfo_msg
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped as point_msg
from cv_bridge import CvBridge


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_node')

        # === ROS subscriptions ===
        self.sub_intrinsics = self.create_subscription(
            camerainfo_msg,
            '/camera/aligned_depth_to_color/camera_info',
            self.sub_intrinsics_callback,
            10)

        self.sub_color = self.create_subscription(
            image_msg,
            '/camera/color/image_raw',
            self.sub_color_callback,
            10)

        self.sub_depth = self.create_subscription(
            image_msg,
            '/camera/aligned_depth_to_color/image_raw',
            self.sub_depth_callback,
            10)

        # === ROS publishers ===
        self.pub_point = self.create_publisher(point_msg, '/goal_point', 10)
        self.pub_angle_orange = self.create_publisher(Float32, '/angle_orange', 10)
        self.pub_angle_green = self.create_publisher(Float32, '/angle_green', 10)

        # === Utilities ===
        self.br = CvBridge()
        self.centroids = {'orange': np.array([0, 0]), 'green': np.array([0, 0])}
        self.depth = {'orange': 0.0, 'green': 0.0}
        self.intrinsics = rs.intrinsics()
        self.last_publish_time = 0.0

        self.get_logger().info("✅ Image Processor node initialized (circle detection mode).")


    # === Color image callback ===
    def sub_color_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Convert to HSV (RealSense outputs RGB)
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_RGB2HSV)

        # === ORANGE === (low-red + high-red range)
        lower_orange1 = np.array([10, 100, 100])
        upper_orange1 = np.array([20, 255, 255])
        lower_orange2 = np.array([170, 100, 100])
        upper_orange2 = np.array([180, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange1, upper_orange1) | cv2.inRange(hsv, lower_orange2, upper_orange2)

        # === GREEN ===
        lower_green = np.array([35, 60, 60])
        upper_green = np.array([85, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

        # Circle detection for both colors
        self.detect_circle(mask_orange, 'orange', current_frame, (0, 165, 255))
        self.detect_circle(mask_green, 'green', current_frame, (0, 255, 0))

        cv2.imshow("Color Image", current_frame)
        cv2.waitKey(1)


    # === Circle detection helper ===
    def detect_circle(self, mask, color_name, frame, draw_color):
        blurred = cv2.GaussianBlur(mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=100,
            param2=20,
            minRadius=5,
            maxRadius=150
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            largest_circle = max(circles[0, :], key=lambda c: c[2])
            cx, cy, radius = largest_circle
            self.centroids[color_name] = np.array([cx, cy])

            cv2.circle(frame, (cx, cy), radius, draw_color, 2)
            cv2.circle(frame, (cx, cy), 3, (255, 255, 255), -1)

            # Optional: compute angle
            if self.intrinsics.fx != 0:
                angle_rad = np.arctan2((cx - self.intrinsics.ppx), self.intrinsics.fx)
                angle_deg = np.degrees(angle_rad)
                msg = Float32()
                msg.data = float(angle_deg)
                if color_name == 'orange':
                    self.pub_angle_orange.publish(msg)
                else:
                    self.pub_angle_green.publish(msg)


    # === Depth callback ===
    def sub_depth_callback(self, msg):
        depth_image = self.br.imgmsg_to_cv2(msg)
        for color, centroid in self.centroids.items():
            try:
                d = float(depth_image[int(centroid[1]), int(centroid[0])]) / 1000.0  # mm → m
                self.depth[color] = d
            except Exception:
                pass

        self.publish_nearest_ball()


    # === Camera intrinsics callback ===
    def sub_intrinsics_callback(self, msg):
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]
        self.intrinsics.model = rs.distortion.none
        self.intrinsics.coeffs = [i for i in msg.d]


    # === Publish nearest circle only ===
    def publish_nearest_ball(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_publish_time < 1.0:
            return  # throttle

        valid_depths = {c: d for c, d in self.depth.items() if d > 0.0}
        if not valid_depths:
            return

        nearest_color = min(valid_depths, key=valid_depths.get)
        u, v = self.centroids[nearest_color]
        depth = self.depth[nearest_color]

        X = (u - self.intrinsics.ppx) * depth / self.intrinsics.fx
        Y = (v - self.intrinsics.ppy) * depth / self.intrinsics.fy
        Z = depth

        point = point_msg()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "camera_color_optical_frame"
        point.point.x = X
        point.point.y = Y
        point.point.z = Z

        self.pub_point.publish(point)
        self.last_publish_time = current_time

        self.get_logger().info(f"🎯 Target = {nearest_color.upper()} | X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")


    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()