#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class CameraToBaseTransformer(Node):
    def __init__(self):
        super().__init__('camera_to_base_transformer')

        self.sub = self.create_subscription(
            PointStamped,
            '/goal_point',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/goal_point_transformed',
            10
        )

        self.get_logger().info("📡 Transformer-node aktivert.")

    def callback(self, msg):
        # Hent punkt fra kamera
        X_cam = msg.point.x
        Y_cam = msg.point.y
        Z_cam = msg.point.z

        # Transformasjon til robotbase
        X_base = Z_cam + 0.125      # fremover
        Y_base = -X_cam + 0.025            # venstre/høyre
        Z_base = -Y_cam + 0.09    # oppover

        # Lag nytt punkt
        transformed = PointStamped()
        transformed.header.stamp = self.get_clock().now().to_msg()
        transformed.header.frame_id = "robot_base"
        transformed.point.x = X_base
        transformed.point.y = Y_base
        transformed.point.z = Z_base

        self.pub.publish(transformed)

        self.get_logger().info(
            f"🎯 Transformert punkt: X={X_base:.3f}, Y={Y_base:.3f}, Z={Z_base:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()