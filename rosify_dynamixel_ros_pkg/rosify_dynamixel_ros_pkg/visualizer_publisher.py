#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray   # 👈 endret til Float32MultiArray
import numpy as np

from adatools import config_generator as cg
from adatools import plotting_tools as pt


class MyDynamixelVisualizationPub(Node):
    def __init__(self) -> None:
        super().__init__("my_dynamixel_visualizer")
        self.pub = self.create_publisher(Float32MultiArray, '/dxl_joint_cmd', 10)

        # Konfigurer robotens lenker
        self.create_config()
        self.create_visualizer()

        # Kjør timer som publiserer hvert sekund
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Visualizer Publisher created")

    def create_config(self):
        # Konfigurer robotens lenker
        self.my_conf_robot = cg.get_robot_config_1(
            link1=0.1815, link1_offset=0.0,
            link2=0.2,    link2_offset=0.0,
            link3=0.2,    link3_offset=0.0,
            link4=0.372,    link4_offset=0.0
        )
        # Startkonfigurasjon (radianer)
        self.my_conf_robot.q = [0.0, np.pi/2, 0.0, 0.0, 0.0]

    def create_visualizer(self):
        self.robot_teach = self.my_conf_robot.teach(
            self.my_conf_robot.q, backend='pyplot', block=False
        )
        self.plot = pt.plot_baseplate(self.robot_teach)

    def timer_callback(self):
        # Publiser radianer direkte (float)
        msg = Float32MultiArray()
        msg.data = [float(q) for q in self.my_conf_robot.q]
        self.pub.publish(msg)

        self.get_logger().info(f"Publiserer vinkler (rad): {msg.data}")

        # Oppdater plottet
        self.plot.step()


def main(args=None):
    rclpy.init(args=args)
    node = MyDynamixelVisualizationPub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()