#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

from adatools import config_generator as cg
from adatools import plotting_tools as pt


class DXLStateVisualizer(Node):
    def __init__(self):
        super().__init__("dxl_state_visualizer")

        # Konfigurer robotmodell
        self.robot = cg.get_robot_config_1(
            link1=0.1815, link1_offset=0.0,
            link2=0.2,    link2_offset=0.0,
            link3=0.2,    link3_offset=0.0,
            link4=0.372,    link4_offset=0.0
        )
        self.robot.q = [0.0, np.pi/2, 0.0, 0.0, 0.0]

        # Lag plott
        self.robot_teach = self.robot.teach(self.robot.q, backend='pyplot', block=False)
        self.plot = pt.plot_baseplate(self.robot_teach)

        # Subscriber for faktiske motorposisjoner
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/dxl_joint_state',
            self.state_callback,
            10
        )

        self.get_logger().info("DXL State Visualizer startet – viser faktiske motorposisjoner")

    def state_callback(self, msg: Float32MultiArray):
        if len(msg.data) == len(self.robot.q):
            self.robot.q = list(msg.data)
            self.robot_teach.q = self.robot.q
            self.plot.step()
            self.get_logger().info(f"Oppdaterer plott med faktiske posisjoner: {msg.data}")
        else:
            self.get_logger().warn(f"Forventet {len(self.robot.q)} verdier, fikk: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = DXLStateVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()