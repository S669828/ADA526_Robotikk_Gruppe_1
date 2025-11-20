#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

from adatools import config_generator as cg
from adatools import plotting_tools as pt
from spatialmath import SE3


class ManualGoalPointVisualizer(Node):
    def __init__(self):
        super().__init__("manual_goal_point_visualizer")

        # Publisher for joint commands
        self.pub = self.create_publisher(Float32MultiArray, '/dxl_joint_cmd', 10)

        # Konfigurer robot
        self.create_config()
        self.create_visualizer()

        self.get_logger().info("✏️ ManualGoalPointVisualizer startet. Skriv inn X Y Z i terminalen.")

    def create_config(self):
        # Konfigurer robotens lenker
        self.robot = cg.get_robot_config_1(
            link1=0.1815, link1_offset=0.0,
            link2=0.2,    link2_offset=0.0,
            link3=0.2,    link3_offset=0.0,
            link4=0.354,  link4_offset=0.0
        )
        # Startkonfigurasjon
        self.start_q = [0.0, np.pi/2, 0.0, 0.0, 0.0]
        self.robot.q = self.start_q

    def create_visualizer(self):
        self.robot_plot = self.robot.plot(self.robot.q, backend='pyplot')
        pt.plot_baseplate(self.robot_plot)

    def run_manual_loop(self):
        while rclpy.ok():
            try:
                user_input = input("👉 Skriv inn målpunkt (X Y Z) i meter, eller 'q' for å avslutte: ")
                if user_input.strip().lower() == 'q':
                    break

                # Parse input
                parts = user_input.strip().split()
                if len(parts) != 3:
                    print("⚠️ Du må skrive inn tre tall: X Y Z")
                    continue

                x, y, z = map(float, parts)
                self.get_logger().info(f"📍 Mottatt manuelt punkt: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

                # Lag SE3-transformasjon for målet
                Tg = SE3(x, y, z) * SE3.Rx(np.pi)  # snu gripper 180° rundt X

                # Løs IK
                sol = self.robot.ikine_LM(
                    Tg,
                    q0=self.start_q,
                    mask=[1, 1, 1, 0, 0, 0]  # bare posisjon
                )

                if sol.success:
                    self.robot.q = sol.q
                    self.robot_plot.step()

                    # Publiser radianer
                    msg_out = Float32MultiArray()
                    msg_out.data = [float(q) for q in self.robot.q]
                    self.pub.publish(msg_out)

                    self.get_logger().info(f"✅ IK løst. Publiserer q={msg_out.data}")
                else:
                    self.get_logger().warn("❌ IK kunne ikke løses for dette punktet.")

            except Exception as e:
                self.get_logger().error(f"Feil ved input: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ManualGoalPointVisualizer()
    try:
        node.run_manual_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()