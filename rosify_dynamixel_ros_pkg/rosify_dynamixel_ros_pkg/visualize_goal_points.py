#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
import numpy as np

from adatools import config_generator as cg
from adatools import plotting_tools as pt
from spatialmath import SE3


class GoalPointVisualizer(Node):
    def __init__(self):
        super().__init__("goal_point_visualizer")

        # Publisher for joint commands
        self.pub = self.create_publisher(Float32MultiArray, '/dxl_joint_cmd', 10)

        # Subscriber på transformerte koordinater
        self.sub = self.create_subscription(
            PointStamped,
            '/goal_point_transformed',
            self.goal_callback,
            10
        )

        # Konfigurer robot
        self.create_config()
        self.create_visualizer()

        # Busy flag
        self.busy = False

        self.get_logger().info("🎨 GoalPointVisualizer startet (busy‑mode).")

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

    def goal_callback(self, msg: PointStamped):
        # Hvis roboten er opptatt, ignorer nye punkter
        if self.busy:
            self.get_logger().info("⏳ Ignorerer nytt punkt (roboten er opptatt).")
            return

        self.busy = True  # sett roboten som opptatt

        # Hent koordinat fra kamera-transformer
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        self.get_logger().info(f"📍 Mottatt målpunkt: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        # Definer sveip-offset i y-retning
        y_offset = 0.05  # 5 cm til venstre/høyre, juster etter behov
        sweep_points = [y - y_offset, y + y_offset]

        # Iterer over punktene (venstre → høyre)
        for yi in sweep_points:
            Tg = SE3(x, yi, z) * SE3.Rx(np.pi)  # snu gripper 180° rundt X

            sol = self.robot.ikine_LM(
                Tg,
                q0=self.robot.q,
                mask=[1, 1, 1, 0, 0, 0]  # bare posisjon, ikke orientering
            )

            if sol.success:
                self.robot.q = sol.q
                self.robot_plot.step()

                msg_out = Float32MultiArray()
                msg_out.data = [float(q) for q in self.robot.q]
                self.pub.publish(msg_out)

                self.get_logger().info(
                    f"✅ IK løst for Y={yi:.3f}. Publiserer q={msg_out.data}"
                )
            else:
                self.get_logger().warn(
                    f"❌ IK kunne ikke løses for punkt Y={yi:.3f}."
                )

        # Ferdig med sveip → klar for neste punkt
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = GoalPointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()