#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from adatools import config_generator as cg
from adatools import plotting_tools as pt
from spatialmath import SE3
import numpy as np
import time


class IKPublisher(Node):
    def __init__(self):
        super().__init__('ik_goal_publisher')
        self.pub = self.create_publisher(Float32MultiArray, '/dxl_joint_cmd', 10)

        # Lag robot basert på config1
        self.robot = cg.get_robot_config_1(
            link1=0.1815, link1_offset=0.0,
            link2=0.2,    link2_offset=0.0,
            link3=0.2,    link3_offset=0.0,
            link4=0.372,    link4_offset=0.0
        )

        # Plot robot og baseplate
        self.robot_plot = self.robot.plot(self.robot.qr, backend='pyplot')
        pt.plot_baseplate(self.robot_plot)

        # 👉 Sett samme startposisjon som i visualizer_publisher
        self.start_q = [0.0, np.pi/2, 0.0, 0.0, 0.0]
        self.robot.q = self.start_q
        self.robot_plot.step()

        # Publiser startposisjon
        msg = Float32MultiArray()
        msg.data = [float(q) for q in self.robot.q]
        self.pub.publish(msg)
        time.sleep(5)
        # Definer målposene
        self.Tgoals = SE3([
            SE3(0.3, 0, 0.2),
            SE3(0.3, 0, 0),
            SE3(0, 0.55/2, 0.2),
            SE3(0, 0.55/2, 0),
            SE3(0, -0.55/2, 0.2),
            SE3(0, -0.55/2, 0),
            SE3(0.6, 0.55/2, 0.2),
            SE3(0.6, 0.55/2, 0),
            SE3(0.6, -0.55/2, 0.2),
            SE3(0.6, -0.55/2, 0)
        ]) * SE3.Rx(180, 'deg')

        self.Tgoals.plot(ax=self.robot_plot.ax, length=0.1, style='rgb')

    def run_goals(self):
        for i, Tg in enumerate(self.Tgoals):
            input(f"Trykk Enter for å publisere pose {i}...")

            # Bruk startposisjonen som initial guess
            sol = self.robot.ikine_LM(
                Tg,
                q0=self.start_q,
                mask=[1, 1, 1, 0.0, 0.0, 0.0]
            )
            self.robot.q = sol.q
            self.robot_plot.step()

            # Publiser radianer
            msg = Float32MultiArray()
            msg.data = [float(q) for q in self.robot.q]
            self.pub.publish(msg)

            # Logg litt info
            T = self.robot.fkine(self.robot.q)
            angular_distance = Tg.angdist(T)
            trans_distance = np.linalg.norm(Tg.t - T.t)
            manip = self.robot.manipulability(self.robot.q, method='minsingular', axes='all')
            J_rank = np.linalg.matrix_rank(self.robot.jacob0(self.robot.q))

            self.get_logger().info(
                f"Pose {i:03d}: "
                f"TransDist={trans_distance:.4f}, "
                f"AngDist={angular_distance:.4f}, "
                f"Manip={manip:.4g}, "
                f"JacRank={J_rank}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = IKPublisher()
    node.run_goals()   # manuell loop med Enter
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()