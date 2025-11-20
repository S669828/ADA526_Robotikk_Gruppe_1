#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

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

        # Busy flag
        self.busy = False

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

    def solve_and_publish(self, Tg, label):
        """Løs IK for målposen og publiser joints en gang."""
        sol = self.robot.ikine_LM(
            Tg,
            q0=self.robot.q,
            mask=[1, 1, 1, 0, 0, 0]
        )
        if not sol.success:
            self.get_logger().warn(f"❌ IK feilet for {label}.")
            return False

        self.robot.q = sol.q
        self.robot_plot.step()

        msg_out = Float32MultiArray()
        msg_out.data = [float(q) for q in self.robot.q]
        self.pub.publish(msg_out)
        self.get_logger().info(f"✅ Publisert joints for {label}: {msg_out.data}")
        return True

    def run_manual_loop(self):
        while rclpy.ok():
            try:
                if self.busy:
                    print("⏳ Robot er opptatt, vent til sekvensen er ferdig...")
                    time.sleep(0.2)
                    continue

                user_input = input("👉 Skriv inn målpunkt (X Y Z) i meter, eller 'q' for å avslutte: ")
                if user_input.strip().lower() == 'q':
                    break

                parts = user_input.strip().split()
                if len(parts) != 3:
                    print("⚠️ Du må skrive inn tre tall: X Y Z")
                    continue

                x, y, z = map(float, parts)
                self.get_logger().info(f"📍 Mottatt punkt: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

                # Sett busy for hele sekvensen
                self.busy = True

                # Definer tre posisjoner: over (z+0.01), mål (z), over (z+0.01)
                z_above = z + 0.05
                Tg_above_1 = SE3(x, y, z_above) * SE3.Rx(np.pi)
                Tg_target   = SE3(x, y, z)      * SE3.Rx(np.pi)
                Tg_above_2 = SE3(x, y, z_above) * SE3.Rx(np.pi)

                # 1) Gå til 1 cm over
                if not self.solve_and_publish(Tg_above_1, "1 cm over (steg 1/3)"):
                    self.busy = False
                    continue
                time.sleep(8)

                # 2) Gå ned til målet
                if not self.solve_and_publish(Tg_target, "på målpunktet (steg 2/3)"):
                    self.busy = False
                    continue
                time.sleep(5)

                # 3) Gå opp igjen til 1 cm over
                if not self.solve_and_publish(Tg_above_2, "tilbake 1 cm over (steg 3/3)"):
                    self.busy = False
                    continue

                # Ferdig med sekvens
                self.busy = False

            except Exception as e:
                self.get_logger().error(f"Feil ved input: {e}")
                self.busy = False


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