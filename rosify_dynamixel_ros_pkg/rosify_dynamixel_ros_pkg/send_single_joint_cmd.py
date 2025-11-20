#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import degrees as r2d
import numpy as np

from custom_dxl.CustomDXL import CustomDXL
import dynamixel_sdk as dxl  # for å lese present position


def deg_to_ticks(deg: float) -> int:
    """Konverter grader til Dynamixel ticks (0–4095)."""
    ticks = int(deg * 4096 / 360.0) + 2048
    return max(0, min(4095, ticks))


def ticks_to_rad(ticks: int, gear_ratio: float) -> float:
    """Konverter ticks tilbake til radianer."""
    deg = (ticks - 2048) * 360.0 / 4096.0
    return np.deg2rad(deg / gear_ratio)


class MyDynamixelController(Node):
    def __init__(self, motor_ids) -> None:
        super().__init__("my_dynamixel_controller")

        self.motor_ids = motor_ids
        self.dxls = CustomDXL(motor_ids)
        self.dxls.open_port()

        # Sett startposisjon (midtpunkt)
        #start_pos = [2047] * len(motor_ids)
        #self.dxls.send_goal(start_pos)

        # Girforhold for motorene
        self.gear_ratios = [1.0, 2.0, 2.0, 1.0, 1.0]

        # Subscriber for ønskede vinkler (radianer)
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/dxl_joint_cmd',
            self.listener_callback,
            10
        )

        # Publisher for faktiske posisjoner
        self.state_pub = self.create_publisher(Float32MultiArray, '/dxl_joint_state', 10)

        # Timer som leser posisjon hvert 0.5 sekund
        self.create_timer(0.5, self.read_positions)

        # Dynamixel SDK handlers
        self.portHandler = self.dxls.portHandler
        self.packetHandler = self.dxls.packetHandler
        self.ADDR_PRESENT_POSITION = 132  # XM430

        self.get_logger().info(f"Controller created for motor IDs {motor_ids} - venter på kommandoer")

    def listener_callback(self, msg: Float32MultiArray):
        expected = len(self.motor_ids)
        if len(msg.data) == expected:
            ticks_list = []
            for i, q_rad in enumerate(msg.data, start=1):
                deg = r2d(q_rad) * self.gear_ratios[i-1]
                ticks_list.append(deg_to_ticks(deg))

            self.get_logger().info(f"Mottatt vinkler (rad): {msg.data}")
            self.get_logger().info(f"Konvertert til ticks: {ticks_list}")

            self.dxls.send_goal(ticks_list)
        else:
            self.get_logger().warn(f"Forventet {expected} verdier, fikk: {msg.data}")

    def read_positions(self):
        rad_list = []
        for i, motor_id in enumerate(self.motor_ids):
            pos, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, motor_id, self.ADDR_PRESENT_POSITION
            )
            if dxl_comm_result == dxl.COMM_SUCCESS and dxl_error == 0:
                rad = ticks_to_rad(pos, self.gear_ratios[i])
                rad_list.append(rad)
            else:
                rad_list.append(0.0)

        msg = Float32MultiArray()
        msg.data = rad_list
        self.state_pub.publish(msg)
        self.get_logger().info(f"Publiserer faktiske posisjoner (rad): {rad_list}")


def main(args=None):
    rclpy.init(args=args)
    motor_ids = [1, 2, 3, 4, 5]
    node = MyDynamixelController(motor_ids)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()