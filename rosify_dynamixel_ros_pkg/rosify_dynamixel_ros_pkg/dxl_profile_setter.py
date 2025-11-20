#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import dynamixel_sdk as dxl

class MotorProfileSetter(Node):
    def __init__(self, dxl_ids=[1, 2, 3, 4, 5], port='/dev/ttyUSB0', baudrate=57600):
        super().__init__('motor_profile_setter')

        # Dynamixel Control Table addresses
        self.ADDR_TORQUE_ENABLE     = 64
        self.ADDR_PROFILE_ACCEL     = 108
        self.ADDR_PROFILE_VELOCITY  = 112
        self.ADDR_OPERATING_MODE    = 11   # <-- Operating Mode

        self.PROTOCOL_VERSION = 2.0
        self.port = port
        self.baudrate = baudrate
        self.dxl_ids = dxl_ids

        # Hvilken modus hver motor skal ha
        # 3 = Position, 4 = Extended Position
        self.modes = {
            1: 3,
            2: 4,
            3: 4,
            4: 3,
            5: 3,
        }

        # Velocity og Acceleration profiler
        self.profiles = {
            1: (30, 20),
            2: (10, 20),
            3: (10, 20),
            4: (10, 20),
            5: (10, 20),
        }

        # Init port and packet handler
        self.portHandler = dxl.PortHandler(self.port)
        self.packetHandler = dxl.PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open port")
            quit()
        if not self.portHandler.setBaudRate(self.baudrate):
            self.get_logger().error("Failed to set baudrate")
            quit()

        self.get_logger().info("Port opened and baudrate set")

        # Sett operating mode først
        self.set_modes()

        # Sett profiler for motorene
        self.set_profiles()

    def set_modes(self):
        for motor_id in self.dxl_ids:
            mode = self.modes.get(motor_id, 3)  # default til Position Mode
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, motor_id, self.ADDR_OPERATING_MODE, mode
            )
            if dxl_comm_result != dxl.COMM_SUCCESS:
                self.get_logger().error(f"[ID:{motor_id}] Mode write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"[ID:{motor_id}] Mode error: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                mode_name = "Position" if mode == 3 else "Extended Position"
                self.get_logger().info(f"[ID:{motor_id}] Operating Mode set to {mode_name} ({mode})")

    def set_profiles(self):
        for motor_id in self.dxl_ids:
            velocity, accel = self.profiles.get(motor_id, (200, 20))

            # Profile Acceleration
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, motor_id, self.ADDR_PROFILE_ACCEL, accel
            )
            if dxl_comm_result != dxl.COMM_SUCCESS:
                self.get_logger().error(f"[ID:{motor_id}] Accel write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"[ID:{motor_id}] Accel error: {self.packetHandler.getRxPacketError(dxl_error)}")

            # Profile Velocity
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, motor_id, self.ADDR_PROFILE_VELOCITY, velocity
            )
            if dxl_comm_result != dxl.COMM_SUCCESS:
                self.get_logger().error(f"[ID:{motor_id}] Vel write failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"[ID:{motor_id}] Vel error: {self.packetHandler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"[ID:{motor_id}] Profile set: Velocity={velocity}, Accel={accel}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorProfileSetter(dxl_ids=[1,2,3,4,5])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()