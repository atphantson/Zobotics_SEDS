#!/usr/bin/env python3
"""Simple ROS2 node: send constant low wheel speed to all FTServo joints."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from servo_interface import HlsServoInterface, ServoInterfaceError


class ConstantWheelSpeedNode(Node):
    def __init__(self):
        super().__init__("constant_wheel_speed_node")

        # --- Params ---
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("servo_ids", [1, 2, 3])
        self.declare_parameter("speed_rad_s", 1.7)  # petite vitesse

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        ids = self.get_parameter("servo_ids").value
        self.speed_rad_s = float(self.get_parameter("speed_rad_s").value)

        # --- Servo interface ---
        self.servos = HlsServoInterface(port=port, baud=baud, ids=ids)

        try:
            self.servos.open()
            self.get_logger().info("Serial port opened")

            self.servos.set_mode_wheel()
            self.get_logger().info("Wheel mode set")

            self.servos.set_torque_enable(True)
            self.get_logger().info("Torque enabled")

        except ServoInterfaceError as e:
            self.get_logger().error(f"Servo init failed: {e}")
            raise

        # --- Timer loop ---
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Constant wheel speed node ready")

    def update(self):
        try:
            n = len(self.servos.ids)

            # même vitesse sur tous les axes
            speed_cmd = np.ones(n) * self.speed_rad_s

            self.servos.send_speed_rad_s(speed_cmd)

        except ServoInterfaceError as e:
            self.get_logger().error(f"Servo error: {e}")

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            self.get_logger().error(traceback.format_exc())


def main():
    rclpy.init()
    node = ConstantWheelSpeedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.servos.disable_torque()
        node.servos.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()