#!/usr/bin/env python3
import time
import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from angle_rad import motor_position_to_angle_rad
sys.path.append("..")

from scservo_sdk import *

KT = 8.3
UNIT_TO_AMPS = 0.0065

ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_TORQUE = 44
ADDR_PRESENT_POSITION = HLS_PRESENT_POSITION_L

IDS = (1, 2, 3)


def to_little_endian_2b(val: int):
    if val < 0:
        val = 32767 - val
    return [val & 0xFF, (val >> 8) & 0xFF]


def read_joint_positions_rad(packet, port) -> np.ndarray:
    group = GroupSyncRead(packet, ADDR_PRESENT_POSITION, 4)
    for i in IDS:
        group.addParam(i)

    group.txRxPacket()

    q = []
    for i in IDS:
        raw = group.getData(i, ADDR_PRESENT_POSITION, 2)
        pos = packet.scs_tohost(raw, 15)
        q.append(motor_position_to_angle_rad(i, pos))

    group.clearParam()
    return np.array(q, dtype=np.float64)


def set_torque_mode(packet):
    for i in IDS:
        packet.write1ByteTxRx(i, ADDR_MODE, 2)          # mode courant
        packet.write1ByteTxRx(i, ADDR_TORQUE_ENABLE, 1) # torque ON


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=1000000)
    args = p.parse_args()

    # Pinocchio
    here = Path(__file__).resolve().parent
    zob = here / "zobotics"
    robot = RobotWrapper.BuildFromURDF(str(zob / "robot.urdf"),
                                       package_dirs=[str(zob)])
    model = robot.model
    data = model.createData()

    # Servo init
    port = PortHandler(args.port)
    packet = hls(port)

    port.openPort()
    port.setBaudRate(args.baud)

    set_torque_mode(packet)

    group_write = GroupSyncWrite(packet, ADDR_GOAL_TORQUE, 2)

    print("Gravity compensation running...\n")

    while True:
        q = read_joint_positions_rad(packet, port)

        g = pin.computeGeneralizedGravity(model, data, q)  # Nm
        g = g * 10.197  # -> kg.cm

        torque_A = -g / KT
        hls_units = torque_A / UNIT_TO_AMPS

        group_write.clearParam()

        for scs_id, val in zip(IDS, hls_units):
            val_i = int(np.clip(val, -2047, 2047))
            group_write.addParam(scs_id, to_little_endian_2b(val_i))

        group_write.txPacket()

        print("HLS goal torque:", [int(v) for v in hls_units])
        time.sleep(0.02)


if __name__ == "__main__":
    main()