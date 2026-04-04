#!/usr/bin/env python
#
# Move HLS servo ID 1 to pose 0 (Target Position = 0, control table reg 42).
#

import argparse
import sys
import time

sys.path.append("..")
from scservo_sdk import *


def wait_until_stopped(packet_handler, servo_id, timeout_s=30.0, poll_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        moving, comm_result, scs_error = packet_handler.ReadMoving(servo_id)
        if comm_result != COMM_SUCCESS:
            print("%s" % packet_handler.getTxRxResult(comm_result))
            return False
        if scs_error != 0:
            print("%s" % packet_handler.getRxPacketError(scs_error))
            return False
        if moving == 0:
            return True
        time.sleep(poll_s)
    print("Timeout waiting for servo %d to stop." % servo_id)
    return False


def main():
    p = argparse.ArgumentParser(description="Move HLS servo to pose 0 (target position 0).")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial device (default: /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=1000000, help="Baud rate (default: 1000000)")
    p.add_argument("--id", type=int, default=1, dest="servo_id", help="Servo ID (default: 1)")
    p.add_argument("--speed", type=int, default=60, help="Speed parameter (same units as write.py, default: 60)")
    p.add_argument("--acc", type=int, default=50, help="Acceleration parameter (default: 50)")
    p.add_argument("--torque", type=int, default=500, help="Torque limit (default: 500)")
    p.add_argument("--no-wait", action="store_true", help="Do not wait until motion finishes")
    args = p.parse_args()

    # Pose 0 = Target Position 0 (see control_table.csv: register 42, Target Position)
    goal_position = 2048

    port_handler = PortHandler(args.port)
    packet_handler = hls(port_handler)

    if not port_handler.openPort():
        print("Failed to open the port")
        return 1
    print("Port opened")

    if not port_handler.setBaudRate(args.baud):
        print("Failed to set baud rate")
        port_handler.closePort()
        return 1
    print("Baud rate set to %d" % args.baud)

    comm_result, scs_error = packet_handler.WritePosEx(
        args.servo_id, goal_position, args.speed, args.acc, args.torque
    )
    if comm_result != COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(comm_result))
        port_handler.closePort()
        return 1
    if scs_error != 0:
        print("%s" % packet_handler.getRxPacketError(scs_error))
        port_handler.closePort()
        return 1

    print("Sent goal position %d to servo ID %d" % (goal_position, args.servo_id))

    if not args.no_wait:
        if wait_until_stopped(packet_handler, args.servo_id):
            pos, cr, er = packet_handler.ReadPos(args.servo_id)
            if cr == COMM_SUCCESS and er == 0:
                print("Current position (host): %d" % pos)

    port_handler.closePort()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
