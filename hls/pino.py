"""
Pinocchio + positions articulaires lues sur les servos HLS (ID 1–3), converties en rad via angle_rad.
Ordre Pinocchio : q[0]=base_rotZ (ID1), q[1]=servo2shaft2 (ID2), q[2]=servo2shaft3 (ID3).
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

_here = Path(__file__).resolve().parent
sys.path.insert(0, str(_here))
sys.path.append(str(_here.parent))

from angle_rad import motor_position_to_angle_rad
from scservo_sdk import *

KT              = 8.3     # kg.cm/A
UNIT_TO_AMPS    = 0.0065  # 6.5mA par unité

def read_joint_positions_rad(port: str, baud: int) -> np.ndarray:
    """Sync-read positions HLS 1–3, retourne q.shape == (3,) en radians."""
    port_handler = PortHandler(port)
    packet_handler = hls(port_handler)
    if not port_handler.openPort():
        raise RuntimeError("Impossible d'ouvrir le port série : %s" % port)
    if not port_handler.setBaudRate(baud):
        port_handler.closePort()
        raise RuntimeError("Impossible de régler le baudrate : %d" % baud)

    group_sync_read = GroupSyncRead(packet_handler, HLS_PRESENT_POSITION_L, 4)
    ids = (1, 2, 3)
    try:
        for scs_id in ids:
            if not group_sync_read.addParam(scs_id):
                raise RuntimeError("groupSyncRead addParam failed for ID %d" % scs_id)

        comm_result = group_sync_read.txRxPacket()
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(packet_handler.getTxRxResult(comm_result))

        q_list = []
        for scs_id in ids:
            ok, scs_error = group_sync_read.isAvailable(scs_id, HLS_PRESENT_POSITION_L, 4)
            if not ok:
                raise RuntimeError("Pas de données sync-read pour le servo ID %d" % scs_id)
            raw_pos = group_sync_read.getData(scs_id, HLS_PRESENT_POSITION_L, 2)
            pos_host = packet_handler.scs_tohost(raw_pos, 15)
            q_list.append(motor_position_to_angle_rad(scs_id, pos_host))
            if scs_error != 0:
                raise RuntimeError(packet_handler.getRxPacketError(scs_error))
    finally:
        group_sync_read.clearParam()
        port_handler.closePort()

    return np.array(q_list, dtype=np.float64)


def main():
    p = argparse.ArgumentParser(description="Gravité généralisée Pinocchio avec q = positions réelles (rad).")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Port série (défaut: /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=1000000, help="Baudrate")
    p.add_argument(
        "--mock",
        action="store_true",
        help="Ne pas lire les servos ; q = [0,0,0] (test sans matériel)",
    )
    args = p.parse_args()

    _zobotics = _here / "zobotics"
    urdf_path = str(_zobotics / "robot.urdf")
    robot = RobotWrapper.BuildFromURDF(urdf_path, package_dirs=[str(_zobotics)])

    model = robot.model
    data = model.createData()
    while True:
        q = read_joint_positions_rad(args.port, args.baud)
        #print("q =", q)
        g = pin.computeGeneralizedGravity(model, data, q)
        # convert g from Nm to kg.cm
        g = g * 10.197
        #print("g =", g)
        torque = - g / KT
        hls_units = torque / UNIT_TO_AMPS
        print("hls_units =", hls_units)
        time.sleep(1)


if __name__ == "__main__":
    main()
