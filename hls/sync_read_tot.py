#!/usr/bin/env python
#
# Sync read des positions HLS (ID 1, 2, 3) et affichage en radians par axe.
# Résolution nominale encodeur : 0.087° / pas (info constructeur).
#

import argparse
import sys
import time

sys.path.append("..")
from scservo_sdk import *
from angle_rad import (
    ANGLE_RAD_MAX,
    ANGLE_RAD_MIN,
    AXIS2_POS_MAX,
    AXIS2_POS_MIN,
    AXIS2_RAD_MAX,
    AXIS2_RAD_MIN,
    AXIS3_POS_MAX,
    AXIS3_POS_MIN,
    AXIS3_RAD_MAX,
    AXIS3_RAD_MIN,
    MOTOR_POS_MAX,
    MOTOR_POS_MIN,
    motor_position_to_angle_rad,
)


def main():
    p = argparse.ArgumentParser(
        description="Sync read positions ID 1–3 et affichage en radians."
    )
    p.add_argument("--port", default="/dev/ttyUSB0", help="Port série (défaut: /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=1000000, help="Baudrate (défaut: 1000000)")
    p.add_argument(
        "--period",
        type=float,
        default=1.0,
        help="Intervalle entre lectures en secondes (défaut: 1.0)",
    )
    args = p.parse_args()

    port_handler = PortHandler(args.port)
    packet_handler = hls(port_handler)

    if not port_handler.openPort():
        print("Échec ouverture port")
        return 1
    if not port_handler.setBaudRate(args.baud):
        print("Échec baudrate")
        port_handler.closePort()
        return 1

    group_sync_read = GroupSyncRead(packet_handler, HLS_PRESENT_POSITION_L, 4)
    ids = (1, 2, 3)

    print(
        "Lecture groupée positions (reg %d), ID 1–3. Ctrl+C pour quitter."
        % HLS_PRESENT_POSITION_L
    )
    print(
        "Cartes rad: id1 [%d..%d]→[%.2f..%.2f], id2 [%d..%d]→[%.2f..%.2f], id3 [%d..%d]→[%.2f..%.2f]"
        % (
            MOTOR_POS_MIN,
            MOTOR_POS_MAX,
            ANGLE_RAD_MIN,
            ANGLE_RAD_MAX,
            AXIS2_POS_MIN,
            AXIS2_POS_MAX,
            AXIS2_RAD_MIN,
            AXIS2_RAD_MAX,
            AXIS3_POS_MIN,
            AXIS3_POS_MAX,
            AXIS3_RAD_MIN,
            AXIS3_RAD_MAX,
        )
    )

    try:
        while True:
            for scs_id in ids:
                if not group_sync_read.addParam(scs_id):
                    print("[ID:%03d] addParam échoué" % scs_id)

            comm_result = group_sync_read.txRxPacket()
            if comm_result != COMM_SUCCESS:
                print("%s" % packet_handler.getTxRxResult(comm_result))

            for scs_id in ids:
                ok, scs_error = group_sync_read.isAvailable(
                    scs_id, HLS_PRESENT_POSITION_L, 4
                )
                if not ok:
                    print("[ID:%03d] données indisponibles" % scs_id)
                    continue

                raw_pos = group_sync_read.getData(scs_id, HLS_PRESENT_POSITION_L, 2)
                raw_spd = group_sync_read.getData(scs_id, HLS_PRESENT_SPEED_L, 2)
                pos_host = packet_handler.scs_tohost(raw_pos, 15)
                spd_host = packet_handler.scs_tohost(raw_spd, 15)
                rad = motor_position_to_angle_rad(scs_id, pos_host)

                print(
                    "[ID:%03d] pos_brute=%d pos_host=%d  %.4f rad  (vit. %d)"
                    % (scs_id, raw_pos, pos_host, rad, spd_host)
                )
                if scs_error != 0:
                    print("%s" % packet_handler.getRxPacketError(scs_error))

            group_sync_read.clearParam()
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\nArrêt.")
    finally:
        port_handler.closePort()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
