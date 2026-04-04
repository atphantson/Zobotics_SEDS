#!/usr/bin/env python3
import os
import sys
import time
import argparse
import numpy as np
import pinocchio as pin
from pathlib import Path
from pinocchio.robot_wrapper import RobotWrapper

# Importation de tes modules locaux
_here = Path(__file__).resolve().parent
sys.path.insert(0, str(_here))
sys.path.append("..")
from angle_rad import motor_position_to_angle_rad
from scservo_sdk import *

KT              = 8.3     # kg.cm/A
UNIT_TO_AMPS    = 0.0065  # 6.5mA par unité

def nm_to_hls_units(tau_nm):
    # 1. Calcul de la valeur registre
    # Courant (A) = Couple / Kt
    # Unités = Courant / 0.0065
    current_amps = tau_nm / KT
    hls_units = int(current_amps / UNIT_TO_AMPS)

    # Sécurité saturation (le registre 44 accepte jusqu'à 2047)
    if hls_units > 2047: hls_units = 2047

    return hls_units

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=1000000)
    args = p.parse_args()

    # 1. CHARGEMENT DU ROBOT (PINOCCHIO)
    _zobotics = _here / "zobotics"
    urdf_path = str(_zobotics / "robot.urdf")
    try:
        robot = RobotWrapper.BuildFromURDF(urdf_path, package_dirs=[str(_zobotics)])
    except Exception as e:
        print(f"Erreur URDF : {e}")
        return

    model = robot.model
    data = model.createData()

    # 2. INITIALISATION SERVOS
    port_handler = PortHandler(args.port)
    packet_handler = hls(port_handler)

    if not port_handler.openPort():
        print("Erreur : Impossible d'ouvrir le port")
        return
    port_handler.setBaudRate(args.baud)

    # Configuration SyncRead (Adresse 0x38 = 56 DEC, Longueur 4 bytes)
    ids = [1, 2, 3]
    group_read = GroupSyncRead(packet_handler, 0x38, 4)
    for s_id in ids:
        group_read.addParam(s_id)
        packet_handler.write1ByteTxRx(s_id, 33, 2)


    print("--- Démarrage de la compensation de gravité ---")
    print("Appuyez sur Ctrl+C pour arrêter et libérer les servos.")

    try:
        while True:
            # A. Lecture synchronisée des positions
            comm = group_read.txRxPacket()
            if comm != COMM_SUCCESS:
                continue

            q = []
            for s_id in ids:
                # Lecture 4 bytes (Position actuelle)
                raw = group_read.getData(s_id, 0x38, 4)
                # Conversion SDK vers entier signé hôte
                pos_host = packet_handler.scs_tohost(raw, 32)
                # Conversion vers Radian (via ton module angle_rad)
                q.append(motor_position_to_angle_rad(s_id, pos_host))
            
            q_np = np.array(q)

            # B. Calcul Pinocchio (Generalized Gravity)
            # Renvoie le couple statique nécessaire pour contrer le poids (N.m)
            g = pin.computeGeneralizedGravity(model, data, q_np)
            print(f"g: {g}")

            # C. Envoi des couples de compensation
            for i, s_id in enumerate(ids):
                hls_torque_val = nm_to_hls_units(g[i])
                
                # Écriture dans Registre 44 (Target Torque / Courant)
                # Note: Le servo doit être en Mode 0 (Position) ou Mode 2 (Torque)
                print(f"Envoi du couple {hls_torque_val} pour le servo {s_id}")
                packet_handler.write2ByteTxRx(s_id, 44, hls_torque_val)

            # Debug affichage (toutes les 10 itérations)
            # print(f"q: {q_np.round(3)} | g(Nm): {g.round(3)}")
            
            time.sleep(0.2) # Boucle à 50Hz

    except KeyboardInterrupt:
        print("\nArrêt... Désactivation du couple.")
        for s_id in ids:
            packet_handler.write1ByteTxRx(s_id, 40, 0) # Torque OFF
    finally:
        group_read.clearParam()
        port_handler.closePort()

if __name__ == "__main__":
    main()