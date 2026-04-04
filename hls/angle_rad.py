#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Angle (radians) ↔ position moteur HLS.

Plage : [-1.57, +1.57] rad (cap), mappée linéairement sur [1024, 3072].
(-1.57 → 1024, +1.57 → 3072 ; proche de ±π/2 ≈ ±1.5708.)
Une unité de position correspond à 0.087° (info constructeur).
"""

from typing import Tuple

# Bornes angulaires (rad) et positions associées (cf. consigne)
ANGLE_RAD_MIN = -1.57
ANGLE_RAD_MAX = 1.57
MOTOR_POS_MIN = 1024
MOTOR_POS_MAX = 3072

# Résolution nominale (° par unité de position), pour référence
DEG_PER_MOTOR_UNIT = 0.087

# Axes 2 et 3 (même cartes que sync_read_tot / Pinocchio q[1], q[2])
AXIS2_POS_MIN, AXIS2_POS_MAX = -975, 1723
AXIS2_RAD_MIN, AXIS2_RAD_MAX = -0.43, 2.20
AXIS3_POS_MIN, AXIS3_POS_MAX = 81, 3736
AXIS3_RAD_MIN, AXIS3_RAD_MAX = -2.0, 2.36


def _linear_motor_pos_to_angle_rad(pos: float, p_lo: float, p_hi: float, r_lo: float, r_hi: float, clamp: bool = True) -> float:
    """Inverse de la carte affine position encodeur (host) → radians."""
    if clamp:
        pos = max(min(pos, p_hi), p_lo)
    span_p = p_hi - p_lo
    if span_p == 0:
        return r_lo
    t = (pos - p_lo) / span_p
    return r_lo + t * (r_hi - r_lo)


def motor_position_to_angle_rad(servo_id: int, pos_host: float) -> float:
    """
    Position moteur (après scs_tohost) → angle en radians pour l’ID 1, 2 ou 3.
    Joint 1 : même plage que angle_rad_to_motor_position (1024…3072 ↔ -1.57…1.57).
    """
    if servo_id == 1:
        return _linear_motor_pos_to_angle_rad(
            pos_host, MOTOR_POS_MIN, MOTOR_POS_MAX, ANGLE_RAD_MIN, ANGLE_RAD_MAX
        )
    if servo_id == 2:
        return _linear_motor_pos_to_angle_rad(
            pos_host, AXIS2_POS_MIN, AXIS2_POS_MAX, AXIS2_RAD_MIN, AXIS2_RAD_MAX
        )
    if servo_id == 3:
        return _linear_motor_pos_to_angle_rad(
            pos_host, AXIS3_POS_MIN, AXIS3_POS_MAX, AXIS3_RAD_MIN, AXIS3_RAD_MAX
        )
    raise ValueError("servo_id doit être 1, 2 ou 3")


def clamp_angle_rad(angle_rad: float) -> float:
    """Borne l'angle dans [-1.57, 1.57] rad."""
    return max(ANGLE_RAD_MIN, min(ANGLE_RAD_MAX, angle_rad))


def angle_rad_to_motor_position(angle_rad: float) -> int:
    """
    Convertit un angle en radians en position moteur (entier SDK).
    Les angles hors [-1.57, 1.57] sont plafonnés (cap) sur cette plage.
    """
    a = clamp_angle_rad(angle_rad)
    span_rad = ANGLE_RAD_MAX - ANGLE_RAD_MIN
    span_pos = MOTOR_POS_MAX - MOTOR_POS_MIN
    # p = 1024 + (a - (-π/2)) / π * 2048
    pos = MOTOR_POS_MIN + (a - ANGLE_RAD_MIN) / span_rad * span_pos
    return int(round(pos))


def angle_rad_to_motor_position_with_clamp_info(angle_rad: float) -> Tuple[int, bool]:
    """
    Comme angle_rad_to_motor_position, mais indique si l'angle a été plafonné.
    Retourne (position_moteur, était_plafonné).
    """
    before = angle_rad
    a = clamp_angle_rad(angle_rad)
    pos = angle_rad_to_motor_position(a)
    capped = abs(before - a) > 1e-12
    return pos, capped


def move_hls_servo_angle_rad(packet_handler, servo_id, angle_rad, speed=60, acc=50, torque=500):
    """
    Envoie une consigne de position dérivée d'un angle en radians (protocole hls.WritePosEx).

    packet_handler : instance hls(...) du SDK
    servo_id : ID du servo
    angle_rad : angle en rad, plafonné implicitement dans [-1.57, 1.57]

    Retourne le même couple (comm_result, scs_error) que WritePosEx.
    """
    position = angle_rad_to_motor_position(angle_rad)
    return packet_handler.WritePosEx(servo_id, position, speed, acc, torque)
