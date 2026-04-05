#!/usr/bin/env python3
"""High-level FTServo/HLS communication utilities for control loops.

This module centralizes low-level servo communication so higher-level control
code can focus on algorithmic logic.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence

import numpy as np

import sys
sys.path.append("/home/valentin/ZOBotics/FTServo_Python")

from angle_rad import motor_position_to_angle_rad
from scservo_sdk import *


# --- Control and feedback scaling ---
KT_KGCM_PER_AMP = 8.3
UNIT_TO_AMPS = 0.0065
NM_TO_KGCM = 10.197162129779

SPEED_UNIT_RPM = 0.732
RPM_TO_RAD_S = 2.0 * np.pi / 60.0


# --- Registers ---
ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_TORQUE = 44
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_CURRENT = 69

MODE_POSITION = 0
MODE_WHEEL = 1
MODE_TORQUE = 2


def to_little_endian_2b(val: int) -> List[int]:
    """Match compensation.py torque encoding."""
    if val < 0:
        val = 32768 - val
    return [val & 0xFF, (val >> 8) & 0xFF]


@dataclass(frozen=True)
class JointStateSample:
    timestamp: float
    ids: Sequence[int]
    raw_position: np.ndarray
    raw_speed: np.ndarray
    raw_current: np.ndarray
    position_rad: np.ndarray
    velocity_rad_s: np.ndarray
    current_a: np.ndarray


class ServoInterfaceError(RuntimeError):
    """Raised when servo communication fails."""


class HlsServoInterface:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 1000000,
        ids: Iterable[int] = (1, 2, 3),
    ) -> None:
        self.port_name = port
        self.baud = int(baud)
        self.ids: List[int] = list(ids)
        if not self.ids:
            raise ValueError("At least one servo id is required")

        self._port = PortHandler(self.port_name)
        self._packet = hls(self._port)

        self._is_open = False
        self._mode = None

        self._sync_state = GroupSyncRead(self._packet, ADDR_PRESENT_POSITION, 6)
        self._sync_current = GroupSyncRead(self._packet, ADDR_PRESENT_CURRENT, 2)
        self._sync_torque_write = GroupSyncWrite(self._packet, ADDR_GOAL_TORQUE, 2)

    @property
    def packet(self):
        return self._packet

    def open(self) -> None:
        if self._is_open:
            return

        if not self._port.openPort():
            raise ServoInterfaceError(f"Failed to open serial port {self.port_name}")
        if not self._port.setBaudRate(self.baud):
            self._port.closePort()
            raise ServoInterfaceError(f"Failed to set baud rate {self.baud}")

        self._is_open = True

    def close(self) -> None:
        if not self._is_open:
            return
        try:
            self._sync_state.clearParam()
            self._sync_current.clearParam()
            self._sync_torque_write.clearParam()
        finally:
            self._port.closePort()
            self._is_open = False

    def set_torque_enable(self, enabled: bool) -> None:
        value = 1 if enabled else 0
        for servo_id in self.ids:
            print("servo_id", servo_id, "enabled", value)
            comm, err = self._packet.write1ByteTxRx(servo_id, ADDR_TORQUE_ENABLE, value)
            self._raise_if_error(comm, err, servo_id, "set_torque_enable")

    def set_mode_torque(self) -> None:
        # Match compensation.py behavior:
        # for each servo, set mode to torque then enable torque immediately.
        for servo_id in self.ids:
            comm, err = self._packet.write1ByteTxRx(servo_id, ADDR_MODE, MODE_TORQUE)
            self._raise_if_error(comm, err, servo_id, "set_mode_torque")
            comm, err = self._packet.write1ByteTxRx(servo_id, ADDR_TORQUE_ENABLE, 1)
            self._raise_if_error(comm, err, servo_id, "set_mode_torque")
        self._mode = MODE_TORQUE

    def set_mode_position(self) -> None:
        self._set_mode(MODE_POSITION)

    def set_mode_wheel(self) -> None:
        self._set_mode(MODE_WHEEL)

    def _set_mode(self, mode: int) -> None:
        if self._mode == mode:
            return
        for servo_id in self.ids:
            comm, err = self._packet.write1ByteTxRx(servo_id, ADDR_MODE, int(mode))
            self._raise_if_error(comm, err, servo_id, "set_mode")
        self._mode = mode

    def read_joint_state(self) -> JointStateSample:
        self._sync_state.clearParam()
        self._sync_current.clearParam()
        for servo_id in self.ids:
            if not self._sync_state.addParam(servo_id):
                raise ServoInterfaceError(f"SyncRead addParam failed for servo {servo_id}")
            if not self._sync_current.addParam(servo_id):
                raise ServoInterfaceError(f"Current SyncRead addParam failed for servo {servo_id}")

        comm_state = self._sync_state.txRxPacket()
        if comm_state != COMM_SUCCESS:
            raise ServoInterfaceError(self._packet.getTxRxResult(comm_state))

        comm_curr = self._sync_current.txRxPacket()
        if comm_curr != COMM_SUCCESS:
            raise ServoInterfaceError(self._packet.getTxRxResult(comm_curr))

        raw_pos = np.zeros(len(self.ids), dtype=np.int32)
        raw_speed = np.zeros(len(self.ids), dtype=np.int32)
        raw_current = np.zeros(len(self.ids), dtype=np.int32)
        q = np.zeros(len(self.ids), dtype=np.float64)
        dq = np.zeros(len(self.ids), dtype=np.float64)
        ia = np.zeros(len(self.ids), dtype=np.float64)

        for i, servo_id in enumerate(self.ids):
            ok_state, err_state = self._sync_state.isAvailable(servo_id, ADDR_PRESENT_POSITION, 6)
            if not ok_state:
                raise ServoInterfaceError(f"No position/speed data for servo {servo_id}")
            if err_state:
                raise ServoInterfaceError(self._packet.getRxPacketError(err_state))

            ok_curr, err_curr = self._sync_current.isAvailable(servo_id, ADDR_PRESENT_CURRENT, 2)
            if not ok_curr:
                raise ServoInterfaceError(f"No current data for servo {servo_id}")
            if err_curr:
                raise ServoInterfaceError(self._packet.getRxPacketError(err_curr))

            pos_raw = int(self._sync_state.getData(servo_id, ADDR_PRESENT_POSITION, 2))
            speed_raw = int(self._sync_state.getData(servo_id, ADDR_PRESENT_SPEED, 2))
            curr_raw = int(self._sync_current.getData(servo_id, ADDR_PRESENT_CURRENT, 2))

            pos_host = int(self._packet.scs_tohost(pos_raw, 15))
            speed_host = int(self._packet.scs_tohost(speed_raw, 15))
            curr_host = int(self._packet.scs_tohost(curr_raw, 15))

            raw_pos[i] = pos_host
            raw_speed[i] = speed_host
            raw_current[i] = curr_host

            q[i] = motor_position_to_angle_rad(servo_id, pos_host)
            dq[i] = speed_host * SPEED_UNIT_RPM * RPM_TO_RAD_S
            ia[i] = curr_host * UNIT_TO_AMPS

        return JointStateSample(
            timestamp=time.time(),
            ids=tuple(self.ids),
            raw_position=raw_pos,
            raw_speed=raw_speed,
            raw_current=raw_current,
            position_rad=q,
            velocity_rad_s=dq,
            current_a=ia,
        )

    def send_torque_units(self, torque_units: Sequence[float], clip: int = 2047) -> None:
        if len(torque_units) != len(self.ids):
            raise ValueError("torque_units must have the same length as ids")

        # Use a fresh sync writer exactly like compensation.py usage.
        group_write = GroupSyncWrite(self._packet, ADDR_GOAL_TORQUE, 2)
        group_write.clearParam()

        for servo_id, command in zip(self.ids, torque_units):
            value = int(np.clip(np.rint(command), -clip, clip))
            if not group_write.addParam(servo_id, to_little_endian_2b(value)):
                raise ServoInterfaceError(f"SyncWrite addParam failed for servo {servo_id}")

        comm = group_write.txPacket()
        group_write.clearParam()
        if comm != COMM_SUCCESS:
           raise ServoInterfaceError(self._packet.getTxRxResult(comm))

    def send_torque_amps(self, torque_a: Sequence[float], clip: int = 2047) -> None:
        torque_units = np.asarray(torque_a, dtype=np.float64) / UNIT_TO_AMPS
        self.send_torque_units(torque_units, clip=clip)

    def send_torque_nm(self, torque_nm: Sequence[float], clip: int = 2047) -> None:
        torque_a = (np.asarray(torque_nm, dtype=np.float64) * NM_TO_KGCM) / KT_KGCM_PER_AMP
        self.send_torque_amps(torque_a, clip=clip)

    def ping(self) -> Dict[int, bool]:
        results: Dict[int, bool] = {}
        for servo_id in self.ids:
            _, comm, err = self._packet.ping(servo_id)
            results[servo_id] = (comm == COMM_SUCCESS and err == 0)
        return results

    def _raise_if_error(self, comm: int, err: int, servo_id: int, op_name: str) -> None:
        if comm != COMM_SUCCESS:
            raise ServoInterfaceError(f"{op_name} failed for servo {servo_id}: {self._packet.getTxRxResult(comm)}")
        if err != 0:
            raise ServoInterfaceError(f"{op_name} failed for servo {servo_id}: {self._packet.getRxPacketError(err)}")

    def disable_torque(self) -> None:
        self._packet.write1ByteTxRx(1, ADDR_TORQUE_ENABLE, 0)
        self._packet.write1ByteTxRx(2, ADDR_TORQUE_ENABLE, 0)
        self._packet.write1ByteTxRx(3, ADDR_TORQUE_ENABLE, 0)
        self._port.closePort()