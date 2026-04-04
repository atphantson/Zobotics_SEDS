#!/usr/bin/env python3
import time
import sys
sys.path.append("..")
from scservo_sdk import *

PORT = '/dev/ttyUSB0'
ID = 2
BAUD = 1000000

# Adresses SRAM
ADDR_TORQUE_ENABLE = 40      # 0x28
ADDR_CURRENT = 69            # 0x45

KT_KGCM_PER_A = 8.3
CURRENT_LSB_A = 0.0065

# Init
portHandler = PortHandler(PORT)
packetHandler = hls(portHandler)

portHandler.openPort()
portHandler.setBaudRate(BAUD)

# Enable torque
packetHandler.write1ByteTxRx(ID, ADDR_TORQUE_ENABLE, 1)

print("Torque enabled. Reading current...\n")

def to_signed(val):
    if val > 32767:
        val = 0 -(val - 32768)
    return val

try:
    while True:
        raw, result, err = packetHandler.read2ByteTxRx(ID, ADDR_CURRENT)
        if result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(result))
            continue

        raw = to_signed(raw)

        current_A = raw * CURRENT_LSB_A
        torque_kgcm = current_A * KT_KGCM_PER_A
        torque_Nm = torque_kgcm * 0.0980665

        print(f"Raw:{raw:6d} | I={current_A:6.3f} A | "
              f"T={torque_kgcm:6.2f} kg·cm | {torque_Nm:5.3f} N·m")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")

# Disable torque on exit
packetHandler.write1ByteTxRx(ID, ADDR_TORQUE_ENABLE, 0)
portHandler.closePort()