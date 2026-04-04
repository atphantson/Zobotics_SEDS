#!/usr/bin/env python3
import sys
import time
sys.path.append("..")
from scservo_sdk import *

# --- CONFIGURATION ---
PORT            = '/dev/ttyUSB0'
BAUD            = 1000000
SERVO_ID        = 2
TARGET_TORQUE_KG = 15.0    # Couple cible en kg.cm
KT              = 8.3     # kg.cm/A
UNIT_TO_AMPS    = 0.0065  # 6.5mA par unité

# 1. Calcul de la valeur registre
# Courant (A) = Couple / Kt
# Unités = Courant / 0.0065
current_amps = TARGET_TORQUE_KG / KT
hls_units = int(current_amps / UNIT_TO_AMPS)

# Sécurité saturation (le registre 44 accepte jusqu'à 2047)
if hls_units > 2047: hls_units = 2047

print(f"Calcul : {TARGET_TORQUE_KG}kg.cm -> {hls_units} unités registre")

# 2. Initialisation SDK
portHandler = PortHandler(PORT)
packetHandler = hls(portHandler)

if not portHandler.openPort():
    print("Erreur port")
    quit()
portHandler.setBaudRate(BAUD)

try:
    print(f"--- Activation Mode Couple sur ID {SERVO_ID} ---")
    
    # Étape A : Mettre le Torque Switch à OFF pour changer de mode en sécurité
    packetHandler.write1ByteTxRx(SERVO_ID, 40, 0)
    
    # Étape B : Changer le Running Mode (Registre 33) -> 2 (Constant Current)
    # On déverrouille l'EPROM d'abord
    packetHandler.write1ByteTxRx(SERVO_ID, 55, 0) 
    packetHandler.write1ByteTxRx(SERVO_ID, 33, 2)
    packetHandler.write1ByteTxRx(SERVO_ID, 55, 1)
    
    # Étape C : Activer le couple (Torque Switch -> 1)
    packetHandler.write1ByteTxRx(SERVO_ID, 40, 1)

    print(f"Envoi du couple constant : {TARGET_TORQUE_KG} kg.cm")
    
    while True:
        # Étape D : Écrire la valeur dans Target Torque (Registre 44)
        # Pour inverser le sens, ajoute 32768 à hls_units
        print(f"Envoi du couple {hls_units} pour le servo {SERVO_ID}")
        packetHandler.write2ByteTxRx(SERVO_ID, 44, hls_units)
        
        # On peut lire la charge actuelle pour vérifier (Registre 60)
        load, res, err = packetHandler.read2ByteTxRx(SERVO_ID, 60)
        print(f"Couple envoyé. Charge actuelle lue : {load}", end='\r')
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nArrêt : Passage en roue libre...")
    packetHandler.write1ByteTxRx(SERVO_ID, 40, 0)
    # Optionnel : remettre en mode position (Mode 0) avant de quitter
    packetHandler.write1ByteTxRx(SERVO_ID, 55, 0)
    packetHandler.write1ByteTxRx(SERVO_ID, 33, 0)
    packetHandler.write1ByteTxRx(SERVO_ID, 55, 1)

finally:
    portHandler.closePort()