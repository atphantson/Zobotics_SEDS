#!/usr/bin/env python
import sys
import os
import time

sys.path.append("..")
from scservo_sdk import * # 1. Initialisation du Port
portHandler = PortHandler('/dev/ttyUSB0')
packetHandler = hls(portHandler) # On utilise la classe hls comme dans ton exemple

if portHandler.openPort():
    print("Port ouvert avec succès")
else:
    print("Échec du port")
    quit()

if portHandler.setBaudRate(1000000):
    print("Baudrate OK (1Mbps)")
else:
    print("Échec Baudrate")
    quit()

# --- CONFIGURATION MODE NORMAL ---

SERVO_ID = 1

def configure_servo_normal(id):
    print(f"Configuration du Servo {id}...")
    
    # Étape A : Déverrouiller l'EPROM (Registre 55 -> 0)
    # write1ByteTxRx est la méthode de base pour un registre simple
    packetHandler.write1ByteTxRx(id, 55, 0)
    
    # Étape B : Fixer les limites d'angle (Reg 9 et 11)
    # On met 0 et 4095 pour brider le servo sur un seul tour
    #packetHandler.write2ByteTxRx(id, 9, -89)
    #packetHandler.write2ByteTxRx(id, 11, 2635)
    
    # Étape C : Désactiver le cumul de tours (Registre 18 -> 0)
    # Cela évite de lire des valeurs > 4095 (ton problème de "6000")
    packetHandler.write1ByteTxRx(id, 18, 0)
    
    # Étape D : Mode Position (Registre 33 -> 0)
    packetHandler.write1ByteTxRx(id, 33, 0)
    
    # Étape E : Re-verrouiller l'EPROM (Registre 55 -> 1)
    packetHandler.write1ByteTxRx(id, 55, 1)
    print("Configuration terminée et sauvegardée.")

# Exécution de la config
configure_servo_normal(SERVO_ID)

# --- TEST DE MOUVEMENT ---

# try:
#     print("Test du mouvement 0 -> 2048 -> 0")
#     while True:
#         # On utilise ta fonction WritePosEx
#         # ID, Position (0-4095), Vitesse, Accélération, Torque
#         print("Aller à 2048 (90°)")
#         packetHandler.WritePosEx(SERVO_ID, 0, 100, 50, 1000)
#         time.sleep(2)
        
#         print("Retour à 0")
#         packetHandler.WritePosEx(SERVO_ID, 4096, 100, 50, 1000)
#         time.sleep(2)

# except KeyboardInterrupt:
#     packetHandler.write1ByteTxRx(2, 40, 0)
#     print("Arrêt du script.")

# Fermeture
portHandler.closePort()