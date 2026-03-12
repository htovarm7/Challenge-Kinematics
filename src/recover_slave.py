#!/usr/bin/env python3
"""Recupera el esclavo después de error C31 o cualquier falla."""
from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('192.168.1.175')
print(f"Estado actual: mode={arm.mode}, state={arm.state}, error={arm.error_code}")

arm.clean_error()
arm.clean_warn()
time.sleep(0.5)

arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.5)

print(f"Después de reset: mode={arm.mode}, state={arm.state}, error={arm.error_code}")

# Mover a HOME suavemente
HOME_DEG = [1.1043, 19.0107, 74.6311, -5.9974, 58.5693, -3.0649]
print("Moviendo a HOME...")
arm.set_servo_angle(angle=HOME_DEG, speed=20, wait=True)
print("En HOME")

# Restaurar modo 1 para ros2_control
arm.set_mode(1)
arm.set_state(0)
time.sleep(0.3)
print(f"Listo para ros2_control: mode={arm.mode}, state={arm.state}")
arm.disconnect()
