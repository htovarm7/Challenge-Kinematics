from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.167')
arm.motion_enable(enable=True)
arm.set_mode(2)
arm.set_state(0)
print(f"Modo: {arm.mode}, Estado: {arm.state}")
print("✅ Maestro en modo teach — muévelo a mano ahora")
input("Presiona Enter cuando termines de moverlo...")
arm.set_mode(0)
arm.set_state(0)
print("✅ Maestro regresó a modo normal")
