# haptic_teleop

Paquete ROS 2 de teleoperación bilateral háptica para **xArm Lite 6**.  
**TE3001B – Kinematics Challenge | ITESM Robótica**

---

## Hardware

| Robot | IP | Rol | Namespace |
|---|---|---|---|
| xArm Lite 6 | 192.168.1.175 | **Esclavo** – ejecuta movimientos | `/` (global) |
| xArm Lite 6 | 192.168.1.167 | **Maestro** – operado por humano | `/master` |
| ESP32 + micro-ROS | USB `/dev/ttyUSB0` | Sensor de fuerza FSR | — |

El sensor ESP32 publica lecturas ADC en `/force_sensor` (Int32). Valor en reposo ≈ 4095; contacto detectado cuando baja de 3900.

---

## Estructura del paquete

```
haptic_teleop/
├── haptic_teleop/
│   ├── __init__.py
│   ├── bilateral_teleop.py     ← Nodo principal (FSM + haptic por esfuerzo)
│   ├── force_stop_node.py      ← Capa de seguridad independiente
│   └── master_teleop.py        ← Nodo del robot maestro
├── launch/
│   └── bilateral_teleop.launch.py
├── config/
│   └── teleop_params.yaml
├── resource/
│   └── haptic_teleop
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Instalación

```bash
cd ~/Desktop/Challenge-Kinematics
colcon build --symlink-install --packages-select haptic_teleop
source install/setup.bash
```

Verifica que los ejecutables están disponibles:

```bash
ros2 pkg executables haptic_teleop
# haptic_teleop bilateral_teleop
# haptic_teleop force_stop_node
# haptic_teleop master_teleop
```

---

## Secuencia de arranque (6 terminales)

Abre cada terminal en orden y espera la confirmación antes de pasar al siguiente.

### Terminal 1 — micro-ROS (sensor de fuerza ESP32)
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
✅ Confirmar: el ESP32 se conecta y aparecen logs de sesión activa.

---

### Terminal 2 — MoveIt completo del esclavo
```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.175 \
  add_realsense_d435i:=false
```
✅ Confirmar: `You can start planning now!`

---

### Terminal 3 — Driver del robot maestro (namespace /master)
```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 run xarm_api xarm_driver_node \
  --ros-args \
  -r __ns:=/master \
  -p robot_ip:=192.168.1.167 \
  -p dof:=6 \
  -p report_type:=dev
```
✅ Confirmar: `[TCP STATUS] CONTROL: 1`

> **`report_type:=dev` es obligatorio.** Con `normal` solo se obtienen ~5 Hz y la calibración falla.

---

### Terminal 4 — Relay de topics del maestro
```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 run topic_tools relay \
  /master/xarm/joint_states \
  /master/joint_states
```
✅ Confirmar: sin errores, terminal en silencio.

---

### Terminal 5 — MoveIt Servo
```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.175
```
✅ Confirmar: `Loaded node '/servo_server'`

> Si aparece `Failed loading controller lite6_traj_controller` — es **normal**, T2 ya lo cargó.

---

### Terminal 6 — Nodo de teleoperación bilateral
```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 run haptic_teleop bilateral_teleop --ros-args \
  --params-file src/haptic_teleop/config/teleop_params.yaml
```
✅ Confirmar: `✅ Calibración OK → RUNNING`

El nodo primero mueve el maestro a HOME automáticamente, espera 2 segundos y luego calibra. No toques ninguno de los dos brazos durante la calibración.

---

## Parámetros (`config/teleop_params.yaml`)

| Parámetro | Valor actual | Descripción |
|---|---|---|
| `traj_time_ms` | 60 | Duración de cada trayectoria enviada al esclavo [ms] |
| `still_thr_deg` | 0.1 | Umbral de movimiento para considerar maestro quieto [°] |
| `still_cycles` | 3 | Ciclos quieto antes de mandar hold al esclavo |
| `force_threshold` | 3900 | Umbral ADC para detectar contacto en FSR |
| `force_hysteresis` | 3950 | Umbral ADC para liberar contacto FSR |
| `calib_samples` | 80 | Muestras para calibración de offsets y baseline de esfuerzo |
| `watchdog_timeout` | 0.5 | Tiempo sin datos de fuerza antes de EMERGENCY [s] |
| `torque_gain` | 0.15 | Ganancia del haptic feedback por error de posición |

Los umbrales de haptic por esfuerzo (`EFFORT_THR`, `HAPTIC_GAIN`, `HAPTIC_MAX`) se editan directamente en `bilateral_teleop.py`:

```python
EFFORT_THR   = 2.0   # N·m — umbral para detectar colisión
HAPTIC_GAIN  = 0.35  # ganancia de reflejo al maestro
HAPTIC_MAX   = 3.0   # N·m máximo enviado al maestro
```

---

## Arquitectura FSM

```
CALIBRATING ──(80 muestras estables)──► RUNNING
                                            │
                                    (contacto FSR)
                                            │
                                            ▼
                                       FORCE_STOP ──(sensor libre)──► RUNNING

RUNNING ──(watchdog timeout)──► EMERGENCY
```

| Estado | Comportamiento |
|---|---|
| **CALIBRATING** | Acumula 80 muestras de posición Y esfuerzo de ambos robots. Calcula `offset = q_slave - q_master` y `tau_baseline` (gravedad en reposo). Rechaza si algún robot se mueve (std > 0.017 rad). |
| **RUNNING** | Teleoperación activa a 50 Hz. Detecta colisiones por esfuerzo externo (`tau_slave - tau_baseline`). Bloquea movimiento hacia la colisión y refleja fuerza al maestro. |
| **FORCE_STOP** | Contacto FSR detectado. Esclavo se detiene. Se reanuda automáticamente al liberar el sensor. |
| **EMERGENCY** | Watchdog disparado por pérdida del sensor FSR. Requiere reinicio del nodo. |

---

## Haptic feedback por esfuerzo (colisiones)

El nodo detecta colisiones en el esclavo usando los esfuerzos articulares de `/joint_states` sin depender únicamente del sensor FSR.

**Flujo:**
1. Durante calibración se mide `tau_baseline` (esfuerzo promedio en reposo = gravedad).
2. En RUNNING: `tau_external = tau_slave - tau_baseline`.
3. Si `|tau_external[i]| > 2.0 N·m` → colisión detectada en joint i.
4. Se bloquea el movimiento del esclavo en la dirección de la fuerza.
5. Se publica `-HAPTIC_GAIN * tau_external` en `/master/reflected_torques` para que el operador sienta la resistencia.

El operador siente el brazo maestro "duro" en la dirección donde el esclavo está siendo obstaculizado, sin necesidad de ver el esclavo.

---

## Topics

| Topic | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/master/xarm/joint_states` | `sensor_msgs/JointState` | Entrada | Estado articular del maestro (~90 Hz) |
| `/master/joint_states` | `sensor_msgs/JointState` | Entrada | Relay del maestro |
| `/joint_states` | `sensor_msgs/JointState` | Entrada | Estado articular + esfuerzos del esclavo (~150 Hz) |
| `/force_sensor` | `std_msgs/Int32` | Entrada | ADC del ESP32 (reposo ~4095, contacto <3900) |
| `/lite6_traj_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Salida | Trayectorias directas al controlador del esclavo |
| `/master/reflected_torques` | `std_msgs/Float64MultiArray` | Salida | Torques reflejados al maestro (haptic) |

---

## Recuperación de error C31

El error C31 (`Collision Caused Abnormal Joint Current`) ocurre por colisión real o trayectoria demasiado agresiva.

```bash
python3 ~/recover_slave.py
```

Contenido de `recover_slave.py`:
```python
from xarm.wrapper import XArmAPI

HOME_DEG = [1.1043, 19.0107, 74.6311, -5.9974, 58.5693, -3.0649]

arm = XArmAPI('192.168.1.175')
arm.clean_error()
arm.clean_warn()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)
arm.set_servo_angle(angle=HOME_DEG, speed=20, wait=True)
arm.set_mode(1)
arm.set_state(0)
arm.disconnect()
print("✅ Esclavo recuperado")
```

Después de correr el script, **reinicia Terminal 5** para recargar `lite6_traj_controller`.

---

## Modo teach del maestro (mover a mano)

```bash
python3 - << 'EOF'
from xarm.wrapper import XArmAPI
arm = XArmAPI('192.168.1.167')
arm.motion_enable(enable=True)
arm.set_mode(2)   # modo teach — robot libre para mover a mano
arm.set_state(0)
print(f"Modo: {arm.mode}, Estado: {arm.state}")
print("✅ Maestro en modo teach — muévelo a mano")
input("Presiona Enter para regresar a modo normal...")
arm.set_mode(0)
arm.set_state(0)
print("✅ Regresado a modo normal")
EOF
```

---

## Diagnóstico

```bash
# Frecuencias de topics críticos
ros2 topic hz /master/joint_states       # debe ser ~90 Hz
ros2 topic hz /joint_states              # debe ser ~150 Hz
ros2 topic hz /force_sensor              # debe ser ~10 Hz

# Verificar esfuerzos del esclavo (colisión si |joint1| > 2 N·m sobre baseline)
ros2 topic echo /joint_states --field effort

# Verificar haptic feedback publicado al maestro
ros2 topic echo /master/reflected_torques

# Nodos activos
ros2 node list
```

---

## Problemas conocidos

| Problema | Causa | Solución |
|---|---|---|
| Calibración stuck en 0/80 | Topic del maestro a 5 Hz | Usar `report_type:=dev` en T3 |
| Calibración falla con std alta | Robots moviéndose al iniciar | No tocar los robots durante los primeros 3s |
| Esclavo oscila con maestro quieto | Cola de trayectorias del controlador no cancelada | El nodo envía `points=[]` para cancelar la cola automáticamente |
| Error C31 en movimiento | Trayectoria demasiado agresiva o colisión real | Ejecutar `recover_slave.py` y reiniciar T5 |
| `servo_server status: 0` | Comportamiento normal en esta versión xArm | Ignorar — los comandos sí llegan al controlador |
| `Failed loading lite6_traj_controller` en T5 | T2 ya lo cargó | Normal — no afecta el funcionamiento |
| Haptic no se siente en maestro | El SDK del maestro debe estar en modo que acepte torques | Verificar que `/master/reflected_torques` se publica con valores distintos de cero con `ros2 topic echo` |