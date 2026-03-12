# haptic_teleop

Paquete ROS 2 de teleoperación bilateral háptica para **xArm Lite 6**.  
**TE3001B – Kinematics Challenge | ITESM Robótica**  
---

## Hardware

| Robot | IP | Rol | Namespace |
|---|---|---|---|
| xArm Lite 6 | 192.168.1.175 | **Esclavo** – ejecuta movimientos | `/` (global) |
| xArm Lite 6 | 192.168.1.167 | **Maestro** – operado por humano | `/master` |
| ESP32 + micro-ROS | USB `/dev/ttyUSB0` | Sensor de fuerza | — |

El sensor ESP32 publica lecturas ADC en `/force_sensor` (Int32). Valor en reposo ≈ 4095; contacto detectado cuando baja de 3900.

---

## Estructura del paquete

```
haptic_teleop/
├── haptic_teleop/
│   ├── __init__.py
│   ├── bilateral_teleop.py     ← Nodo principal (FSM + filtro + bilateral)
│   ├── force_stop_node.py      ← Capa de seguridad independiente
│   └── master_teleop.py        ← Nodo del robot maestro + haptic feedback
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
cd ~/Challenge-Kinematics
colcon build --symlink-install --packages-select haptic_teleop
source install/setup.bash
```

Verifica que los 3 ejecutables están disponibles:

```bash
ros2 pkg executables haptic_teleop
# haptic_teleop bilateral_teleop
# haptic_teleop force_stop_node
# haptic_teleop master_teleop
```

---

## Secuencia de arranque (6 terminales)

### Terminal 1 — micro-ROS (sensor de fuerza ESP32)
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### Terminal 2 — MoveIt completo del esclavo (move_group + driver + controlador)
```bash
cd ~/Challenge-Kinematics && source install/setup.bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.175 \
  add_realsense_d435i:=false
```
Espera: `You can start planning now!`

### Terminal 3 — Driver del robot maestro (namespace /master)
```bash
cd ~/Challenge-Kinematics && source install/setup.bash
ros2 run xarm_api xarm_driver_node \
  --ros-args \
  -r __ns:=/master \
  -p robot_ip:=192.168.1.167 \
  -p dof:=6 \
  -p report_type:=dev
```
Espera: `[TCP STATUS] CONTROL: 1`

> **Nota:** `report_type:=dev` es obligatorio para obtener ~90 Hz en `/master/xarm/joint_states`.  
> Con `report_type:=normal` solo se obtienen ~5 Hz y la calibración falla.

### Terminal 4 — Relay de topics del maestro
```bash
cd ~/Challenge-Kinematics && source install/setup.bash
ros2 run topic_tools relay \
  /master/xarm/joint_states \
  /master/joint_states
```

### Terminal 5 — MoveIt Servo (servidor de comandos de velocidad)
```bash
cd ~/Challenge-Kinematics && source install/setup.bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.175
```
Espera: `Loaded node '/servo_server'`

### Terminal 6 — Activar servo + teleoperación bilateral
```bash
cd ~/Challenge-Kinematics && source install/setup.bash
ros2 service call /servo_server/start_servo std_srvs/srv/Trigger "{}"
ros2 run haptic_teleop bilateral_teleop --ros-args \
  --params-file src/haptic_teleop/config/teleop_params.yaml
```
Espera: `✅ Calibración OK → RUNNING`

---

## Modo teach del maestro (mover a mano)

Para mover el robot maestro físicamente, ponlo en modo teach:

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

Instalar SDK si no está disponible:
```bash
pip install xarm-python-sdk
```

---

## Topics

| Topic | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/master/xarm/joint_states` | `sensor_msgs/JointState` | Entrada | Estado articular del maestro (~90 Hz) |
| `/master/joint_states` | `sensor_msgs/JointState` | Entrada | Relay del maestro (usado por bilateral_teleop) |
| `/joint_states` | `sensor_msgs/JointState` | Entrada | Estado articular del esclavo (~150 Hz) |
| `/force_sensor` | `std_msgs/Int32` | Entrada | ADC del ESP32 (reposo ~4095, contacto <3900) |
| `/servo_server/delta_joint_cmds` | `control_msgs/JointJog` | Salida | Comandos de velocidad al esclavo (50 Hz) |
| `/master/reflected_torques` | `std_msgs/Float64MultiArray` | Salida | Torques reflejados al maestro (haptic feedback) |

---

## Parámetros (`config/teleop_params.yaml`)

| Parámetro | Valor actual | Descripción |
|---|---|---|
| `kp` | 0.8 | Ganancia proporcional del controlador P [rad/s per rad] |
| `max_vel` | 0.6 | Velocidad máxima del esclavo [rad/s] |
| `max_accel` | 1.0 | Rampa máxima de aceleración [rad/s²] |
| `alpha_vel` | 0.08 | Suavizado exponencial del filtro (0=máx suavidad) |
| `force_threshold` | 3900 | Umbral ADC para detectar contacto |
| `force_hysteresis` | 3950 | Umbral ADC para liberar contacto |
| `calib_samples` | 80 | Muestras para calibración de offsets |
| `calib_std_max` | 0.02 | Desviación estándar máxima para calibrar [rad] |
| `watchdog_timeout` | 0.5 | Tiempo sin fuerza antes de EMERGENCY [s] |
| `torque_gain` | 0.15 | Ganancia del haptic feedback [N·m] |

---

## Arquitectura FSM

```
CALIBRATING ──(80 muestras estables)──► RUNNING
                                            │
                                    (contacto sensor)
                                            │
                                            ▼
                                       FORCE_STOP ──(sensor libre)──► RUNNING
                                            
RUNNING ──(watchdog timeout)──► EMERGENCY
```

### Estados

- **CALIBRATING**: Acumula 80 muestras de posición de ambos robots para calcular el offset `q_slave - q_master`. Rechaza si algún robot se mueve (std > 0.02 rad).
- **RUNNING**: Teleoperación activa. El esclavo sigue al maestro a 50 Hz usando control proporcional con zona muerta (±0.015 rad) y filtro exponencial.
- **FORCE_STOP**: Contacto detectado. El esclavo se detiene completamente. El maestro recibe feedback háptico proporcional al error de posición.
- **EMERGENCY**: Watchdog disparado por pérdida del sensor de fuerza. Requiere reinicio del nodo.

---

## Diagnóstico

```bash
# Verificar frecuencia de topics críticos
ros2 topic hz /master/joint_states       # debe ser ~90 Hz
ros2 topic hz /joint_states              # debe ser ~150 Hz
ros2 topic hz /force_sensor              # debe ser ~10 Hz
ros2 topic hz /servo_server/delta_joint_cmds  # debe ser ~50 Hz

# Verificar que los robots publican posiciones distintas
ros2 topic echo /master/joint_states --once | grep -A7 position
ros2 topic echo /joint_states --once | grep -A7 position

# Estado del servo
ros2 topic echo /servo_server/status --once

# Identificar nodos activos
ros2 node list
```

---

## Problemas conocidos y soluciones

| Problema | Causa | Solución |
|---|---|---|
| Calibración nunca completa (stuck en 0/80) | Topic del maestro a 5 Hz | Usar `report_type:=dev` en Terminal 3 |
| Calibración falla con std alta en joint4 | Topics del maestro y esclavo colisionan en `/ufactory/joint_states` | Usar `ros2 run xarm_api xarm_driver_node --ros-args -r __ns:=/master` |
| Esclavo oscila cuando maestro está quieto | Inercia residual del filtro exponencial | `alpha_vel: 0.08` + zona muerta 0.015 rad |
| `[set_state], xArm is not ready` en T2 | ros2_control sobreescribe el modo | Normal — el hardware interface gestiona el modo automáticamente |
| `Failed loading controller lite6_traj_controller` en T5 | T2 ya cargó el controlador | Normal — el servo usa el controlador existente de T2 |
| Servo `status: 0` después de `start_servo` | Comportamiento normal de esta versión del servo xArm | Ignorar — los comandos sí llegan al controlador |