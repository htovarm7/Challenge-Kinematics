#!/usr/bin/env python3
"""
haptic_teleop/bilateral_teleop.py
==================================
Sistema de Teleoperación Bilateral Maestro-Esclavo para xArm Lite 6
Autor : Jose Luis Dominguez – ITESM Robótica
Curso  : TE3001B – Kinematics Challenge

Arquitectura:
  - Maestro  (IP 192.168.1.167) → namespace /master  → publica /master/joint_states
  - Esclavo  (IP 192.168.1.175) → namespace global   → controlado por moveit_servo
  - Sensor   ESP32 + micro-ROS  → publica /force_sensor (Int32, reposo ~4095)

Nodo ROS 2: bilateral_teleop
  ros2 run haptic_teleop bilateral_teleop
  ros2 run haptic_teleop bilateral_teleop --ros-args --params-file <yaml>
"""

# ──────────────────────────────────────────────────────────────────────────────
#  IMPORTS
# ──────────────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node     import Node
from rclpy.qos      import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg     import JointState
from std_msgs.msg        import Int32, Float64MultiArray
from control_msgs.msg    import JointJog
from trajectory_msgs.msg import JointTrajectory

import numpy as np
from collections import deque
from enum        import Enum, auto
import threading

# ──────────────────────────────────────────────────────────────────────────────
#  CONSTANTES Y PARÁMETROS POR DEFECTO
# ──────────────────────────────────────────────────────────────────────────────
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
N_JOINTS    = len(JOINT_NAMES)

# Control
KP_DEFAULT       = 1.2    # [rad/s per rad]
MAX_VEL_DEFAULT  = 0.8    # [rad/s]
MAX_ACCEL_DEFAULT= 1.5    # [rad/s²]
ALPHA_DEFAULT    = 0.35   # Suavizado exponencial
RATE_HZ          = 50.0
DT               = 1.0 / RATE_HZ

# Calibración
CALIB_SAMPLES    = 80
CALIB_STD_MAX    = 0.005  # [rad] – máx desviación estándar permitida

# Fuerza
FORCE_THR_DEF    = 3900
FORCE_HYS_DEF    = 3950
FORCE_WIN        = 3      # muestras consecutivas para confirmar contacto
WATCHDOG_TIMEOUT = 0.5    # [s]

# Bilateral
TORQUE_GAIN_DEF  = 0.15


# ──────────────────────────────────────────────────────────────────────────────
#  FSM – Estados del nodo
# ──────────────────────────────────────────────────────────────────────────────
class TeleopState(Enum):
    CALIBRATING = auto()   # Acumulando muestras para calcular offsets
    RUNNING     = auto()   # Teleoperación activa
    FORCE_STOP  = auto()   # Contacto detectado – movimiento bloqueado
    EMERGENCY   = auto()   # Watchdog o error crítico


# ──────────────────────────────────────────────────────────────────────────────
#  FILTRO DE VELOCIDAD  (exponencial + rampa de aceleración)
# ──────────────────────────────────────────────────────────────────────────────
class VelocityFilter:
    """
    Capa 1 – Filtro paso-bajo exponencial:
        v_smooth[k] = α·v_raw[k] + (1-α)·v_smooth[k-1]
    Capa 2 – Clip de aceleración (rampa lineal):
        Δv  limitado a  max_accel · DT  por paso
    Capa 3 – Clip de velocidad máxima absoluta.
    """
    def __init__(self, n, alpha, max_vel, max_accel, dt):
        self.alpha     = alpha
        self.max_vel   = max_vel
        self.max_accel = max_accel
        self.dt        = dt
        self._smooth   = np.zeros(n)
        self._prev     = np.zeros(n)

    def reset(self):
        self._smooth[:] = 0.0
        self._prev[:]   = 0.0
        
    def force_stop(self):
        """Reset agresivo — para inmediato sin inercia residual."""
        self._smooth[:] = 0.0
        self._prev[:]   = 0.0


    def __call__(self, raw: np.ndarray) -> np.ndarray:
        # 1) Exponencial
        self._smooth = self.alpha * raw + (1.0 - self.alpha) * self._smooth
        # 2) Rampa
        delta  = np.clip(self._smooth - self._prev,
                         -self.max_accel * self.dt,
                          self.max_accel * self.dt)
        output = self._prev + delta
        self._prev = output.copy()
        # 3) Límite absoluto
        return np.clip(output, -self.max_vel, self.max_vel)


# ──────────────────────────────────────────────────────────────────────────────
#  NODO PRINCIPAL
# ──────────────────────────────────────────────────────────────────────────────
class BilateralTeleop(Node):

    def __init__(self):
        super().__init__("bilateral_teleop")
        self._declare_and_load_params()

        # ── Estado ────────────────────────────────────────────────
        self.state      = TeleopState.CALIBRATING
        self._lock      = threading.Lock()

        # ── Articulaciones ────────────────────────────────────────
        self.q_master   = np.zeros(N_JOINTS)
        self.q_slave    = np.zeros(N_JOINTS)
        self.offset     = np.zeros(N_JOINTS)   # q_s0 - q_m0
        self._m_ready   = False
        self._s_ready   = False
        self._buf_m     = []
        self._buf_s     = []

        # ── Filtro de velocidad ───────────────────────────────────
        self.vfilter = VelocityFilter(
            N_JOINTS, self.alpha, self.max_vel, self.max_accel, DT
        )

        # ── Sensor de fuerza ──────────────────────────────────────
        self._fwin          = deque(maxlen=FORCE_WIN)
        self._last_force_t  = self.get_clock().now()
        self._in_contact    = False

        # ── QoS ───────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Suscriptores ──────────────────────────────────────────
        self.create_subscription(JointState, "/master/joint_states",
                                 self._cb_master, qos_be)
        self.create_subscription(JointState, "/joint_states",
                                 self._cb_slave,  qos_be)
        self.create_subscription(Int32,      "/force_sensor",
                                 self._cb_force,  qos_be)

        # ── Publicadores ──────────────────────────────────────────
        self._pub_servo  = self.create_publisher(
            JointJog,        "/servo_server/delta_joint_cmds", qos_rel)
        self._pub_traj   = self.create_publisher(
            JointTrajectory, "/xarm_traj_controller/joint_trajectory", qos_rel)
        self._pub_torque = self.create_publisher(
            Float64MultiArray, "/master/reflected_torques", qos_rel)

        # ── Timers ────────────────────────────────────────────────
        self.create_timer(DT,  self._control_loop)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            "✅ bilateral_teleop iniciado  |  estado: CALIBRATING\n"
            f"   Kp={self.kp}  max_vel={self.max_vel}  alpha={self.alpha}"
        )

    # ─────────────────────────────────────────────────────────
    #  PARÁMETROS
    # ─────────────────────────────────────────────────────────
    def _declare_and_load_params(self):
        self.declare_parameter("kp",               KP_DEFAULT)
        self.declare_parameter("max_vel",          MAX_VEL_DEFAULT)
        self.declare_parameter("max_accel",        MAX_ACCEL_DEFAULT)
        self.declare_parameter("alpha_vel",        ALPHA_DEFAULT)
        self.declare_parameter("force_threshold",  FORCE_THR_DEF)
        self.declare_parameter("force_hysteresis", FORCE_HYS_DEF)
        self.declare_parameter("calib_samples",    CALIB_SAMPLES)
        self.declare_parameter("calib_std_max",    0.02)
        self.declare_parameter("watchdog_timeout", WATCHDOG_TIMEOUT)
        self.declare_parameter("torque_gain",      TORQUE_GAIN_DEF)

        self.kp           = self.get_parameter("kp").value
        self.max_vel      = self.get_parameter("max_vel").value
        self.max_accel    = self.get_parameter("max_accel").value
        self.alpha        = self.get_parameter("alpha_vel").value
        self.force_thr    = self.get_parameter("force_threshold").value
        self.force_hys    = self.get_parameter("force_hysteresis").value
        self.calib_n      = self.get_parameter("calib_samples").value
        self.calib_std_max = self.get_parameter("calib_std_max").value
        self.wdog_t       = self.get_parameter("watchdog_timeout").value
        self.torque_gain  = self.get_parameter("torque_gain").value

    # ─────────────────────────────────────────────────────────
    #  CALLBACKS
    # ─────────────────────────────────────────────────────────
    def _cb_master(self, msg: JointState):
        pos = self._extract(msg)
        if pos is None:
            return
        self.q_master = pos
        self._m_ready = True
        if self.state == TeleopState.CALIBRATING:
            self._buf_m.append(pos.copy())
            self._try_calibrate()

    def _cb_slave(self, msg: JointState):
        pos = self._extract(msg)
        if pos is None:
            return
        self.q_slave  = pos
        self._s_ready = True
        if self.state == TeleopState.CALIBRATING:
            self._buf_s.append(pos.copy())

    def _cb_force(self, msg: Int32):
        self._fwin.append(msg.data)
        self._last_force_t = self.get_clock().now()

        if len(self._fwin) < FORCE_WIN:
            return

        contact  = all(v < self.force_thr for v in self._fwin)
        released = all(v > self.force_hys for v in self._fwin)

        with self._lock:
            if contact and not self._in_contact:
                self._in_contact = True
                if self.state == TeleopState.RUNNING:
                    self._enter_force_stop()
            elif released and self._in_contact:
                self._in_contact = False
                if self.state == TeleopState.FORCE_STOP:
                    self._exit_force_stop()

    # ─────────────────────────────────────────────────────────
    #  CALIBRACIÓN DE OFFSETS
    # ─────────────────────────────────────────────────────────
    def _try_calibrate(self):
        n = min(len(self._buf_m), len(self._buf_s))
        if n < self.calib_n:
            remaining = self.calib_n - n
            if n % 20 == 0:  # log cada 20 muestras
                self.get_logger().info(
                    f"Calibrando... {n}/{self.calib_n} muestras")
            return

        arr_m = np.array(self._buf_m[-self.calib_n:])
        arr_s = np.array(self._buf_s[-self.calib_n:])

        # Filtrar ruido numérico del driver (velocidades ~1e-15 no son movimiento real)
        # Solo rechazar si la std supera 1 grado (0.017 rad) — movimiento real
        POS_STD_MAX = 0.017  # rad (~1 grado)
        if (arr_m.std(axis=0) > POS_STD_MAX).any() or \
        (arr_s.std(axis=0) > POS_STD_MAX).any():
            self.get_logger().warn(
                f"⚠ Robots moviéndose durante calibración. Reintentando...\n"
                f"  std_m={np.round(arr_m.std(axis=0),4)}\n"
                f"  std_s={np.round(arr_s.std(axis=0),4)}")
            self._buf_m.clear()
            self._buf_s.clear()
            return

        self.offset = arr_s.mean(axis=0) - arr_m.mean(axis=0)
        self.vfilter.reset()
        self.state = TeleopState.RUNNING

        self.get_logger().info(
            f"✅ Calibración OK\n"
            f"   offset = {np.round(self.offset, 4)} rad\n"
            f"   std_m  = {np.round(arr_m.std(axis=0), 5)}\n"
            f"   std_s  = {np.round(arr_s.std(axis=0), 5)}\n"
            f"   Estado → RUNNING"
        )

    # ─────────────────────────────────────────────────────────
    #  TRANSICIONES FORCE STOP
    # ─────────────────────────────────────────────────────────
    def _enter_force_stop(self):
        self.state = TeleopState.FORCE_STOP
        self.vfilter.reset()
        # Parada total — cero velocidades, sin trayectoria de regreso
        self._pub_zero()
        self._pub_zero()  # doble envío para garantizar
        self.get_logger().warn("🛑 FORCE STOP – contacto detectado, manteniendo posición")

    def _exit_force_stop(self):
        self.state = TeleopState.RUNNING
        self.vfilter.reset()
        self.get_logger().info("▶ FORCE STOP liberado – reanudando")

    # ─────────────────────────────────────────────────────────
    #  LOOP DE CONTROL  (50 Hz)
    # ─────────────────────────────────────────────────────────
    def _control_loop(self):
        with self._lock:
            st = self.state

        if st != TeleopState.RUNNING:
            return
        if not (self._m_ready and self._s_ready):
            return

        # 1. Error de posición con offset de calibración
        q_des = self.q_master + self.offset
        q_err = q_des - self.q_slave

        # 2. Zona muerta dinámica
        DEADBAND = 0.015
        q_err_db = np.where(np.abs(q_err) < DEADBAND, 0.0, q_err)

        # 3. Si TODO el error está en zona muerta → parada limpia
        if np.all(q_err_db == 0.0):
            self.vfilter.force_stop()
            self._pub_zero()
            self._pub_zero()
            self._reflect_torques(q_err)
            return

        # 3b. Si el maestro está quieto (velocidad baja) → misma lógica
        master_moving = np.any(np.abs(q_err_db) > DEADBAND)
        if not master_moving:
            self.vfilter.force_stop()
            self._pub_zero()
            self._pub_zero()
            return

        # 4. Velocidad de referencia proporcional
        vel_raw = self.kp * q_err_db

        # 5. Filtrado (exponencial + rampa)
        vel_cmd = self.vfilter(vel_raw)

        # 6. Umbral mínimo de velocidad — evita comandos de velocidad casi cero
        MIN_VEL = 0.005  # rad/s
        vel_cmd = np.where(np.abs(vel_cmd) < MIN_VEL, 0.0, vel_cmd)

        # 7. Si todos los comandos son cero → resetea filtro y para
        if np.all(vel_cmd == 0.0):
            self.vfilter.reset()
            self._pub_zero()
            return

        # 8. Publicar al servo
        self._pub_jog(vel_cmd)

        # 9. Retroalimentación bilateral
        self._reflect_torques(q_err)

    # ─────────────────────────────────────────────────────────
    #  RETROALIMENTACIÓN BILATERAL
    # ─────────────────────────────────────────────────────────
    def _reflect_torques(self, q_err: np.ndarray):
        """
        Retroalimentación bilateral activa:
        - En contacto: refleja fuerza proporcional al error acumulado
        - En movimiento libre: amortiguamiento suave para dar sensación de inercia
        """
        if self._in_contact:
            # Fuerza resistiva proporcional — el operador siente la colisión
            tau = -self.torque_gain * q_err * 3.0
        else:
            # Amortiguamiento leve en movimiento libre
            tau = -self.torque_gain * 0.1 * q_err

        tau = np.clip(tau, -2.0, 2.0)  # límite de seguridad [N·m]

        msg      = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub_torque.publish(msg)

        if self._in_contact:
            self.get_logger().info(
                f"Haptic τ = {np.round(tau, 3)} N·m",
                throttle_duration_sec=0.5
            )

    # ─────────────────────────────────────────────────────────
    #  WATCHDOG
    # ─────────────────────────────────────────────────────────
    def _watchdog(self):
        elapsed = (self.get_clock().now() - self._last_force_t).nanoseconds * 1e-9
        if elapsed > self.wdog_t:
            with self._lock:
                if self.state == TeleopState.RUNNING:
                    self.state = TeleopState.EMERGENCY
                    self.vfilter.reset()
                    self._pub_zero()
                    self.get_logger().error(
                        f"🚨 EMERGENCY  – sin datos de fuerza por {elapsed:.2f}s")

    # ─────────────────────────────────────────────────────────
    #  HELPERS
    # ─────────────────────────────────────────────────────────
    def _pub_jog(self, vel: np.ndarray):
        msg               = JointJog()
        msg.header.stamp  = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.joint_names   = JOINT_NAMES
        msg.velocities    = vel.tolist()
        msg.duration      = DT
        self._pub_servo.publish(msg)

    def _pub_zero(self):
        self._pub_jog(np.zeros(N_JOINTS))

    def _extract(self, msg: JointState) -> np.ndarray | None:
        m = dict(zip(msg.name, msg.position))
        try:
            return np.array([m[j] for j in JOINT_NAMES])
        except KeyError as e:
            self.get_logger().warn(
                f"Joint faltante: {e}", throttle_duration_sec=2.0)
            return None

    # ─────────────────────────────────────────────────────────
    #  API PÚBLICA
    # ─────────────────────────────────────────────────────────
    def recalibrate(self):
        """Reinicia calibración (llámalo desde un servicio externo si lo necesitas)."""
        with self._lock:
            self._buf_m.clear()
            self._buf_s.clear()
            self._m_ready = self._s_ready = False
            self.vfilter.reset()
            self.state = TeleopState.CALIBRATING
        self.get_logger().info("🔄 Recalibrando...")


# ──────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = BilateralTeleop()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🔴 Shutdown")
    finally:
        node._pub_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
