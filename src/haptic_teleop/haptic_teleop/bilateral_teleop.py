#!/usr/bin/env python3
"""
haptic_teleop/bilateral_teleop.py
==================================
Sistema de Teleoperación Bilateral Maestro-Esclavo para xArm Lite 6
Autor : Jose Luis Dominguez – ITESM Robótica
Curso  : TE3001B – Kinematics Challenge
"""

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg        import JointState
from std_msgs.msg           import Int32, Float64MultiArray, Bool
from trajectory_msgs.msg    import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
from collections import deque
from enum        import Enum, auto
from pathlib     import Path
import threading
import time
import csv
import datetime
import json

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
N_JOINTS    = len(JOINT_NAMES)
RATE_HZ     = 50.0
DT          = 1.0 / RATE_HZ
CALIB_STD_MAX = 0.017

# ── Effort-based collision detection thresholds ──────────
EFFORT_THR   = 3.5    # N·m — umbral para detectar colisión (>3.5 para evitar falsos)
HAPTIC_GAIN  = 0.35   # ganancia de reflejo al maestro
HAPTIC_MAX   = 3.0    # N·m máximo enviado al maestro
COLLISION_CONFIRM = 5 # muestras consecutivas para confirmar (5×20ms = 100ms)

HOME_RAD = [
     0.01927255094051361,
     0.3318978250026703,
     1.3028117418289185,
    -0.1046941876411438,
     1.0222543478012085,
    -0.05349757894873619,
]

class TeleopState(Enum):
    CALIBRATING = auto()
    RUNNING     = auto()
    FORCE_STOP  = auto()
    EMERGENCY   = auto()


class BilateralTeleop(Node):

    def __init__(self):
        super().__init__("bilateral_teleop")
        self._declare_and_load_params()

        self.state   = TeleopState.CALIBRATING
        self._lock   = threading.Lock()

        self.q_master        = np.zeros(N_JOINTS)
        self.q_slave         = np.zeros(N_JOINTS)
        self.offset          = np.zeros(N_JOINTS)
        self._q_master_prev  = np.zeros(N_JOINTS)
        self._still_count    = 0
        self._m_ready        = False
        self._s_ready        = False
        self._buf_m          = []
        self._buf_s          = []

        self._fwin         = deque(maxlen=3)
        self._last_force_t = self.get_clock().now()
        self._force_ever_received = False
        self._force_raw    = 4095          # último valor crudo del sensor
        self._in_contact   = False
        self._collision_active = False
        self._last_tau     = np.zeros(N_JOINTS)
        self._external_perturbation = False  # perturbación detectada por q_err

        # ── Effort-based collision detection ────────────────
        self._tau_slave       = np.zeros(N_JOINTS)   # esfuerzo actual esclavo
        self._tau_master      = np.zeros(N_JOINTS)   # esfuerzo actual maestro
        self._tau_baseline    = np.zeros(N_JOINTS)   # gravedad en reposo
        self._tau_baseline_ok = False
        self._buf_tau         = []                    # buffer calibración esfuerzos
        self._effort_collision_win = deque(maxlen=COLLISION_CONFIRM)
        self._effort_collision_active = False         # colisión por esfuerzo activa
        self._collision_event_id = 0                  # contador de eventos

        # ── Hilo de gestión del maestro (lock/unlock) ────────
        self._master_locked   = False   # True = maestro bloqueado en posición
        self._master_lock_req = False   # petición de bloqueo
        self._master_lock_thr = threading.Thread(
            target=self._master_lock_loop, daemon=True)
        self._master_lock_thr.start()

        # ── Logging de experimentos ────────────────────────────
        self._init_experiment_logger()

        # ── Conexión persistente al maestro (SDK) ────────────────
        self._master_arm = None
        self._init_master()
        # NOTA: NO tocar el esclavo con SDK directo — ros2_control lo gestiona
        # El usuario debe correr recover_slave.py antes de este nodo si es necesario

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # Publicador de joint states del maestro (reemplaza xarm_driver + relay)
        self._pub_master_js = self.create_publisher(
            JointState, "/master/joint_states", qos_be)

        self.create_subscription(JointState, "/master/joint_states",
                                 self._cb_master, qos_be)
        self.create_subscription(JointState, "/joint_states",
                                 self._cb_slave, qos_be)
        self.create_subscription(Int32, "/force_sensor",
                                 self._cb_force, qos_be)
        self.create_subscription(Bool, "/slave/collision_active",
                                 self._cb_collision, qos_rel)

        self._pub_traj = self.create_publisher(
            JointTrajectory,
            "/lite6_traj_controller/joint_trajectory",
            qos_rel)
        self._pub_torque = self.create_publisher(
            Float64MultiArray,
            "/master/reflected_torques",
            qos_rel)

        # Timer para leer posiciones del maestro vía SDK (~90 Hz)
        self.create_timer(1.0 / 90.0, self._poll_master)
        self.create_timer(DT,  self._control_loop)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            "✅ bilateral_teleop iniciado  |  estado: CALIBRATING\n"
            f"   traj_time={self.traj_time_ms}ms  "
            f"still_thr={self.still_thr_deg}°  "
            f"torque_gain={self.torque_gain}"
        )

    # ── Experiment logging ──────────────────────────────────
    def _init_experiment_logger(self):
        self._exp_dir = Path("/home/hector/Desktop/Challenge-Kinematics/experiments")
        self._exp_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_stamp = stamp

        # ── CSV principal: todo el run ──────────────────────
        self._csv_path = self._exp_dir / f"teleop_{stamp}.csv"
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        header = (
            ["timestamp", "state", "force_sensor", "in_contact",
             "effort_collision"]
            + [f"master_pos_j{i+1}" for i in range(N_JOINTS)]
            + [f"slave_pos_j{i+1}" for i in range(N_JOINTS)]
            + [f"slave_effort_j{i+1}" for i in range(N_JOINTS)]
            + [f"slave_ext_torque_j{i+1}" for i in range(N_JOINTS)]
            + [f"master_effort_j{i+1}" for i in range(N_JOINTS)]
            + [f"reflected_tau_j{i+1}" for i in range(N_JOINTS)]
            + [f"q_err_j{i+1}" for i in range(N_JOINTS)]
        )
        self._csv_writer.writerow(header)
        self.get_logger().info(f"Experiment log: {self._csv_path}")

        # ── CSV de colisiones: solo eventos de fuerza/contacto ──
        self._collision_csv_path = self._exp_dir / f"collisions_{stamp}.csv"
        self._collision_csv_file = open(self._collision_csv_path, "w", newline="")
        self._collision_csv_writer = csv.writer(self._collision_csv_file)
        col_header = (
            ["timestamp", "event_id", "trigger",
             "force_sensor_raw"]
            + [f"slave_pos_j{i+1}" for i in range(N_JOINTS)]
            + [f"slave_effort_j{i+1}" for i in range(N_JOINTS)]
            + [f"slave_ext_torque_j{i+1}" for i in range(N_JOINTS)]
            + [f"master_pos_j{i+1}" for i in range(N_JOINTS)]
            + [f"master_effort_j{i+1}" for i in range(N_JOINTS)]
            + [f"reflected_tau_j{i+1}" for i in range(N_JOINTS)]
        )
        self._collision_csv_writer.writerow(col_header)
        self.get_logger().info(f"Collision log: {self._collision_csv_path}")

    def _log_experiment(self, q_err: np.ndarray):
        """Escribe una fila al CSV principal con el estado actual."""
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            tau_ext = (self._tau_slave - self._tau_baseline
                       if self._tau_baseline_ok
                       else np.zeros(N_JOINTS))
            row = (
                [f"{now:.6f}", self.state.name, self._force_raw,
                 int(self._in_contact),
                 int(self._effort_collision_active)]
                + [f"{v:.6f}" for v in self.q_master]
                + [f"{v:.6f}" for v in self.q_slave]
                + [f"{v:.6f}" for v in self._tau_slave]
                + [f"{v:.6f}" for v in tau_ext]
                + [f"{v:.6f}" for v in self._tau_master]
                + [f"{v:.6f}" for v in self._last_tau]
                + [f"{v:.6f}" for v in q_err]
            )
            self._csv_writer.writerow(row)
        except Exception:
            pass

    def _log_collision_event(self, trigger: str, reflected_tau: np.ndarray):
        """Escribe una fila al CSV de colisiones cuando hay contacto/fuerza."""
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            tau_ext = (self._tau_slave - self._tau_baseline
                       if self._tau_baseline_ok
                       else np.zeros(N_JOINTS))
            row = (
                [f"{now:.6f}", self._collision_event_id, trigger,
                 self._force_raw]
                + [f"{v:.6f}" for v in self.q_slave]
                + [f"{v:.6f}" for v in self._tau_slave]
                + [f"{v:.6f}" for v in tau_ext]
                + [f"{v:.6f}" for v in self.q_master]
                + [f"{v:.6f}" for v in self._tau_master]
                + [f"{v:.6f}" for v in reflected_tau]
            )
            self._collision_csv_writer.writerow(row)
            self._collision_csv_file.flush()
        except Exception:
            pass

    # ── Hilo de gestión de bloqueo del maestro ───────────
    def _master_lock_loop(self):
        """Hilo dedicado que bloquea/desbloquea el maestro en función de colisión.
        Cuando hay colisión en el esclavo → maestro en modo posición (hold).
        Cuando se libera → maestro vuelve a teach mode.
        Esto evita el cycling de modos en el loop de control."""
        while rclpy.ok():
            try:
                req = self._master_lock_req
                if req and not self._master_locked:
                    self._do_master_hold()
                elif not req and self._master_locked:
                    self._do_master_teach()
            except Exception:
                pass
            time.sleep(0.05)  # 20 Hz — fuera del loop ROS

    def _do_master_hold(self):
        """Bloquea el maestro en posición actual (mode 0 = posición).
        El operador siente resistencia física al intentar mover el brazo."""
        if self._master_arm is None:
            return
        try:
            code, angles = self._master_arm.get_servo_angle(is_radian=True)
            if code != 0:
                return
            cur_deg = [float(a * 180.0 / np.pi) for a in angles[:N_JOINTS]]
            self._master_arm.set_mode(0)
            self._master_arm.set_state(0)
            time.sleep(0.05)
            self._master_arm.set_servo_angle(
                angle=cur_deg, speed=30, wait=False)
            self._master_locked = True
            self.get_logger().warn(
                "🔒 Maestro bloqueado (colisión en esclavo)")
        except Exception as e:
            self.get_logger().warn(
                f"Error bloqueando maestro: {e}", throttle_duration_sec=2.0)

    def _do_master_teach(self):
        """Devuelve el maestro a teach mode (operador puede moverlo libremente)."""
        if self._master_arm is None:
            return
        try:
            self._master_arm.set_mode(2)
            self._master_arm.set_state(0)
            self._master_locked = False
            self.get_logger().info(
                "🔓 Maestro desbloqueado (colisión liberada)")
        except Exception as e:
            self.get_logger().warn(
                f"Error desbloqueando maestro: {e}", throttle_duration_sec=2.0)

    def _declare_and_load_params(self):
        self.declare_parameter("traj_time_ms",     60)
        self.declare_parameter("still_thr_deg",    0.1)
        self.declare_parameter("still_cycles",     3)
        self.declare_parameter("force_threshold",  3900)
        self.declare_parameter("force_hysteresis", 3950)
        self.declare_parameter("calib_samples",    80)
        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("torque_gain",      0.15)
        self.declare_parameter("perturbation_thr_deg", 2.0)  # umbral para detectar empujón

        self.traj_time_ms  = self.get_parameter("traj_time_ms").value
        self.still_thr_deg = self.get_parameter("still_thr_deg").value
        self.still_thr     = np.deg2rad(self.still_thr_deg)
        self.still_cycles  = self.get_parameter("still_cycles").value
        self.force_thr     = self.get_parameter("force_threshold").value
        self.force_hys     = self.get_parameter("force_hysteresis").value
        self.calib_n       = self.get_parameter("calib_samples").value
        self.wdog_t        = self.get_parameter("watchdog_timeout").value
        self.torque_gain   = self.get_parameter("torque_gain").value
        self.perturb_thr   = np.deg2rad(
            self.get_parameter("perturbation_thr_deg").value)

    def _init_master(self):
        """Conecta al maestro, lo lleva a HOME y lo deja en teach mode."""
        self.get_logger().info("🏠 Moviendo maestro a HOME...")
        HOME_DEG = [round(r * 180.0 / np.pi, 4) for r in HOME_RAD]
        try:
            from xarm.wrapper import XArmAPI
            arm = XArmAPI('192.168.1.167')
            arm.motion_enable(enable=True)
            arm.set_mode(0)
            arm.set_state(0)
            arm.set_servo_angle(angle=HOME_DEG, speed=25, wait=True)
            arm.set_mode(2)   # teach mode — operador puede mover a mano
            arm.set_state(0)
            self._master_arm = arm   # mantener conexión abierta
            self.get_logger().info("✅ Maestro en HOME (teach mode)")
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"⚠ HOME maestro falló: {e} — continuando")

    def _poll_master(self):
        """Lee posiciones del maestro vía SDK y publica en /master/joint_states."""
        if self._master_arm is None:
            return
        try:
            code, angles = self._master_arm.get_servo_angle(is_radian=True)
            if code != 0:
                return
            pos = angles[:N_JOINTS]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = [float(a) for a in pos]
            self._pub_master_js.publish(msg)
        except Exception as e:
            self.get_logger().warn(
                f"⚠ Error leyendo maestro: {e}", throttle_duration_sec=2.0)

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

        # Extraer esfuerzos del esclavo
        if len(msg.effort) >= N_JOINTS:
            name_map = dict(zip(msg.name, range(len(msg.name))))
            try:
                indices = [name_map[j] for j in JOINT_NAMES]
                self._tau_slave = np.array([msg.effort[i] for i in indices])
            except KeyError:
                pass

        if self.state == TeleopState.CALIBRATING:
            self._buf_s.append(pos.copy())
            # También acumular esfuerzos para calibración baseline
            if np.any(self._tau_slave != 0.0):
                self._buf_tau.append(self._tau_slave.copy())

    def _cb_force(self, msg: Int32):
        self._fwin.append(msg.data)
        self._force_raw = msg.data
        self._last_force_t = self.get_clock().now()
        self._force_ever_received = True
        if len(self._fwin) < 3:
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

    def _cb_collision(self, msg: Bool):
        """Cuando el nodo de colisión detecta contacto, pausar teleoperación."""
        self._collision_active = msg.data

    def _try_calibrate(self):
        n = min(len(self._buf_m), len(self._buf_s))
        if n < self.calib_n:
            if n % 20 == 0:
                self.get_logger().info(f"Calibrando... {n}/{self.calib_n} muestras")
            return

        arr_m = np.array(self._buf_m[-self.calib_n:])
        arr_s = np.array(self._buf_s[-self.calib_n:])

        if (arr_m.std(axis=0) > CALIB_STD_MAX).any() or \
           (arr_s.std(axis=0) > CALIB_STD_MAX).any():
            self.get_logger().warn(
                f"⚠ Robots moviéndose durante calibración. Reintentando...\n"
                f"  std_m={np.round(arr_m.std(axis=0), 4)}\n"
                f"  std_s={np.round(arr_s.std(axis=0), 4)}")
            self._buf_m.clear()
            self._buf_s.clear()
            return

        self.offset = arr_s.mean(axis=0) - arr_m.mean(axis=0)
        self._q_master_prev = self.q_master.copy()
        self._still_count   = 0
        self._last_force_t  = self.get_clock().now()

        # Calibrar baseline de esfuerzos (gravedad en reposo)
        if len(self._buf_tau) >= self.calib_n:
            arr_tau = np.array(self._buf_tau[-self.calib_n:])
            self._tau_baseline = arr_tau.mean(axis=0)
            self._tau_baseline_ok = True
            self.get_logger().info(
                f"   tau_baseline = {np.round(self._tau_baseline, 3)} N·m")
        elif len(self._buf_tau) > 10:
            arr_tau = np.array(self._buf_tau)
            self._tau_baseline = arr_tau.mean(axis=0)
            self._tau_baseline_ok = True
            self.get_logger().warn(
                f"   tau_baseline (parcial, {len(self._buf_tau)} muestras) = "
                f"{np.round(self._tau_baseline, 3)} N·m")
        else:
            self.get_logger().warn(
                "   ⚠ Sin datos de esfuerzo suficientes — "
                "haptic por esfuerzo desactivado")

        self.state = TeleopState.RUNNING

        self.get_logger().info(
            f"✅ Calibración OK\n"
            f"   offset = {np.round(self.offset, 4)} rad\n"
            f"   std_m  = {np.round(arr_m.std(axis=0), 5)}\n"
            f"   std_s  = {np.round(arr_s.std(axis=0), 5)}\n"
            f"   effort_baseline = {self._tau_baseline_ok}\n"
            f"   Estado → RUNNING"
        )

    def _enter_force_stop(self):
        self.state = TeleopState.FORCE_STOP
        self._master_lock_req = True   # bloquear maestro al detectar FSR
        if self._s_ready:
            self._pub_hold()
            self._pub_hold()
        self.get_logger().warn("🛑 FORCE STOP – contacto detectado")

    def _exit_force_stop(self):
        self.state = TeleopState.RUNNING
        self._still_count = 0
        self._master_lock_req = False  # liberar maestro
        self.get_logger().info("▶ FORCE STOP liberado – reanudando")

    def _control_loop(self):
        with self._lock:
            st = self.state

        if st != TeleopState.RUNNING:
            return
        if not (self._m_ready and self._s_ready):
            return

        # Si el nodo de colisión está activo, él controla trayectorias y torques
        if self._collision_active:
            return

        q_des = self.q_master + self.offset
        q_err = q_des - self.q_slave

        # ── Detección de colisión por esfuerzo ──────────────
        tau_ext = np.zeros(N_JOINTS)
        if self._tau_baseline_ok:
            tau_ext = self._tau_slave - self._tau_baseline
            collision_joints = np.abs(tau_ext) > EFFORT_THR
            self._effort_collision_win.append(np.any(collision_joints))

            if (len(self._effort_collision_win) >= COLLISION_CONFIRM and
                    all(self._effort_collision_win)):
                if not self._effort_collision_active:
                    self._effort_collision_active = True
                    self._master_lock_req = True   # ← bloquear maestro
                    self._collision_event_id += 1
                    self.get_logger().warn(
                        f"⚡ COLISIÓN POR ESFUERZO detectada (evento #{self._collision_event_id})\n"
                        f"   tau_ext = {np.round(tau_ext, 3)} N·m")
            else:
                if self._effort_collision_active:
                    self._effort_collision_active = False
                    self._master_lock_req = False   # ← liberar maestro
                    self.get_logger().info(
                        "✅ Colisión por esfuerzo liberada")

        # ── Si hay colisión por esfuerzo: hold + haptic fuerte ─
        if self._effort_collision_active:
            self._pub_hold()
            reflected = self._reflect_effort_collision(tau_ext, q_err)
            self._log_collision_event("effort", reflected)
            return

        # Detectar si el maestro está quieto
        delta = np.max(np.abs(self.q_master - self._q_master_prev))
        self._q_master_prev = self.q_master.copy()

        if delta < self.still_thr:
            self._still_count += 1
        else:
            self._still_count = 0

        if self._still_count >= self.still_cycles:
            self._pub_hold()
            self._reflect_torques(q_err)
            return

        self._pub_traj_point(q_des)
        self._reflect_torques(q_err)

    def _reflect_effort_collision(self, tau_ext: np.ndarray,
                                    q_err: np.ndarray) -> np.ndarray:
        """Refleja fuerzas de colisión reales al maestro (por esfuerzo).
        El maestro ya está bloqueado físicamente por _master_lock_loop."""
        reflected = -HAPTIC_GAIN * tau_ext
        reflected = np.clip(reflected, -HAPTIC_MAX, HAPTIC_MAX)
        self._last_tau = reflected.copy()

        msg = Float64MultiArray()
        msg.data = reflected.tolist()
        self._pub_torque.publish(msg)

        self.get_logger().info(
            f"⚡ Haptic esfuerzo τ = {np.round(reflected, 3)} N·m",
            throttle_duration_sec=0.3)

        # Log de experimento
        self._log_experiment(q_err)
        return reflected

    def _reflect_torques(self, q_err: np.ndarray):
        if self._in_contact:
            tau = -self.torque_gain * q_err * 3.0
        else:
            tau = -self.torque_gain * 0.05 * q_err
        tau = np.clip(tau, -2.0, 2.0)
        self._last_tau = tau.copy()
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub_torque.publish(msg)

        # Logging
        self._log_experiment(q_err)

        # Contacto FSR: publicar torques reflejados y loguear
        if self._in_contact:
            self.get_logger().info(
                f"Haptic FSR τ = {np.round(tau, 3)} N·m",
                throttle_duration_sec=0.5)
            self._log_collision_event("fsr", tau)

    def _watchdog(self):
        if not self._force_ever_received:
            return
        elapsed = (self.get_clock().now() - self._last_force_t).nanoseconds * 1e-9
        if elapsed > self.wdog_t:
            with self._lock:
                if self.state == TeleopState.RUNNING:
                    self.state = TeleopState.EMERGENCY
                    if self._s_ready:
                        self._pub_hold()
                    self.get_logger().error(
                        f"🚨 EMERGENCY – sin datos de fuerza por {elapsed:.2f}s")

    def _pub_traj_point(self, q_des: np.ndarray):
        msg = JointTrajectory()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.joint_names     = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions        = q_des.tolist()
        pt.velocities       = [0.0] * N_JOINTS
        pt.time_from_start  = Duration(
            sec=0, nanosec=int(self.traj_time_ms * 1_000_000))
        msg.points = [pt]
        self._pub_traj.publish(msg)

    def _pub_hold(self):
        if self._s_ready:
            self._pub_traj_point(self.q_slave.copy())

    def _extract(self, msg: JointState):
        m = dict(zip(msg.name, msg.position))
        try:
            return np.array([m[j] for j in JOINT_NAMES])
        except KeyError as e:
            self.get_logger().warn(
                f"Joint faltante: {e}", throttle_duration_sec=2.0)
            return None


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
        try:
            if node._s_ready:
                node._pub_hold()
        except Exception:
            pass
        if node._master_arm is not None:
            try:
                node._master_arm.disconnect()
            except Exception:
                pass
        try:
            node._csv_file.close()
            node._collision_csv_file.close()
            node.get_logger().info(
                f"Experiment saved: {node._csv_path}\n"
                f"Collisions saved: {node._collision_csv_path}\n"
                f"Total collision events: {node._collision_event_id}")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()