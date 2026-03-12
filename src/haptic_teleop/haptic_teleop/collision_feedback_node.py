#!/usr/bin/env python3
"""
haptic_teleop/collision_feedback_node.py
=========================================
Nodo de retroalimentación de colisión para teleoperación bilateral.

Detecta colisiones en el esclavo usando los esfuerzos (effort) de sus juntas.
Cuando el esclavo colisiona con un objeto:
  1. Detiene al esclavo (hold en posición actual)
  2. Calcula las fuerzas de colisión (effort - baseline)
  3. Refleja esas fuerzas al maestro vía /master/reflected_torques
     → el operador siente la colisión como si el maestro hubiera chocado

Flujo:
  CALIBRATING → recoge N muestras de esfuerzo en reposo (baseline)
  MONITORING  → compara esfuerzos actuales vs baseline
  COLLISION   → hold esclavo + reflejar fuerzas al maestro

Autor : Jose Luis Dominguez – ITESM Robótica
Uso   : ros2 run haptic_teleop collision_feedback_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
from collections import deque
from enum import Enum, auto
import threading

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
N_JOINTS = len(JOINT_NAMES)
RATE_HZ = 50.0
DT = 1.0 / RATE_HZ


class CollisionState(Enum):
    CALIBRATING = auto()
    MONITORING = auto()
    COLLISION = auto()


class CollisionFeedbackNode(Node):

    def __init__(self):
        super().__init__("collision_feedback_node")
        self._declare_and_load_params()

        self.state = CollisionState.CALIBRATING
        self._lock = threading.Lock()

        # Estado del esclavo
        self.q_slave = np.zeros(N_JOINTS)
        self.effort_slave = np.zeros(N_JOINTS)
        self._s_ready = False

        # Calibración de baseline de esfuerzos
        self._effort_buf = []
        self._baseline = np.zeros(N_JOINTS)
        self._baseline_std = np.zeros(N_JOINTS)

        # Ventana de detección de colisión
        self._effort_win = deque(maxlen=self.collision_samples)

        # QoS
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # Suscriptores
        self.create_subscription(
            JointState, "/joint_states",
            self._cb_slave, qos_be)

        # Publicadores
        self._pub_torque = self.create_publisher(
            Float64MultiArray,
            "/master/reflected_torques",
            qos_rel)

        self._pub_traj = self.create_publisher(
            JointTrajectory,
            "/lite6_traj_controller/joint_trajectory",
            qos_rel)

        self._pub_collision = self.create_publisher(
            Bool,
            "/slave/collision_active",
            qos_rel)

        # Timer de control
        self.create_timer(DT, self._control_loop)

        self.get_logger().info(
            f"collision_feedback_node iniciado | estado: CALIBRATING\n"
            f"   effort_threshold={self.effort_threshold} N*m  "
            f"calib_samples={self.calib_n}  "
            f"reflection_gain={self.reflection_gain}")

    def _declare_and_load_params(self):
        self.declare_parameter("effort_threshold", 3.0)
        self.declare_parameter("calib_samples", 100)
        self.declare_parameter("collision_samples", 3)
        self.declare_parameter("reflection_gain", 0.5)
        self.declare_parameter("max_reflection_torque", 2.0)
        self.declare_parameter("traj_time_ms", 60)
        self.declare_parameter("release_factor", 0.5)

        self.effort_threshold = self.get_parameter("effort_threshold").value
        self.calib_n = self.get_parameter("calib_samples").value
        self.collision_samples = self.get_parameter("collision_samples").value
        self.reflection_gain = self.get_parameter("reflection_gain").value
        self.max_torque = self.get_parameter("max_reflection_torque").value
        self.traj_time_ms = self.get_parameter("traj_time_ms").value
        self.release_factor = self.get_parameter("release_factor").value

    def _cb_slave(self, msg: JointState):
        """Callback de joint_states del esclavo: extrae posiciones y esfuerzos."""
        name_map = dict(zip(msg.name, range(len(msg.name))))
        try:
            indices = [name_map[j] for j in JOINT_NAMES]
        except KeyError as e:
            self.get_logger().warn(
                f"Joint faltante: {e}", throttle_duration_sec=2.0)
            return

        self.q_slave = np.array([msg.position[i] for i in indices])

        if len(msg.effort) >= N_JOINTS:
            self.effort_slave = np.array([msg.effort[i] for i in indices])
        else:
            return

        self._s_ready = True

        # Acumular muestras de calibración
        if self.state == CollisionState.CALIBRATING:
            self._effort_buf.append(self.effort_slave.copy())
            self._try_calibrate()

    def _try_calibrate(self):
        """Calibra el baseline de esfuerzos cuando hay suficientes muestras."""
        n = len(self._effort_buf)
        if n < self.calib_n:
            if n % 25 == 0:
                self.get_logger().info(
                    f"Calibrando esfuerzos... {n}/{self.calib_n} muestras")
            return

        arr = np.array(self._effort_buf[-self.calib_n:])
        self._baseline = arr.mean(axis=0)
        self._baseline_std = arr.std(axis=0)

        # Umbral adaptativo: baseline + max(threshold, 3*std)
        self._adaptive_thr = np.maximum(
            self.effort_threshold,
            3.0 * self._baseline_std)

        with self._lock:
            self.state = CollisionState.MONITORING

        self.get_logger().info(
            f"Calibracion de esfuerzos OK\n"
            f"   baseline = {np.round(self._baseline, 3)} N*m\n"
            f"   std      = {np.round(self._baseline_std, 4)} N*m\n"
            f"   umbral   = {np.round(self._adaptive_thr, 3)} N*m\n"
            f"   Estado -> MONITORING")

    def _control_loop(self):
        """Loop principal: detecta colisión y refleja fuerzas."""
        if not self._s_ready:
            return

        with self._lock:
            st = self.state

        if st == CollisionState.CALIBRATING:
            return

        # Calcular delta de esfuerzos respecto al baseline
        effort_delta = self.effort_slave - self._baseline

        # Magnitud del delta por junta
        abs_delta = np.abs(effort_delta)

        # Verificar si alguna junta excede el umbral adaptativo
        collision_detected = np.any(abs_delta > self._adaptive_thr)

        # Ventana de confirmación
        self._effort_win.append(collision_detected)

        if st == CollisionState.MONITORING:
            # Necesitamos N muestras consecutivas confirmando colisión
            if len(self._effort_win) >= self.collision_samples and \
               all(self._effort_win):
                self._enter_collision(effort_delta)

        elif st == CollisionState.COLLISION:
            # Seguir reflejando fuerzas mientras hay colisión
            self._reflect_collision_forces(effort_delta)

            # Mantener esclavo en posición
            self._pub_hold()

            # Verificar si la colisión se liberó
            release_thr = self._adaptive_thr * self.release_factor
            if np.all(abs_delta < release_thr):
                self._exit_collision()

        # Publicar estado de colisión
        col_msg = Bool()
        col_msg.data = (st == CollisionState.COLLISION)
        self._pub_collision.publish(col_msg)

    def _enter_collision(self, effort_delta: np.ndarray):
        """Transición a estado de colisión."""
        with self._lock:
            self.state = CollisionState.COLLISION

        # Detener esclavo inmediatamente
        self._pub_hold()
        self._pub_hold()

        # Reflejar fuerzas iniciales
        self._reflect_collision_forces(effort_delta)

        self.get_logger().warn(
            f"COLISION DETECTADA\n"
            f"   effort_delta = {np.round(effort_delta, 3)} N*m\n"
            f"   Reflejando fuerzas al maestro...")

    def _exit_collision(self):
        """Libera el estado de colisión."""
        with self._lock:
            self.state = CollisionState.MONITORING

        self._effort_win.clear()

        # Enviar torque cero al maestro
        msg = Float64MultiArray()
        msg.data = [0.0] * N_JOINTS
        self._pub_torque.publish(msg)

        self.get_logger().info("Colision liberada - reanudando monitoreo")

    def _reflect_collision_forces(self, effort_delta: np.ndarray):
        """
        Refleja las fuerzas de colisión al maestro.
        El signo se invierte para que el operador sienta resistencia
        en la dirección opuesta al movimiento que causó la colisión.
        """
        # tau_reflected = -gain * effort_delta
        tau = -self.reflection_gain * effort_delta
        tau = np.clip(tau, -self.max_torque, self.max_torque)

        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub_torque.publish(msg)

        self.get_logger().info(
            f"Haptic colision: tau = {np.round(tau, 3)} N*m",
            throttle_duration_sec=0.3)

    def _pub_hold(self):
        """Publica trayectoria de hold: mantener posición actual del esclavo."""
        if not self._s_ready:
            return
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = self.q_slave.tolist()
        pt.velocities = [0.0] * N_JOINTS
        pt.time_from_start = Duration(
            sec=0, nanosec=int(self.traj_time_ms * 1_000_000))
        msg.points = [pt]
        self._pub_traj.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionFeedbackNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
