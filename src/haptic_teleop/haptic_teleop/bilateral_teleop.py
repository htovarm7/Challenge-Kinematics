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
import threading
import time

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
N_JOINTS    = len(JOINT_NAMES)
RATE_HZ     = 50.0
DT          = 1.0 / RATE_HZ
CALIB_STD_MAX = 0.017

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
        self._in_contact   = False
        self._collision_active = False

        self._go_home_master()

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

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

        self.create_timer(DT,  self._control_loop)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            "✅ bilateral_teleop iniciado  |  estado: CALIBRATING\n"
            f"   traj_time={self.traj_time_ms}ms  "
            f"still_thr={self.still_thr_deg}°  "
            f"torque_gain={self.torque_gain}"
        )

    def _declare_and_load_params(self):
        self.declare_parameter("traj_time_ms",     60)
        self.declare_parameter("still_thr_deg",    0.1)
        self.declare_parameter("still_cycles",     3)
        self.declare_parameter("force_threshold",  3900)
        self.declare_parameter("force_hysteresis", 3950)
        self.declare_parameter("calib_samples",    80)
        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("torque_gain",      0.15)

        self.traj_time_ms  = self.get_parameter("traj_time_ms").value
        self.still_thr_deg = self.get_parameter("still_thr_deg").value
        self.still_thr     = np.deg2rad(self.still_thr_deg)
        self.still_cycles  = self.get_parameter("still_cycles").value
        self.force_thr     = self.get_parameter("force_threshold").value
        self.force_hys     = self.get_parameter("force_hysteresis").value
        self.calib_n       = self.get_parameter("calib_samples").value
        self.wdog_t        = self.get_parameter("watchdog_timeout").value
        self.torque_gain   = self.get_parameter("torque_gain").value

    def _go_home_master(self):
        self.get_logger().info("🏠 Moviendo maestro a HOME...")
        HOME_DEG = [round(r * 180.0 / np.pi, 4) for r in HOME_RAD]
        try:
            from xarm.wrapper import XArmAPI
            arm = XArmAPI('192.168.1.167')
            arm.motion_enable(enable=True)
            arm.set_mode(0)
            arm.set_state(0)
            arm.set_servo_angle(angle=HOME_DEG, speed=25, wait=True)
            arm.disconnect()
            self.get_logger().info("✅ Maestro en HOME")
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"⚠ HOME maestro falló: {e} — continuando")

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
        self.state = TeleopState.RUNNING

        self.get_logger().info(
            f"✅ Calibración OK\n"
            f"   offset = {np.round(self.offset, 4)} rad\n"
            f"   std_m  = {np.round(arr_m.std(axis=0), 5)}\n"
            f"   std_s  = {np.round(arr_s.std(axis=0), 5)}\n"
            f"   Estado → RUNNING"
        )

    def _enter_force_stop(self):
        self.state = TeleopState.FORCE_STOP
        if self._s_ready:
            self._pub_hold()
            self._pub_hold()
        self.get_logger().warn("🛑 FORCE STOP – contacto detectado")

    def _exit_force_stop(self):
        self.state = TeleopState.RUNNING
        self._still_count = 0
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

    def _reflect_torques(self, q_err: np.ndarray):
        if self._in_contact:
            tau = -self.torque_gain * q_err * 3.0
        else:
            tau = -self.torque_gain * 0.05 * q_err
        tau = np.clip(tau, -2.0, 2.0)
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub_torque.publish(msg)
        if self._in_contact:
            self.get_logger().info(
                f"Haptic τ = {np.round(tau, 3)} N·m",
                throttle_duration_sec=0.5)

    def _watchdog(self):
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
        if node._s_ready:
            node._pub_hold()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()