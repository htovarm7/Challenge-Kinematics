#!/usr/bin/env python3
"""
haptic_teleop/force_stop_node.py
=================================
Nodo de parada de emergencia por sensor de fuerza.
Suscribe /force_sensor y detiene el xArm Lite 6 esclavo via
acción follow_joint_trajectory cuando se detecta contacto.

Autor : Jose Luis Dominguez – ITESM Robótica
Uso   : ros2 run haptic_teleop force_stop_node
"""

import rclpy
from rclpy.node   import Node
from rclpy.qos    import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from std_msgs.msg           import Int32
from trajectory_msgs.msg    import JointTrajectory
from control_msgs.action    import FollowJointTrajectory

from collections import deque

JOINT_NAMES  = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
FORCE_WIN    = 3     # muestras consecutivas para confirmar contacto
FORCE_HYS    = 50    # histéresis de liberación (+offset del umbral)


class ForceStopNode(Node):

    def __init__(self):
        super().__init__("force_stop_node")

        # ── Parámetros ────────────────────────────────────────────
        self.declare_parameter("force_threshold", 3900)
        self.force_thr = self.get_parameter("force_threshold").value
        self.force_hys = self.force_thr + FORCE_HYS

        # ── Estado ────────────────────────────────────────────────
        self._stopped  = False
        self._fwin     = deque(maxlen=FORCE_WIN)

        # ── QoS ───────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Suscriptor de fuerza ──────────────────────────────────
        self.create_subscription(Int32, "/force_sensor", self._cb_force, qos_be)

        # ── Publicador de trayectoria vacía (stop) ────────────────
        self._pub_traj = self.create_publisher(
            JointTrajectory,
            "/xarm_traj_controller/joint_trajectory",
            qos_rel,
        )

        # ── Action client para cancelar trayectorias activas ──────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/xarm_traj_controller/follow_joint_trajectory",
        )

        self.get_logger().info(
            f"✅ force_stop_node activo  |  umbral={self.force_thr}"
        )

    def _cb_force(self, msg: Int32):
        self._fwin.append(msg.data)
        if len(self._fwin) < FORCE_WIN:
            return

        contact  = all(v < self.force_thr for v in self._fwin)
        released = all(v > self.force_hys for v in self._fwin)

        if contact and not self._stopped:
            self._stopped = True
            self._do_stop()

        elif released and self._stopped:
            self._stopped = False
            self.get_logger().info("▶ Fuerza liberada – listo para reanudar")

    def _do_stop(self):
        """
        Detiene el robot de forma inmediata:
          1. Publica trayectoria vacía (interrumpe la actual)
          2. Cancela todos los goals activos del action server
        """
        # Trayectoria vacía → interrupción inmediata del servo
        stop_msg             = JointTrajectory()
        stop_msg.joint_names = JOINT_NAMES
        self._pub_traj.publish(stop_msg)

        # Cancelar goal activo si existe
        if self._action_client.server_is_ready():
            self._action_client._cancel_goal_async  # cancela goal activo

        self.get_logger().warn(
            f"🛑 STOP  – fuerza < {self.force_thr}  (ventana: {list(self._fwin)})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForceStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
