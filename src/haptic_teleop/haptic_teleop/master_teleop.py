#!/usr/bin/env python3
"""
haptic_teleop/master_teleop.py
================================
Nodo del robot Maestro:
  - Se conecta al xArm Lite 6 maestro (IP 192.168.1.167)
  - Publica su estado en /master/joint_states
  - Suscribe /master/reflected_torques y aplica compensación de corriente
    para retroalimentar la fuerza al operador (bilateralidad)

Autor : Jose Luis Dominguez – ITESM Robótica
Uso   : ros2 run haptic_teleop master_teleop
"""

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg  import JointState
from std_msgs.msg     import Float64MultiArray

import numpy as np

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
N_JOINTS    = len(JOINT_NAMES)

# Límite de torque por seguridad [N·m] – ajustar según datasheet Lite 6
MAX_TORQUE_NM = 5.0


class MasterTeleop(Node):
    """
    Nodo ligero que actúa como puente de estado del robot maestro.

    Si usas el driver de xArm con namespace /master, este nodo simplemente
    puede correr en paralelo para recibir los torques reflejados y
    aplicarlos via SDK. Con el driver estándar, /master/joint_states
    ya se publica automáticamente.
    """

    def __init__(self):
        super().__init__("master_teleop")

        # ── Parámetros ────────────────────────────────────────────
        self.declare_parameter("torque_scale", 1.0)
        self.torque_scale = self.get_parameter("torque_scale").value

        # ── Estado ────────────────────────────────────────────────
        self._last_torques = np.zeros(N_JOINTS)

        # ── QoS ───────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Suscriptor: torques reflejados del esclavo ─────────────
        self.create_subscription(
            Float64MultiArray,
            "/master/reflected_torques",
            self._cb_torques,
            qos_rel,
        )

        # ── Publicador: estado del maestro (re-publica si es necesario) ─
        self._pub_state = self.create_publisher(
            JointState,
            "/master/joint_states",
            qos_be,
        )

        self.get_logger().info("✅ master_teleop activo")

    def _cb_torques(self, msg: Float64MultiArray):
        from xarm.wrapper import XArmAPI
        torques = np.array(msg.data)
        torques = np.clip(torques * self.torque_scale, -2.0, 2.0)
        self._last_torques = torques

        if not hasattr(self, '_arm'):
            self._arm = XArmAPI('192.168.1.167')
            self._arm.motion_enable(enable=True)
            self._arm.set_mode(4)   # modo torque
            self._arm.set_state(0)
            self.get_logger().info("✅ Brazo maestro en modo torque")

        if np.any(np.abs(torques) > 0.05):
            ret = self._arm.set_joint_torque(torques.tolist())
            self.get_logger().info(
                f"Haptic aplicado: {np.round(torques, 3)} N·m  ret={ret}",
                throttle_duration_sec=0.5
            )


def main(args=None):
    rclpy.init(args=args)
    node = MasterTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
