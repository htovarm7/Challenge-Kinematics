#!/usr/bin/env python3
"""
Nodo ROS2 que monitorea el sensor de fuerza y detiene el robot xarm6
cuando la lectura cae por debajo del umbral (contacto detectado).

Uso:
  ros2 run --prefix 'python3' force_stop_node force_stop_node.py
  o simplemente:
  python3 force_stop_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


FORCE_THRESHOLD = 3900
CONTROLLER_ACTION = '/xarm6_traj_controller/follow_joint_trajectory'


class ForceStopNode(Node):
    def __init__(self):
        super().__init__('force_stop_node')

        # Parametros configurables
        self.declare_parameter('force_threshold', FORCE_THRESHOLD)
        self.declare_parameter('controller_action', CONTROLLER_ACTION)

        self.threshold = self.get_parameter('force_threshold').value
        action_name = self.get_parameter('controller_action').value

        # Subscriber al sensor de fuerza
        self.force_sub = self.create_subscription(
            Int32,
            '/force_sensor',
            self.force_callback,
            10
        )

        # Action client para cancelar trayectorias
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name
        )

        self.stopped = False
        self.last_force = None

        self.get_logger().info(
            f'Force stop monitor activo — umbral: {self.threshold}, '
            f'action: {action_name}'
        )

    def force_callback(self, msg: Int32):
        self.last_force = msg.data

        if msg.data < self.threshold and not self.stopped:
            self.get_logger().warn(
                f'Fuerza detectada: {msg.data} < {self.threshold} — deteniendo robot!'
            )
            self.stop_robot()
            self.stopped = True

        elif msg.data >= self.threshold and self.stopped:
            self.get_logger().info(
                f'Fuerza liberada: {msg.data} >= {self.threshold} — listo para moverse'
            )
            self.stopped = False

    def stop_robot(self):
        """Cancela cualquier goal activo y envia trayectoria vacia para frenar."""
        # Cancelar todos los goals activos
        if self.action_client.server_is_ready():
            self.get_logger().info('Cancelando goals activos...')
            cancel_future = self.action_client._cancel_goal_async(None)
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().warn(
                'Action server no disponible — intentando enviar trayectoria vacia'
            )

        # Enviar trayectoria vacia como respaldo para detener el movimiento
        self.send_empty_trajectory()

    def cancel_done_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Goals cancelados: {result}')
        except Exception as e:
            self.get_logger().error(f'Error cancelando goals: {e}')

    def send_empty_trajectory(self):
        """Envia una trayectoria vacia para detener el robot inmediatamente."""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server no disponible para enviar stop')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        # Trayectoria vacia = detenerse
        goal.trajectory.points = []

        self.action_client.send_goal_async(goal)
        self.get_logger().info('Trayectoria vacia enviada — robot detenido')


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


if __name__ == '__main__':
    main()
