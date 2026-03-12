"""
launch/bilateral_teleop.launch.py
===================================
Launch completo del sistema de teleoperación bilateral.
Jose Luis Dominguez – ITESM TE3001B

Uso:
    ros2 launch haptic_teleop bilateral_teleop.launch.py
    ros2 launch haptic_teleop bilateral_teleop.launch.py slave_ip:=192.168.1.175
"""

from launch                             import LaunchDescription
from launch.actions                     import (DeclareLaunchArgument,
                                                TimerAction, LogInfo)
from launch.substitutions               import (LaunchConfiguration,
                                                PathJoinSubstitution)
from launch_ros.actions                 import Node
from launch_ros.substitutions           import FindPackageShare


def generate_launch_description():

    # ── Argumentos ────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("slave_ip",    default_value="192.168.1.175"),
        DeclareLaunchArgument("master_ip",   default_value="192.168.1.167"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("dof",         default_value="6"),
    ]

    params_file = PathJoinSubstitution(
        [FindPackageShare("haptic_teleop"), "config", "teleop_params.yaml"]
    )

    # ── 1. micro-ROS agent (ESP32 sensor de fuerza) ───────────────
    microros = Node(
        package="micro_ros_agent", executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", LaunchConfiguration("serial_port")],
        output="screen",
    )

    # ── 2. Nodo maestro ───────────────────────────────────────────
    master_node = Node(
        package="haptic_teleop", executable="master_teleop",
        name="master_teleop", output="screen",
        parameters=[params_file],
    )

    # ── 3. Nodo force_stop (capa de seguridad independiente) ──────
    force_stop = Node(
        package="haptic_teleop", executable="force_stop_node",
        name="force_stop_node", output="screen",
        parameters=[params_file],
    )

    # ── 4. Nodo de teleoperación bilateral (arranca 3s después) ───
    bilateral = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="🤖 Iniciando BilateralTeleop..."),
            Node(
                package="haptic_teleop", executable="bilateral_teleop",
                name="bilateral_teleop", output="screen",
                parameters=[params_file],
            ),
        ]
    )

    return LaunchDescription(args + [
        microros,
        master_node,
        force_stop,
        bilateral,
    ])
