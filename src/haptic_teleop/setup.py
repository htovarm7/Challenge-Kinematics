from setuptools import find_packages, setup
import os
from glob import glob

package_name = "haptic_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Registro del paquete en ament
        ("share/ament_index/resource_index/packages",
            ["resource/haptic_teleop"]),
        (f"share/{package_name}",
            ["package.xml"]),
        # Launch files
        (f"share/{package_name}/launch",
            glob("launch/*.launch.py")),
        # Config / parámetros YAML
        (f"share/{package_name}/config",
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jose Luis Dominguez",
    maintainer_email="jose.dominguez@itesm.mx",
    description="Haptic bilateral teleoperation for xArm Lite 6 – TE3001B ITESM",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run haptic_teleop bilateral_teleop
            "bilateral_teleop = haptic_teleop.bilateral_teleop:main",
            # ros2 run haptic_teleop force_stop_node
            "force_stop_node  = haptic_teleop.force_stop_node:main",
            # ros2 run haptic_teleop master_teleop
            "master_teleop    = haptic_teleop.master_teleop:main",
            # ros2 run haptic_teleop collision_feedback_node
            "collision_feedback_node = haptic_teleop.collision_feedback_node:main",
        ],
    },
)
