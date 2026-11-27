"""Visualizacion en RViz: el modelo sigue /satelite/look_angles.

Uso:
    ros2 launch interestelar_pkg rviz_satelite.launch.py
    ros2 launch interestelar_pkg rviz_satelite.launch.py n2yo:=true   # tambien lanza la API

No se usa joint_state_publisher_gui: joints_pub ya publica /joint_states y
dos publicadores a la vez harian parpadear el modelo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('interestelar_pkg')
    rviz_file = os.path.join(pkg_share, 'rviz2', 'rviz.rviz')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('interestelar_pkg'), 'URDF', 'interestelar.urdf.xacro'])

    return LaunchDescription([
        DeclareLaunchArgument('n2yo', default_value='false',
                              description='Lanzar tambien n2yo_connect'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', xacro_file]), value_type=str)}]),

        Node(package='interestelar_pkg', executable='joints_pub',
             name='joints_pub', output='screen'),

        Node(package='interestelar_pkg', executable='n2yo_connect',
             name='n2yo_api', output='screen',
             condition=IfCondition(LaunchConfiguration('n2yo'))),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_file], output='screen'),
    ])
