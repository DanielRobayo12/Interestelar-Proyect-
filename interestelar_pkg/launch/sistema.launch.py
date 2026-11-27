"""Lanza todo el sistema: API N2YO -> ROS 2 -> PIC (serial) + interfaz Qt.

Uso:
    ros2 launch interestelar_pkg sistema.launch.py port:=/dev/ttyUSB0
    ros2 launch interestelar_pkg sistema.launch.py gui:=false   # solo nodos ROS
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Puerto serie del PIC16F18446'),
        DeclareLaunchArgument('baud', default_value='9600'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la interfaz Qt'),

        # Node(package='interestelar_pkg', executable='n2yo_connect',
        #      name='n2yo_api', output='screen'),

        Node(package='interestelar_pkg', executable='uart_bridge',
             name='serial_bridge', output='screen',
             parameters=[{
                 'port': LaunchConfiguration('port'),
                 'baud': LaunchConfiguration('baud'),
                 'source': 'look_angles',
             }]),

        Node(package='interestelar_pkg', executable='app',
             name='interestelar_gui', output='screen',
             condition=IfCondition(LaunchConfiguration('gui'))),
    ])
