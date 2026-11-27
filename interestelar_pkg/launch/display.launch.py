from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('interestelar_pkg')
    rviz_file = os.path.join(pkg_share, 'rviz2', 'rviz.rviz')
    
    xacro_file = PathJoinSubstitution([
        FindPackageShare('interestelar_pkg'),
        'URDF',
        'interestelar.urdf.xacro'
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                                      description='Puerto serie del PIC16F18446'),
        DeclareLaunchArgument('baud', default_value='9600'),
        DeclareLaunchArgument('gui', default_value='true',
                                description='Abrir la interfaz Qt'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command([
                    'xacro ', xacro_file
                ]),
                value_type=str
            )}]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package='rviz2',
            executable='rviz2', 
            arguments=['-d', rviz_file],
            output='screen'
        ),
        Node(package='interestelar_pkg', executable='n2yo_connect',
             name='n2yo_api', output='screen'),
        
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

        Node(package='interestelar_pkg', executable='joints_pub',
                     name='joints_pub', output='screen'),
        
        
    ])