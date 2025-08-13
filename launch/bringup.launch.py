from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('pressure_sensor_hardware_interface')

    udp_port_arg = DeclareLaunchArgument('udp_port', default_value='5005')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg, 'urdf', 'pressure_sensor.urdf.xacro']),
        ' udp_port:=', LaunchConfiguration('udp_port')
    ])

    return LaunchDescription([
        udp_port_arg,
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
    ])
