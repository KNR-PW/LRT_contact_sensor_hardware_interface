from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('contact_sensor_hardware_interface')
    
    urdf_default = PathJoinSubstitution([pkg_share, 'urdf', 'contact_sensor.urdf.xacro'])
    urdf_arg = DeclareLaunchArgument('urdf', default_value=urdf_default)
    bind_port_arg = DeclareLaunchArgument('bind_port', default_value='5005')
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        LaunchConfiguration('urdf'), ' ',
        'bind_port:=', LaunchConfiguration('bind_port')
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            PathJoinSubstitution([pkg_share, 'config', 'contact_sensor_controllers.yaml'])
        ],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    contact_sensors_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['contact_sensors_broadcaster', '--controller-manager', '/controller_manager'],
    )

    delayed_contact_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[contact_sensors_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        urdf_arg,
        bind_port_arg,
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delayed_contact_spawner,
    ])