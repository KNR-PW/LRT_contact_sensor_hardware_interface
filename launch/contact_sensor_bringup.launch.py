from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('contact_sensor_hardware_interface')

    urdf_default = PathJoinSubstitution([pkg_share, 'urdf', 'contact_sensor.urdf.xacro'])

    urdf = DeclareLaunchArgument(
        'urdf',
        default_value=urdf_default,
        description='Ścieżka do pliku URDF/Xacro.'
    )

    bind_port = DeclareLaunchArgument(
        'bind_port',
        default_value='5005',
        description='Port UDP nasłuchu dla ContactSensor.'
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        LaunchConfiguration('urdf'), ' ',
        'bind_port:=', LaunchConfiguration('bind_port')
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description,
                    PathJoinSubstitution([pkg_share, 'config', 'contact_sensor_controllers.yaml'])],
        output='screen',
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        urdf,
        bind_port,
        controller_manager_node,
        joint_state_broadcaster_spawner,
    ])