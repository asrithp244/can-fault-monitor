from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    interface_arg = DeclareLaunchArgument(
        'interface', default_value='vcan0',
        description='CAN interface. Use "can0" for physical hardware.')

    params_file = PathJoinSubstitution([
        FindPackageShare('can_fault_monitor'), 'config', 'params.yaml'])

    iface = LaunchConfiguration('interface')

    def make_node(executable, name):
        return Node(
            package='can_fault_monitor',
            executable=executable,
            name=name,
            output='screen',
            parameters=[
                params_file,
                {'interface': iface}
            ],
        )

    return LaunchDescription([
        interface_arg,
        make_node('can_publisher',      'can_publisher'),
        make_node('can_subscriber',     'can_subscriber'),
        make_node('fault_injector',     'fault_injector'),
        make_node('can_health_monitor', 'can_health_monitor'),
    ])
