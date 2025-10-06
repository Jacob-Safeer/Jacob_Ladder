from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('precision_land'),
        'cfg',
        'front_approach_pid_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='precision_land',
            executable='front_approach_pid_precision_land',
            name='front_approach_pid_precision_land',
            output='screen',
            parameters=[params]
        )
    ])
