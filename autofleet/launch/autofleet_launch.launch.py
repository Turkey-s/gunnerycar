import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    autofleet_pkg_path = get_package_share_directory('autofleet')
    
    action_launch_robot = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'ackermann'), '/launch', '/single_robot.launch.py']),
    )

    autofleet_config = launch.substitutions.LaunchConfiguration(
        'config', 
        default = os.path.join(autofleet_pkg_path, 'config', 'autofleet_config.yaml')),

    action_autofleet_node = launch_ros.actions.Node(
        package='autofleet',
        executable='autofleet_node_exec',
        name='autofleet_node',
        output='screen',
        parameters=[autofleet_config],
    )


    return launch.LaunchDescription([
        action_launch_robot,
        action_autofleet_node,
    ])

    