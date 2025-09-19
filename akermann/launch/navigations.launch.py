import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the path to the package
    ackermann_pkg_path = get_package_share_directory('ackermann')

    robots = ['robot1', 'robot2']  # Define multiple robot names

    rviz_config_path = os.path.join(ackermann_pkg_path, 'config', 'nav2_namespaced_view.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default = os.path.join(ackermann_pkg_path, 'map', 'simple_room.yaml'))
    
    init_pose_path = launch.substitutions.LaunchConfiguration(
        'init_pose', default = os.path.join(ackermann_pkg_path, 'config', 'init_pose_node.yaml'))
    
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'nav2_param_file', default = os.path.join(ackermann_pkg_path, 'config', 'nav2_params.yaml'))
    
    action_init_pose = launch_ros.actions.Node(
            namespace='robot1',  # 这里加命名空间
            package='ackermann',
            executable='init_pose_node',
            name='init_pose_node',
            output='screen',
            parameters = [init_pose_path],
        )
    
    action_send_goal = launch_ros.actions.Node(
            namespace='robot1',  # 这里加命名空间
            package='ackermann',
            executable='init_target_node',
            name='init_target_node',
            output='screen',
            parameters = [init_pose_path],
        )
    

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument("map", default_value= map_yaml_path, description="Full path to map file to load"),
        launch.actions.DeclareLaunchArgument("params_file", default_value= nav2_param_path, description="Full path to the ROS2 parameters file to load"),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ackermann_pkg_path, '/launch', '/bringup_launch.py']),

            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'namespace': 'robot1',
                'use_namespace': 'true',
            }.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path, '--ros-args', '--log-level', 'WARN'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        action_init_pose,
        action_send_goal,
    ])