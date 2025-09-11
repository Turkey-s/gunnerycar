import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Get the path to the package
    gunnerycar_pkg_path = get_package_share_directory('ackermann')
    
    bringup_pkg_path = get_package_share_directory('nav2_bringup')

    rviz_config_path = os.path.join(bringup_pkg_path, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default = os.path.join(gunnerycar_pkg_path, 'map', 'simple_room.yaml'))
    
    init_pose_path = launch.substitutions.LaunchConfiguration(
        'init_pose', default = os.path.join(gunnerycar_pkg_path, 'config', 'init_pose_node.yaml'))
    
    follow_path_path = launch.substitutions.LaunchConfiguration(
        'waypoints', default = os.path.join(gunnerycar_pkg_path, 'config', 'follow_path_node.yaml'))
    
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'nav2_param_file', default = os.path.join(gunnerycar_pkg_path, 'config', 'nav2_params.yaml'))
    
    action_init_pose = launch_ros.actions.Node(
            package='gunnerycar',
            executable='init_pose_node',
            name='init_pose_node',
            output='screen',
            parameters = [init_pose_path],
        )
    
    action_send_goal = launch_ros.actions.Node(
            package='gunnerycar',
            executable='follow_path_node',
            name='follow_path_node',
            output='screen',
            parameters = [follow_path_path],
        )
    
    action_speaker = launch_ros.actions.Node(
            package='gunnerycar',
            executable='speaker_service',
            name='speaker_service',
            output='screen',
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
                [bringup_pkg_path, '/launch', '/bringup_launch.py']),

            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        action_init_pose,
        # action_send_goal,
        # action_speaker,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnExecutionComplete(
        #         target_action=action_init_pose,
        #         on_completion=[action_send_goal]
        #     )
        # ),
    ])