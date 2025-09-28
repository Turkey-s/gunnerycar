import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction, DeclareLaunchArgument,OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def launch_setup(context, *args, **kwargs):
    ackermann_pkg_path = get_package_share_directory('ackermann')
    robot_name = LaunchConfiguration('robot_name').perform(context)

    rviz_config_path = os.path.join(ackermann_pkg_path, 'config', 'nav2_'+robot_name+'_view.rviz')

    use_sim_time = LaunchConfiguration(
        'use_sim_time', default='true')
    
    map_yaml_path = LaunchConfiguration(
        'map', default = os.path.join(ackermann_pkg_path, 'map', 'simple_room.yaml'))
    
    init_pose_path = LaunchConfiguration(
        'init_pose', default = os.path.join(ackermann_pkg_path, 'config', 'init_pose_node.yaml'))
    
    nav2_param_path = os.path.join(ackermann_pkg_path, 'config', 'nav2_'+robot_name+'_params.yaml')

    ret = []
    
    ret.append(launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ackermann_pkg_path, '/launch', '/bringup_launch.py']),

        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
            'namespace': robot_name,
            'use_namespace': 'true',
        }.items(),
    ))

    if robot_name == 'robot1':
        ret.append(launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path, '--ros-args', '--log-level', 'WARN'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ))
    
    ret.append(launch_ros.actions.Node(
        namespace=robot_name,  # 这里加命名空间
        package='ackermann',
        executable='init_pose_node',
        name='init_pose_node',
        output='screen',
        parameters = [init_pose_path, {"robot_name" : robot_name}],
    ))
    return ret

def generate_launch_description():
    # Get the path to the package
    ackermann_pkg_path = get_package_share_directory('ackermann')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value= 'true',
        description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument("map", default_value= os.path.join(ackermann_pkg_path, 'map', 'simple_room.yaml'), description="Full path to map file to load"),
        DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Top-level namespace'),
        OpaqueFunction(function=launch_setup),
    ])