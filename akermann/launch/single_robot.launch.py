from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

def generate_launch_description():
    ackermann_pkg_path = get_package_share_directory('ackermann')
    
    config_path = os.path.join(ackermann_pkg_path, 'config', 'gazebo_pose.yaml')
    # 读取YAML配置文件
    with open(config_path, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)

    launch_group = []
    for(robot_name) in data['robots_name']:
        print(robot_name, ' | ', data[robot_name])
        launch_group.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ackermann'),
                        'launch',
                        'display_robot.launch.py'
                    ])
                ]),
                launch_arguments={
                    'robot_name': robot_name,
                    'robot_init_xyz': data[robot_name]
                }.items()
            )
        )

        launch_group.append(
            TimerAction(
                period=10.0,  # 延迟10秒
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([
                                FindPackageShare('ackermann'),
                                'launch',
                                'navigations.launch.py'
                            ])
                        ]),
                        launch_arguments={
                            'robot_name': robot_name
                        }.items()
                    )
                ]
            )
        )

    default_world_config_path = os.path.join(ackermann_pkg_path, 'world', 'simple_world.world')
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_world_config_path), ('verbose', 'true')],  # 设置world文件路径和verbose模式
    )
    
    return LaunchDescription([
        action_launch_gazebo,
    ]+launch_group)