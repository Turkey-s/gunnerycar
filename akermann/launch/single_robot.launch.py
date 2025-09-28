from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 第一个launch文件
    #robots_init_xyz = {'robot1' : ['-10.0', '-5.5', '0.1'], 'robot2' : ['-12.0', '-5.5', '0.1']}  # 定义每个机器人的初始位置
    display_launch_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ackermann'),
                'launch',
                'display_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'robot1',
            'robot_init_xyz': '-10.0 -5.5 0.1'
        }.items()
    )

    display_launch_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ackermann'),
                'launch',
                'display_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'robot2',
            'robot_init_xyz': '-10.0 -5.5 0.1'
        }.items()
    )
    
    # 第二个launch文件，延迟5秒启动
    navigations_launch_robot1 = TimerAction(
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
                    'robot_name': 'robot1'
                }.items()
            )
        ]
    )

    # 第二个launch文件，延迟5秒启动
    navigations_launch_robot2 = TimerAction(
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
                    'robot_name': 'robot2'
                }.items()
            )
        ]
    )

    urdf_pkg_path = get_package_share_directory('ackermann')
    default_world_config_path = os.path.join(urdf_pkg_path, 'world', 'simple_world.world')
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_world_config_path), ('verbose', 'true')],  # 设置world文件路径和verbose模式
    )
    
    return LaunchDescription([
        action_launch_gazebo,
        # display_launch_robot1,
        display_launch_robot2,
        # navigations_launch_robot1,
        navigations_launch_robot2,
    ])