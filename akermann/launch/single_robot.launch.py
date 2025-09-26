from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 第一个launch文件
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ackermann'),
                'launch',
                'display_robot.launch.py'
            ])
        ])
    )
    
    # 第二个launch文件，延迟5秒启动
    navigations_launch = TimerAction(
        period=10.0,  # 延迟10秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ackermann'),
                        'launch',
                        'navigations.launch.py'
                    ])
                ])
            )
        ]
    )
    
    return LaunchDescription([
        display_launch,
        navigations_launch
    ])