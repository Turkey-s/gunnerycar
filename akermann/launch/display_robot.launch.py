import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    print("启动文件已加载，准备启动机器人相关节点！")
    robots = ['robot1', 'robot2']  # 定义多个机器人名称
    robots_init_xyz = {'robot1' : ['-10.0', '-5.5', '0.1'], 'robot2' : ['-12.0', '-5.5', '0.1']}  # 定义每个机器人的初始位置
    urdf_pkg_path = get_package_share_directory('ackermann')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'robot.urdf.xacro')
    default_world_config_path = os.path.join(urdf_pkg_path, 'world', 'simple_world.world')

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_world_config_path), ('verbose', 'true')],  # 设置world文件路径和verbose模式
    )

    groups = []

    for robot in robots:
        command = launch.substitutions.Command(['xacro ', default_urdf_path, ' robot_name:=', robot])
        value = launch_ros.parameter_descriptions.ParameterValue(command, value_type=str)

        groups.append( GroupAction([
            PushRosNamespace(robot),
            launch_ros.actions.Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot,
                    '-topic', 'robot_description',
                    '-x', robots_init_xyz[robot][0],  # 设置初始位置x坐标
                    '-y', robots_init_xyz[robot][1],  # 设置初始位置y坐标
                    '-z', robots_init_xyz[robot][2]   # 设置初始位置z坐标
                ],
            ),
            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': value, 'use_sim_time': True}],
            ),
        ])  
        )

    return launch.LaunchDescription([
        action_launch_gazebo,
    ] + groups)