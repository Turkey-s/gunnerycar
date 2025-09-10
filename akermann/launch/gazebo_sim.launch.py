import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    print("启动文件已加载，准备启动机器人相关节点！")
    urdf_pkg_path = get_package_share_directory('ackermann')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'robot.urdf.xacro')
    # default_rviz_config_path = os.path.join(urdf_pkg_path, 'config', 'default_rviz_config.rviz')
    default_world_config_path = os.path.join(urdf_pkg_path, 'world', 'warehouse.world')

    action_declare_robot_description = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value= str(default_urdf_path),
        description='Absolute path to robot urdf file',
    )

    #通过文件读取URDF内容，并传递给robot_state_publisher节点
    command = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    value = launch_ros.parameter_descriptions.ParameterValue(command, value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': value}],
    )

    # action_rviz2_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', default_rviz_config_path], # 相当于命令行参数 -d
    # )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_world_config_path), ('verbose', 'true')],  # 设置world文件路径和verbose模式
    )

    action_load_joint_state_broadcaster = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'gunnerycar_joint_state_broadcaster'],
        output='screen',
    )

    action_load_effort_state_broadcaster = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'gunnerycar_effort_controller'],
        output='screen',
    )

    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'gunnerycar_diff_drive_controller'],
        output='screen',
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'gunnerycar',
            '-topic', 'robot_description',
            '-x', '0.0',  # 设置初始位置x坐标
            '-y', '0.0',  # 设置初始位置y坐标
            '-z', '0.1'   # 设置初始位置z坐标
        ],
    )

    return launch.LaunchDescription([
        action_declare_robot_description,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        # action_twist_transform,
        # action_joint_state_publisher,
        # action_rviz2_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_broadcaster]
            )
        ),

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_broadcaster,
                on_exit=[action_load_diff_drive_controller]
            )
        ),
    ])