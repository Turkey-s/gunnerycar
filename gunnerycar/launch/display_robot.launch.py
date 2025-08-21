import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions

def generate_launch_description():
    print("启动文件已加载，准备启动机器人相关节点！")
    urdf_pkg_path = get_package_share_directory('gunnerycar')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(urdf_pkg_path, 'config', 'default_rviz_config.rviz')
    
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

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher', # 这是传参给joint_state_publisher的可执行文件
    )

    action_rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path], # 相当于命令行参数 -d
    )

    return launch.LaunchDescription([
        action_declare_robot_description,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz2_node,
    ])