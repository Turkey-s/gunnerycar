import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot_name').perform(context)
    robot_xyz = LaunchConfiguration('robot_init_xyz').perform(context)

    robot_xyz_value = [str(x) for x in robot_xyz.split()]  # 定义多个机器人的初始位置
    
    urdf_pkg_path = get_package_share_directory('ackermann')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'robot.urdf.xacro')

    groups = []

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
                '-x', robot_xyz_value[0],  # 设置初始位置x坐标
                '-y', robot_xyz_value[1],  # 设置初始位置y坐标
                '-z', robot_xyz_value[2]   # 设置初始位置z坐标
            ],
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': value, 'use_sim_time': True}],
        ),
    ])  
    )
    return groups

def generate_launch_description():
    print("启动文件已加载，准备启动机器人相关节点！")

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Top-level namespace')
    
    declare_robot_xyz_cmd = DeclareLaunchArgument(
        'robot_init_xyz',
        default_value='0.0 0.0 0.1',
        description='Robot initial position in the format "x y z"')

    return launch.LaunchDescription([
        declare_robot_name_cmd,
        declare_robot_xyz_cmd,
        OpaqueFunction(function=launch_setup)
    ])