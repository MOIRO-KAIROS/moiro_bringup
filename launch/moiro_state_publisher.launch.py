from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('moiro_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'moiro.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    return LaunchDescription([
        gui_arg,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
        ),
        Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
        ),
        Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
        )
    ])