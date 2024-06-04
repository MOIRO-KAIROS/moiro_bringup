#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # FOR MOIRO AGV
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    moiro_param_dir = LaunchConfiguration(
        'moiro_param_dir',
        default=os.path.join(get_package_share_directory('moiro_bringup'), 'param', 'moiro.yaml')
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    moiro_agv_node = Node(
        package='moiro_agv_node',
        executable='moiro_agv_ros',
        parameters=[moiro_param_dir],
        arguments=['-i', usb_port],
        output='screen'
    )
    
    # FOR MOIRO ARM
    moveit_config = (
        MoveItConfigsBuilder("moiro", package_name="mycobot_moveit_config")
        .robot_description(file_path="config/moiro.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict()
        ]
    )

    rviz_config_file = os.path.join(get_package_share_directory("mycobot_movegroup"), "launch", "default_move_group.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ]
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description]
    )

    ros2_controllers_path = os.path.join(get_package_share_directory("mycobot_moveit_config"), "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both"
    )

    load_controllers = [
        ExecuteProcess(
            cmd=["ros2 run controller_manager spawner {}".format(controller)],
            shell=True,
            output="screen"
        ) for controller in ["mycobot_arm_controller", "joint_state_broadcaster"]
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('usb_port', default_value=usb_port, description='Connected USB port with OpenCR'),
        DeclareLaunchArgument('moiro_param_dir', default_value=moiro_param_dir, description='Full path to moiro parameter file to load'),
        moiro_agv_node,
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
    ] + load_controllers)