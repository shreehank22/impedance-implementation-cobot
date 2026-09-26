#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.join(
                get_package_share_directory('cobot_c1_description'),
                'urdf',
                'cobot_c1_description.urdf'),
            description='Path to the robot URDF file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_path',
            default_value='/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf',
            description='Path to the world file to load'
        )
    )

    # Initialize Arguments
    model_path = LaunchConfiguration('model_path')
    world_path = LaunchConfiguration('world_path')

    # Start Ignition Gazebo
    # Start Ignition Gazebo in paused mode
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'cobot_c1',
            '-file', model_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add declared arguments
    for arg in declared_arguments:
        ld.add_action(arg)

    # Add the actions to launch all nodes
    ld.add_action(ignition_gazebo)
    ld.add_action(spawn_entity)

    return ld

