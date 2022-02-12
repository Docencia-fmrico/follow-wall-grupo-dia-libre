# Copyright 2022 Grupo Día Libre
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    composable_node_pub_cmd = Node(
        package='follow_wall',
        executable='follow_wall_node',
        # node_name='composable_node_pub',
        output='screen',
        parameters=[]
        )

    world = LaunchConfiguration('world', default='home')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='home',
        description='World name')

    tiago_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')),
        launch_arguments={
          'world_name': world
        }.items())

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(composable_node_pub_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(tiago_sim_cmd)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    return ld
