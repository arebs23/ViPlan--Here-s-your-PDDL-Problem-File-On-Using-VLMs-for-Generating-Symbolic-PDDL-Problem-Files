# Copyright (c) 2018 Intel Corporation
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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
# from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Get the launch directory

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            name='move_action_node',
            package='moveit2_scripts',
            executable='pick_place',
            remappings=remappings,
            output='screen'),

        Node(
            name='open_gripper_action_node',
            package='moveit2_scripts',
            executable='open_gripper_action_server',
            remappings=remappings,
            output='screen'),

         Node(
            name='close_gripper_action_node',
            package='moveit2_scripts',
            executable='close_gripper_action_server',
            remappings=remappings,
            output='screen'),

         Node(
            name='add_table_node',
            package='moveit2_scripts',
            executable='test_collision',
            remappings=remappings,
            output='screen'),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld