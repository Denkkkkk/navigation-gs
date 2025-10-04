# Copyright 2025 Weilin Zhu
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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    basic_dir = get_package_share_directory("basic_bringup")
    this_dir = get_package_share_directory("gazebo_sim")

    hub_id = LaunchConfiguration("hub_id")

    declare_hub_id_cmd = DeclareLaunchArgument(
        "hub_id",
        default_value=os.environ.get("HUB_ID", None),
        description="hub id"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether or not to use the simulation time instead of the system time."
    )
    
    terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(basic_dir, "launch", "terrain.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )
    
    transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_dir, "launch", "point_cloud_transform.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_hub_id_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(terrain_launch)
    ld.add_action(transform_launch)

    return ld
