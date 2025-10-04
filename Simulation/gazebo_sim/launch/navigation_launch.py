# Copyright 2025 Lihan Chen
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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution

from launch.actions import LogInfo

param_rewrites = lambda prefix: {
    'global_costmap.global_costmap.ros__parameters.global_frame': [prefix, TextSubstitution(text="/map")],
    'global_costmap.global_costmap.ros__parameters.robot_base_frame': [prefix, TextSubstitution(text="/base_footprint")],
    'global_costmap.global_costmap.ros__parameters.intensity_voxel_layer.terrain_map_ext.sensor_frame': [prefix, TextSubstitution(text="/base_footprint")],
    'local_costmap.local_costmap.ros__parameters.global_frame': [prefix, TextSubstitution(text="/odom")],
    'local_costmap.local_costmap.ros__parameters.robot_base_frame': [prefix, TextSubstitution(text="/base_footprint")],
    'local_costmap.local_costmap.ros__parameters.intensity_voxel_layer.terrain_map.sensor_frame': [prefix, TextSubstitution(text="/base_footprint")],
    'bt_navigator.ros__parameters.global_frame': [prefix, TextSubstitution(text="/map")],
    'bt_navigator.ros__parameters.robot_base_frame': [prefix, TextSubstitution(text="/base_footprint")],
    'behavior_server.ros__parameters.local_frame': [prefix, TextSubstitution(text="/odom")],
    'behavior_server.ros__parameters.global_frame': [prefix, TextSubstitution(text="/map")],
    'behavior_server.ros__parameters.robot_base_frame': [prefix, TextSubstitution(text="/base_footprint")]
}

def generate_launch_description():
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "use_sim_time": use_sim_time, 
        "autostart": autostart
    }

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT", "1"
    )
    
    hub_id = os.environ.get('HUB_ID', '')
    bringup_dir = get_package_share_directory('nav_bringup')
    param_file = os.path.join(bringup_dir, 'config', 'nav2_params_new.yaml')
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_file,
            # 指定参数的命名空间根节点
            root_key=hub_id,
            # 确保参数值被正确转换为适当的类型
            convert_types=True,
            param_rewrites=param_rewrites(hub_id)
        ),
        # 允许在参数中使用 ROS2 启动文件的替换表达式
        allow_substs=True,
    )
    
    container_name = "nav2_container"
    
    nav2_container_node = Node(
        name=container_name,
        namespace=hub_id,
        package="rclcpp_components",
        executable="component_container_isolated",
        parameters=[
            configured_params,
            param_substitutions
        ],
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=PathJoinSubstitution([hub_id, container_name]),
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                namespace=hub_id,
                parameters=[
                    {
                        "use_sim_time": use_sim_time, 
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                namespace=hub_id,
                parameters=[
                    configured_params, 
                    {
                        "use_sim_time":use_sim_time,
                        # "global_frame": hub_id + "/map",
                        # "robot_base_frame": hub_id + "/base_footprint"
                    }
                ],
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                namespace=hub_id,
                parameters=[
                    configured_params, 
                    {
                        "use_sim_time":use_sim_time,
                        # "global_frame": hub_id + "/map",
                        # "local_frame": hub_id + "/odom",
                        # "robot_base_frame": hub_id + "/base_footprint"
                    }
                ],
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                namespace=hub_id,
                parameters=[
                    configured_params, 
                    {
                        "use_sim_time":use_sim_time,
                        # "global_frame": hub_id + "/map",
                        # "robot_base_frame": hub_id + "/base_footprint",
                        # "sensor_frame": hub_id + "/base_footprint"
                    }
                ],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                namespace=hub_id,
                parameters=[
                    configured_params, 
                    {
                        "use_sim_time":use_sim_time
                    }
                ],
            ),
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                namespace=hub_id,
                remappings=[
                    ("cmd_vel", "cmd_vel")
                ],
                parameters=[
                    configured_params, 
                    {
                        "use_sim_time":use_sim_time,
                        # TODO: this could be `odom`
                        # "global_frame": hub_id + "/map",
                        # "robot_base_frame": hub_id + "/base_footprint",
                        # "sensor_frame": hub_id + "/base_footprint"
                    }
                ],
                
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                namespace=hub_id,
                remappings=[
                    ("cmd_vel", "cmd_vel"),  # remap input
                ],
                parameters=[
                    configured_params, 
                    {
                        "use_sim_time":use_sim_time
                    }
                ],
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                namespace=hub_id,
                parameters=[
                    configured_params, 
                    {
                        "use_sim_time":use_sim_time
                    }
                ]
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(nav2_container_node)
    ld.add_action(load_composable_nodes)

    return ld
