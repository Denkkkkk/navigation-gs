import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hub_id = LaunchConfiguration("hub_id")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether or not to use the simulation time instead of the system time."
    )

    declare_hub_id_cmd = DeclareLaunchArgument(
        "hub_id",
        default_value=os.environ.get("HUB_ID"),
        description="hub id, defult hub_id is read from environment variable HUB_ID"
    )
    
    sdk_node_cmd = Node(
            package='nav_top_sdk',
            executable='sdk_node',
            namespace=hub_id,
            name='sdk_node',
            output='screen',
            parameters=[{'use_sim_time' : use_sim_time}]
        )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_hub_id_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(sdk_node_cmd)
    
    return ld