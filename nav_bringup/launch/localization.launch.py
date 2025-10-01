import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import TextSubstitution

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Whether or not to use the simulation time instead of the system time."
    )
    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        description="Full path to prior PCD file to load",
    )
    declare_default_enable_cmd = DeclareLaunchArgument(
        "default_enable",
        default_value="True",
        description="Whether or not to enable the default behavior."
    )

    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_enable = LaunchConfiguration("default_enable")
    
    bringup_dir = get_package_share_directory('nav_bringup')
    param_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    
    hub_id = os.environ.get('HUB_ID', '')

    small_gicp_container = ComposableNodeContainer(
        name='gicp_container',
        namespace=hub_id,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='small_gicp_relocalization',
                plugin='small_gicp_relocalization::SmallGicpRelocalizationNode',
                namespace=hub_id,
                name='small_gicp_relocalization',
                parameters=[
                    param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "prior_pcd_file": prior_pcd_file,
                        "default_enable": default_enable,
                        "map_frame": hub_id + "/map",
                        "odom_frame": hub_id + "/odom",
                        "robot_base_frame": hub_id + "/base_footprint"
                    }
                ],
                remappings=[("gicp/cloud_in", "/tower/mapping/cloud")],
                extra_arguments=[{'thread_num': 4}],
            )
        ],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_default_enable_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(small_gicp_container)

    return ld
