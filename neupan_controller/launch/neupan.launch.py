from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('neupan_controller')

    namespace = LaunchConfiguration("namespace")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether or not to use the simulation time instead of the system time."
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    config_file = os.path.join(pkg_share, 'config', 'neupan_planner.yaml')
    dune_checkpoint = os.path.join(pkg_share, 'checkpoints', 'model_5000.pth')
    
    neupan_node = Node(
        package='neupan_controller',
        executable='neupan_node',
        name='neupan_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'config_file': config_file,
            'map_frame': [namespace, TextSubstitution(text='/map')],
            'base_frame': [namespace, TextSubstitution(text='/base_footprint')],
            'lidar_frame': [namespace, TextSubstitution(text='/hlidar')],
            'marker_size': 0.05,
            'marker_z': 0.3,
            'scan_angle_range': '-3.14 3.14',
            'scan_downsample': 1,
            'scan_range': '0.0 5.0',
            'dune_checkpoint': dune_checkpoint,
            'refresh_initial_path': True,
            'include_initial_path_direction': False,
        }, 
        {'use_sim_time' : use_sim_time},
        ],
        remappings=[
            ('scan', 'scan'),
            ('initial_path', 'received_global_plan'),
        ]
    )
        
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(neupan_node)
    
    return ld
