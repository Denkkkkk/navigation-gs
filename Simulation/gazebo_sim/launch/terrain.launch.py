from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, OrSubstitution, NotSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    basic_dir = get_package_share_directory("gazebo_sim")
    
    param_file = os.path.join(basic_dir, "config", "basic_params.yaml")


    # Declare arguments for CLI override
    declare_hub_id_cmd = DeclareLaunchArgument(
        "hub_id",
        default_value=os.environ.get("HUB_ID", None),
        description="hub id"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether or not to use the simulation time instead of the system time."
    )
    
    # LaunchConfiguration variables
    hub_id = LaunchConfiguration("hub_id")
    use_sim_time = LaunchConfiguration("use_sim_time")

    param_rewrites={
        'terrain_analysis.ros__parameters.odom_frame': [hub_id, TextSubstitution(text="/odom")],
        'terrain_analysis_ext.ros__parameters.odom_frame': [hub_id, TextSubstitution(text="/odom")],
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_file,
            # 指定参数的命名空间根节点
            root_key=hub_id,
            # 确保参数值被正确转换为适当的类型
            convert_types=True,
            param_rewrites=param_rewrites | {"use_sim_time": use_sim_time}
        ),
        # 允许在参数中使用 ROS2 启动文件的替换表达式
        allow_substs=True,
    )
    
    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        namespace=hub_id,
        remappings=[
            ("lidar_odometry", "odometry"),
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    start_terrain_analysis_ext_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        namespace=hub_id,
        remappings=[
            ("lidar_odometry", "odometry"),
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_hub_id_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    ld.add_action(start_terrain_analysis_cmd)
    ld.add_action(start_terrain_analysis_ext_cmd)
    
    return ld
