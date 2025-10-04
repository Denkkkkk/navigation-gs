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
    basic_dir = get_package_share_directory("basic_bringup")
    
    params_file = os.path.join(basic_dir, "config", "basic_params.yaml")

    # LaunchConfiguration variables
    hub_id = LaunchConfiguration("hub_id")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    param_rewrites={
        'pointcloud_to_laserscan.ros__parameters.target_frame': [hub_id, TextSubstitution(text="/base_footprint")]
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            # 指定参数的命名空间根节点
            root_key=hub_id,
            # 确保参数值被正确转换为适当的类型
            convert_types=True,
            param_rewrites=param_rewrites | {"use_sim_time": use_sim_time}
        ),
        # 允许在参数中使用 ROS2 启动文件的替换表达式
        allow_substs=True,
    )
    
    start_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        namespace=hub_id,
        output="screen",
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cloud_in", "terrain_map"),
            ("scan", "scan"),
        ],
    )
    
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_hub_id_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    # Add nodes to the launch description
    ld.add_action(start_pointcloud_to_laserscan_node)
    
    return ld
