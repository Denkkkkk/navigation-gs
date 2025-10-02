import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch.conditions import UnlessCondition
from launch.substitutions import TextSubstitution
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ################ Declare arguments for CLI override ################
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Whether or not to use the simulation time instead of the system time."
    )
    declare_use_relocalization_cmd = DeclareLaunchArgument(
        'use_relocalization', 
        default_value='ERROR_BOOL', 
        description='Whether to use relocalization'
    )
    declare_use_online_map_cmd = DeclareLaunchArgument(
        'use_online_map',
        default_value='False', 
        description='Whether to use online map, if set to true, the map server will not be started and the map will be loaded from /HUB_ID/map topic instead of a static file.'
    )
    urdf_package_path=get_package_share_directory('gazebo_sim')
    declare_scene_name_cmd = DeclareLaunchArgument(
        'scene_name',
        default_value="rmuc2024",
        description='Name of the scene to load'
    )
    declare_maps_dir_cmd = DeclareLaunchArgument(
        'maps_dir',
        default_value=os.path.join(urdf_package_path,'map'),
        description='Directory containing the scene\'s map yaml file and pcd'
    )
    
    bringup_dir = get_package_share_directory('nav_bringup')
    param_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    
    # parameters
    hub_id = os.environ.get('HUB_ID', '')
    use_relocalization = LaunchConfiguration('use_relocalization')
    use_online_map = LaunchConfiguration('use_online_map')
    maps_dir = LaunchConfiguration("maps_dir")
    scene_name = LaunchConfiguration("scene_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # make map paths
    prior_pcd_file = PathJoinSubstitution([maps_dir, scene_name, PythonExpression(["'", scene_name, "' + '.pcd'"])])
    map_yaml_file = PathJoinSubstitution([maps_dir, scene_name, PythonExpression(["'", scene_name, "' + '.yaml'"])])

    map_server_container_node = Node(
        name="nav2_map_server_container",
        namespace=hub_id,
        condition=UnlessCondition(use_online_map),
        package="rclcpp_components",
        executable="component_container_isolated",
        parameters=[
            param_file, 
            {
                "autostart": True,
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=PathJoinSubstitution([hub_id, "nav2_map_server_container"]),
        condition=UnlessCondition(use_online_map),
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                namespace=hub_id,
                name="map_server",
                parameters=[
                    param_file,
                    {
                        "frame_id": hub_id + "/map",
                        "yaml_filename": map_yaml_file,
                        "use_sim_time": use_sim_time
                    }
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                namespace=hub_id,
                name="lifecycle_manager_map_server",
                parameters=[
                    {"autostart": True,
                     "node_names": ["map_server"],
                     "use_sim_time":use_sim_time
                    },
                ],
            ),
        ],
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                "launch",
                "navigation_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time":use_sim_time,
        }.items(),
    )
    
    neupan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('neupan_controller'),
                "launch",
                "neupan.launch.py"
            )
        ),
        launch_arguments={
            "namespace": hub_id,
            "use_sim_time":use_sim_time
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "localization.launch.py")
        ),
        condition=IfCondition(
            PythonExpression(['not ', LaunchConfiguration('use_online_map')])
        ),
        launch_arguments={
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "default_enable": PythonExpression([
                use_relocalization,
                ' and not ',
                LaunchConfiguration('use_online_map')
            ])
        }.items(),
    )
    
    nav_mode_switcher_node = Node(
        package='navigation_switch',
        executable='navigation_switch_node',
        name='nav_mode_switcher',
        namespace=hub_id,
        output='screen',
        parameters=[{"use_sim_time":use_sim_time}],
    )
    
    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav_top_sdk'),
                "launch",
                "bringup.launch.py"
            )
        ),
        launch_arguments={
            "hub_id": hub_id,
            "use_sim_time":use_sim_time,
        }.items(),
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_relocalization_cmd)
    ld.add_action(declare_use_online_map_cmd)
    ld.add_action(declare_scene_name_cmd)
    ld.add_action(declare_maps_dir_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # 添加节点和启动文件
    ld.add_action(map_server_container_node)
    ld.add_action(load_composable_nodes)
    ld.add_action(navigation_launch)
    ld.add_action(neupan_launch)
    ld.add_action(localization_launch)
    ld.add_action(nav_mode_switcher_node)
    ld.add_action(sdk_launch)

    return ld