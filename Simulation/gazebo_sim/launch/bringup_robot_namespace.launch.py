import launch
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from launch.substitutions import EnvironmentVariable
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        参数声明
    """
    hub_id = os.environ.get('HUB_ID', '')
    # 声明所有参数
    declared_arguments = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Whether or not to use the simulation time instead of the system time."
    )
    
    prefix_declare = DeclareLaunchArgument(
            'prefix',
            default_value=hub_id,
            description='Prefix of the joint and link names'
    )

    hub_id_declare = DeclareLaunchArgument(
        'hub_id',
        default_value=hub_id,
        description='HUB_ID for namespace isolation'
    )

    prefix = LaunchConfiguration('prefix')
    use_sim= LaunchConfiguration('use_sim_time')
    hub_id = LaunchConfiguration('hub_id')

    """@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        路径获取
    """
    # 获取功能包的share路径
    this_package_path = get_package_share_directory('gazebo_sim')
    default_rviz_config_path = os.path.join(this_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(this_package_path, 'world', 'RMUC2024.world')
    
    # get the robot description from the xacro file
    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('gazebo_sim'),
                    'urdf',
                    'robot/robot.urdf.xacro'
                ]
            ),
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'hub_id:=',
            hub_id,
        ]
    )
    
    """@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        launch文件调用
    """
    # 启动实车的相关导航节点
    nav_sim_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [this_package_path, '/launch', '/nav_sim.launch.py']
        ),
        launch_arguments=[
            ('use_sim_time', 'true')
        ]
    )
    # 启动 gazebo-world
    gazebo_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[
            ('world', default_gazebo_world_path),
            ('verbose', 'true'),
            ('use_sim_time', 'true')
        ]
    )
    terrain_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(this_package_path, "launch", "terrain.launch.py")
        ),
        launch_arguments={"use_sim_time": 'true'}.items()
    )

    """@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        node调用
    """
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # namespace=hub_id,
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        output='screen'
    )
    
    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        # namespace=hub_id,
        parameters=[{
            'use_sim_time': True
            # 'frame_prefix': f'{hub_id}/' if hub_id else ''  # 确保这里正确设置
            }],
        output='screen',
    )
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
            ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # RViz2 
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )
    
    # robot_simulator 节点
    robot_simulator_node = Node(
        package='gazebo_sim',
        executable='gazebo_sim',
        name='gazebo_sim',
        namespace=hub_id,
        output='screen',
        respawn=True,
        parameters=[{
            'hub_id': hub_id,
            'use_sim_time': True
        }]
    )
    
    # 为 map_to_odom 节点添加命名空间
    map_to_odom_node = Node(
        package='gazebo_sim',
        executable='map_to_odom',
        name='map_to_odom',
        namespace=hub_id,
        output='screen',
        respawn=True,
        parameters=[{
            'vehicleX': 0.0,
            'vehicleY': 0.0,
            'vehicleYaw': 0.0,
            # 'hub_id': hub_id,
            'use_sim_time': True
        }],
        # 重新映射坐标系话题
        # remappings=[
        #     ('/tf', f'/{hub_id}/tf' if hub_id else '/tf'),
        #     ('/tf_static', f'/{hub_id}/tf_static' if hub_id else '/tf_static'),
        # ] if hub_id else []
    )

    ld = LaunchDescription()
    # 添加所有声明的参数
    ld.add_action(declared_arguments)
    ld.add_action(prefix_declare)
    ld.add_action(hub_id_declare)
    # 添加所有动作
    ld.add_action(terrain_launch)
    ld.add_action(nav_sim_launch)
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_pub_node)
    ld.add_action(joint_state_pub_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(rviz2_node)
    ld.add_action(robot_simulator_node)
    ld.add_action(map_to_odom_node)
    
    return ld