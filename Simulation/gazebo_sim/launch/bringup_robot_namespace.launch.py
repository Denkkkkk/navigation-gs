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
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Whether or not to use the simulation time instead of the system time."
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value=hub_id,
            description='Prefix of the joint and link names'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        'hub_id',
        default_value=hub_id,
        description='HUB_ID for namespace isolation'
        )
    )
    
    prefix = LaunchConfiguration('prefix')
    use_sim= LaunchConfiguration('use_sim_time')

    """@@@
        路径获取
    """
    # 获取功能包的share路径
    urdf_package_path = get_package_share_directory('gazebo_sim')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'RMUC2024.world')
    
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
            'prefix:=',
            prefix,
            ' ',
            'use_sim:=',
            use_sim,
        ]
    )
    
    """@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        launch文件调用
    """
    # 启动实车的相关导航节点
    action_launch_nav_sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/nav_sim.launch.py']
        ),
        launch_arguments=[
            ('use_sim_time', 'true')
        ]
    )
    # 启动 gazebo-world
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[
            ('world', default_gazebo_world_path),
            ('verbose', 'true'),
            ('use_sim_time', 'true')
        ]
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
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # namespace=hub_id,
        # arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{
            'use_sim_time': True
            # 'frame_prefix': f'{hub_id}/' if hub_id else ''  # 确保这里正确设置
            }],
        output='screen',
    )
    
    # 为 spawn_entity 
    action_spawn_entity = Node(
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
        parameters=[{'use_sim_time': True}]
    ),

    # RViz2 
    action_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )
    
    # robot_simulator 节点
    action_robot_simulator = Node(
        package='gazebo_sim',
        executable='gazebo_sim',
        name='gazebo_sim',
        namespace=hub_id,
        output='screen',
        respawn=True,
        parameters=[{
            'robotFrame': f'{hub_id}/base_footprint' if hub_id else 'base_footprint',
            'use_sim_time': True
        }]
    )
    
    # 为 map_to_odom 节点添加命名空间
    action_map_to_odom = Node(
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
            'use_sim_time': True
        }],
        # 重新映射坐标系话题
        # remappings=[
        #     ('/tf', f'/{hub_id}/tf' if hub_id else '/tf'),
        #     ('/tf_static', f'/{hub_id}/tf_static' if hub_id else '/tf_static'),
        # ] if hub_id else []
    )

    ld = LaunchDescription()
    
    # 添加所有动作
    
    nodes = [
        action_launch_gazebo,
        action_launch_nav_sim,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        action_spawn_entity,
        action_rviz_node,
        action_robot_simulator,
        action_map_to_odom
    ]

    
    return LaunchDescription(declared_arguments + nodes)