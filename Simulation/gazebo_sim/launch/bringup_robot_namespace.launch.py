import launch
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
import launch_ros.parameter_descriptions

def generate_launch_description():
    # 获取 HUB_ID 环境变量，如果没有则使用空字符串
    hub_id = os.environ.get('HUB_ID', '')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Whether or not to use the simulation time instead of the system time."
    )
    
    # 声明 HUB_ID 参数，可以在命令行覆盖
    declare_hub_id_cmd = DeclareLaunchArgument(
        'hub_id',
        default_value=hub_id,
        description='HUB_ID for namespace isolation'
    )
    
    # 获取功能包的share路径
    urdf_package_path = get_package_share_directory('gazebo_sim')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'robot/robot.urdf.xacro')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'RMUC2024.world')

    # 声明一个urdf目录的参数,方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_xacro_path), description='加载的模型文件路径'
    )
    
    # 通过文件路径，获取内容，并转换成参数值对象，以供传入 robot_state_publisher
    substitutions_command_result = launch.substitutions.Command([
        'xacro ',
        launch.substitutions.LaunchConfiguration('model'),
        ' hub_id:=',
        launch.substitutions.LaunchConfiguration('hub_id')  # 传递 hub_id 参数
    ])
    
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result, value_type=str
    )
    
    """
    下面开始节点或者其他launch文件的声明
    """
    
    ## 启动gazebo
    action_launch_nav_sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/nav_sim.launch.py']
        ),
        launch_arguments=[
            ('use_sim_time', 'true')
        ]
    )
    
    # 为 robot_state_publisher 添加命名空间
    action_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=hub_id,
        parameters=[{
            'robot_description': robot_description_value,
            'use_sim_time': True
            # 'frame_prefix': f'{hub_id}/' if hub_id else ''  # 为所有坐标系添加前缀
        }],
    )

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
    
    # 为 spawn_entity 添加命名空间
    action_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=hub_id,
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        parameters=[{'use_sim_time': True}]
    )

    # 为 joint_state_publisher 添加命名空间
    action_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=hub_id,
        parameters=[{
            'use_sim_time': True
            # 'frame_prefix': f'{hub_id}/' if hub_id else ''  # 确保这里正确设置
            }]
    )

    # RViz2 通常不需要命名空间，但可以设置remap规则
    action_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )
    
    # 为 robot_simulator 节点添加命名空间
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
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_hub_id_cmd)  # 添加 HUB_ID 声明
    ld.add_action(action_declare_arg_mode_path)
    ld.add_action(action_robot_state_publisher)
    ld.add_action(action_joint_state_publisher)
    ld.add_action(action_launch_gazebo)
    ld.add_action(action_spawn_entity)
    ld.add_action(action_rviz_node)
    # ld.add_action(action_robot_simulator)
    # ld.add_action(action_map_to_odom)
    # ld.add_action(action_launch_nav_sim)
    
    return ld