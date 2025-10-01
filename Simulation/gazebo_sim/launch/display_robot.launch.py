import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


import launch_ros.parameter_descriptions

def generate_launch_description():
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Whether or not to use the simulation time instead of the system time."
    )
    
    #获取功能包的share路径
    urdf_package_path=get_package_share_directory('gazebo_sim')
    default_xacro_path =os.path.join(urdf_package_path,'urdf','robot/robot.urdf.xacro')
    default_rviz_config_path =os.path.join(urdf_package_path,'config','display_robot_model.rviz')
    default_gazebo_world_path =os.path.join(urdf_package_path,'world','RMUC2024.world')

    #声明一个urdf目录的参数,方便修改
    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_xacro_path),description='加载的模型文件路径'
    )

    #通过文件路径，获取内容，并转换成参数值对象，以供传入 robot_state_publisher
    substitutions_command_result=launch.substitutions.Command([
        'xacro ',
        launch.substitutions.LaunchConfiguration('model')
        ]
    )
    robot_description_value=launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,value_type=str)

    action_robot_state_publisher=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    action_launch_gazebo=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[
            ('world',default_gazebo_world_path),
            ('verbose','true'),
            ('use_sim_time','true')
            ]
    )

    action_spawn_entity=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','robot']
    )

    action_joint_state_publisher=launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    action_rviz_node=launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',default_rviz_config_path]
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(action_declare_arg_mode_path)
    ld.add_action(action_robot_state_publisher)
    ld.add_action(action_joint_state_publisher)
    ld.add_action(action_launch_gazebo)
    ld.add_action(action_spawn_entity)
    ld.add_action(action_rviz_node)
    
    return ld