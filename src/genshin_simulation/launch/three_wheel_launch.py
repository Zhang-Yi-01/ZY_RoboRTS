import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = FindPackageShare(package='genshin_simulation').find('genshin_simulation') 
    # urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    pkg_path = os.path.join(pkg_share, '../../../../src/genshin_simulation')
    world_model_path = pkg_path + '/world/2023rmus_true.world'
    urdf_model_path = pkg_path + '/urdf/fish_bot.urdf'
    
    robot_description = ParameterValue(Command(['xacro ', urdf_model_path]),value_type=str)
    
    # start_gazebo_cmd = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #             launch_arguments={'use_sim_time': 'true', 'world':""}.items()
    #          )

    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    # 将机器人模型通过robot_description话题发布出去（gazebo好像不能直接加载xacro文件）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path]
        # parameters=[{'robot_description': robot_description}]
        )

    # Launch the robot
    robot_name_in_model = "my_car"
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments = [ '-entity', robot_name_in_model,
                    #   '-topic', 'robot_description' ,
                      '-file' , urdf_model_path,
                      '-x', '0',
                      '-y', '0',
                      '-z', '0' 
                    ], output='screen')
        
        
    
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity_cmd) 

    return ld


