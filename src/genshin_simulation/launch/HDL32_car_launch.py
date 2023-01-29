import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, EmitEvent, RegisterEventHandler


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = FindPackageShare(package='genshin_simulation').find('genshin_simulation') 
    # urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    pkg_path = os.path.join(pkg_share, '../../../../src/genshin_simulation')
    world_model_path = pkg_path + '/world/2023rmus_full.world'
    urdf_model_path = pkg_path + '/urdf/my_car_HDL32.xacro'
    

    declare_gpu_cmd = DeclareLaunchArgument(
    'gpu',
    default_value='False',
    description='Whether to use Gazebo gpu_ray or ray')
    declare_organize_cloud_cmd = DeclareLaunchArgument(
    'organize_cloud',
    default_value='False',
    description='Organize PointCloud2 into 2D array with NaN placeholders, otherwise 1D array and leave out invlaid points')
    gpu = LaunchConfiguration('gpu')
    organize_cloud = LaunchConfiguration('organize_cloud')

    robot_description = ParameterValue(Command(['xacro ', urdf_model_path, ' gpu:=', gpu, ' organize_cloud:=', organize_cloud]),value_type=str)

    # 启动 gazebo世界，开verbose可能会有怪怪的警告
    # start_gazebo_cmd =  ExecuteProcess(
    #     cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
    #     world_model_path],output='screen')

    # start_gazebo_cmd =  ExecuteProcess(
    #     cmd=['gazebo', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
    #     output='screen')

    start_gazebo_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={
                                    'use_sim_time': 'true',
                                    'world':world_model_path
                                 }.items()
             )
    # 将机器人模型通过robot_description话题发布出去（gazebo好像不能直接加载xacro文件）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # arguments=[urdf_model_path]
        parameters=[
                    {
                        'robot_description': robot_description,
                        'gpu': 'false',
                        'organize_cloud': 'false'
                    }
                    
                    
                    ]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # arguments=[urdf_model_path]
        parameters=[{'robot_description': robot_description}]
        )

    # Launch the robot
    robot_name_in_model = "my_car"
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments = [ '-entity', robot_name_in_model,
                      '-topic', 'robot_description' , #有两种模型连接方式，通过话题或者通过文件读取，后者有些报错解决不了
                    #   '-file', urdf_model_path ,  
                      '-x', '5.0',
                      '-y', '2.0',
                      '-z', '0.15' 
                    ], output='screen')
    
        
    
    ld.add_action(start_gazebo_cmd)
    ld.add_action(declare_gpu_cmd)
    ld.add_action(declare_organize_cloud_cmd)

    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity_cmd) 

    return ld


