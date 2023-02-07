import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='robot_localization').find('robot_localization')
    pkg_path = os.path.join(pkg_share, '../../../../src/robot_localization')
    
    
    data_pretreat_end_node = Node(
                                package="robot_localization",
                                executable='data_pretreat_end_node',
                                output='screen',emulate_tty=True

    )   

    lidar_odom_end_node = Node(
                                package="robot_localization",
                                executable='lidar_odom_end_node',
                                output='screen',emulate_tty=True
                              )
    start_rviz2_node = Node(
                        package="rviz2",
                        executable="rviz2",
                        name='rviz2',
                        output='screen',emulate_tty=True,
                        arguments=['-d', pkg_path+'/config/rviz/3d_lidar.rviz']

                     )
        
    map_to_odom_static_transform_node = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name = 'map_to_odom_static_tf',
        arguments = [ "0 0 0 0 0 0 /map /odom 1000"]




    )
    
    # ld.add_action(map_to_odom_static_transform_node)
    ld.add_action(data_pretreat_end_node)
    ld.add_action(lidar_odom_end_node)

    ld.add_action(start_rviz2_node)


    return ld


