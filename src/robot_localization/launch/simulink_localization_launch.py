import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    

    start_simulation_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('genshin_simulation'), 'launch', 'HDL32_car_launch.py')])
             )
             

    lidar_odom_end_node = Node(
                                package="robot_localization",
                                executable='lidar_odom_end_node',

                                output='screen'


                              )
    rviz2_node = Node(
                        package="rviz2",
                        executable="rviz2"
                     )
        
    
    ld.add_action(start_simulation_cmd)
    ld.add_action(lidar_odom_end_node)
    ld.add_action(rviz2_node)


    return ld


