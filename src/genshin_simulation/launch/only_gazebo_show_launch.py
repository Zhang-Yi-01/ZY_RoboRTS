import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='genshin_simulation').find('genshin_simulation') 
    pkg_path = os.path.join(pkg_share, '../../../../src/genshin_simulation')
    world_model_path = pkg_path + '/world/2023rmus_full.world'

    
    start_gazebo_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'use_sim_time': 'true', 'world':world_model_path}.items()
             )
        
    
    ld.add_action(start_gazebo_cmd)

    return ld


