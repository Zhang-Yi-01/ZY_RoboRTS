# ZY_RoboRTS
各种包含之下，又引入了太多模型，编译过程实在是，太冗长
``catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_localization"`` 这已经是ros1的过去式了

编译请用：
`` colcon build --packages-select genshin_simulation robot_localization --symlink-install ``


dpkg -l | grep gazebo



sudo apt-get remove gazebo gazebo-common gazebo-plugin-base libgazebo-dev libgazebo11:amd64 ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-msgs-dbgsym ros-humble-gazebo-plugins ros-humble-gazebo-plugins-dbgsym  ros-humble-gazebo-ros  ros-humble-gazebo-ros-dbgsym ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control  ros-humble-gazebo-ros2-control-dbgsym ros-humble-gazebo-ros2-control-demos  ros-humble-gazebo-ros2-control-demos-dbgsym ros-humble-velodyne-gazebo-plugins ros-humble-velodyne-gazebo-plugins-dbgsym

