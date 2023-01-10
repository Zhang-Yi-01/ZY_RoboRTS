# ZY_RoboRTS
各种包含之下，又引入了太多模型，编译过程实在是，太冗长
``catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_localization"`` 这已经是ros1的过去式了

编译请用：
`` colcon build --packages-select genshin_simulation robot_localization --symlink-install ``