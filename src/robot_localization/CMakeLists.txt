cmake_minimum_required(VERSION 3.8)
project(robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs)
find_package(sensor_msgs )
find_package(tf2)
find_package(tf2_ros)
find_package(nav_msgs)
find_package(tf2_sensor_msgs)
find_package(tf2_geometry_msgs)
find_package(pcl_msgs)

set(ROS2_DEPENDENCIES
  geometry_msgs
  rclcpp
  nav_msgs
  pcl_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

# 解释 .h.in文件里面制定的路径，解释到.h里
configure_file(
                ${PROJECT_SOURCE_DIR}/include/global_path_defination/global_path.h.in 
                ${PROJECT_SOURCE_DIR}/include/global_path_defination/global_path.h
              )

#第三方库配置
set(THIRD_PART_LIBRARIES "")
include(cmake/glog.cmake) 
include(cmake/pcl.cmake) 
include(cmake/yamlcpp.cmake)
include(cmake/sophus.cmake)
include(cmake/g2o.cmake)
include(cmake/ceres.cmake)

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp" )
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS} )

# 数据预处理端节点
add_executable(data_pretreat_end_node src/apps/data_pretreat_end_node.cpp ${ALL_SRCS} )
# ament_target_dependencies只能链接find_package()找到的包，如果是自定义的库文件，仍需要使用target_link_libraries的方式链接
ament_target_dependencies( 
  data_pretreat_end_node
  ${ROS2_DEPENDENCIES}
)
target_link_libraries(data_pretreat_end_node 
  Sophus::Sophus
  ${THIRD_PART_LIBRARIES}
)


# lidar耦合imu里程计节点
add_executable(lidar_odom_end_node src/apps/lidar_odom_end_node.cpp ${ALL_SRCS} )
ament_target_dependencies(
  lidar_odom_end_node
  ${ROS2_DEPENDENCIES}
)
target_link_libraries(lidar_odom_end_node 
  Sophus::Sophus
  ${THIRD_PART_LIBRARIES}
)


# 点云图匹配节点
add_executable(matching_end_node src/apps/matching_end_node.cpp ${ALL_SRCS} )
ament_target_dependencies(
  matching_end_node
  ${ROS2_DEPENDENCIES}
)
target_link_libraries(matching_end_node 
  Sophus::Sophus
  ${THIRD_PART_LIBRARIES}
)

# # 基于图优化之滑动窗口模型的后端优化节点
# add_executable(optimization_end_node src/apps/optimizing_end_node.cpp ${ALL_SRCS} )
# ament_target_dependencies(
#   optimization_end_node
#   ${ROS2_DEPENDENCIES}
# )
# target_link_libraries(optimization_end_node 
#   ${THIRD_PART_LIBRARIES}
# )



install(TARGETS data_pretreat_end_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_odom_end_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS matching_end_node
  DESTINATION lib/${PROJECT_NAME})
# install(TARGETS optimization_end_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})



















if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
