#定义cmake需求版本和工程名称
cmake_minimum_required(VERSION 3.0.2)
project(robot_localization)

#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#第三方库配置
set(ALL_TARGET_LIBRARIES "")
include(cmake/CSM.cmake)
include(cmake/g2o.cmake)
include(cmake/glog.cmake) 
include(cmake/pcl.cmake) 
include(cmake/sophus.cmake)
include(cmake/yamlcpp.cmake)
include(cmake/protobuf.cmake)
# include(cmake/geographicLib.cmake)
include(cmake/ceres.cmake)

#ROS配置
set( ROS_CXX_DEPENDENCIES
  roscpp
  roslib
  std_msgs
  tf
)
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

# lidar耦合imu里程计节点
add_executable(lidar_odom_end_node src/apps/lidar_odom_end_node.cpp ${ALL_SRCS} )
target_link_libraries(lidar_odom_end_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )

# 点云图匹配节点
add_executable(matching_end_node src/apps/matching_end_node.cpp ${ALL_SRCS} )
target_link_libraries(matching_end_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )

# 基于图优化之滑动窗口模型的后端优化节点
add_executable(optimization_end_node src/apps/optimizing_end_node.cpp ${ALL_SRCS} )
target_link_libraries(optimization_end_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )
