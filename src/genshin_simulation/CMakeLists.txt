cmake_minimum_required(VERSION 3.8)
project(genshin_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(PROJECT_PATH ${PROJECT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(std_msgs)
find_package(geometry_msgs)



find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(nav_msgs)
find_package(nav2_msgs)

find_package(map_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)

# find_package(message_filters REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)


## 添加非ROS2功能包的依赖项时，需要将其对于的头文件路径在include_directories中写明（如下）。而对于依赖项为ROS2功能包时，则无需此操作（如上）。
## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

set(dependencies
  geometry_msgs
  map_msgs
  # message_filters
  nav2_msgs
  nav_msgs
  pluginlib
  rclcpp
  # rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  angles
)


include_directories($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>)

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)
# target_include_directories(teleop_twist_keyboard PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_twist_keyboard PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  teleop_twist_keyboard
  ${dependencies}
)
install(TARGETS teleop_twist_keyboard
  DESTINATION lib/${PROJECT_NAME})
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
