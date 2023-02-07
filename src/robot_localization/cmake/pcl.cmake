# set(PCL_DIR "/usr/local/pcl-1.10/share/pcl-1.10")
# find_package(PCL 1.10 REQUIRED )

# set(PCL_DIR "/usr/local/pcl-1.13.0/share/pcl-1.13.0")
find_package(PCL  REQUIRED )



# find_package(PCL 1.12 REQUIRED)




# FIND_PATH(PCL_LIBRARY_DIRS 
#   /usr/local/include
#   # /opt/ros/$ENV{ROS_DISTRO}/include
#   NO_DEFAULT_PATH
# )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



list(APPEND THIRD_PART_LIBRARIES ${PCL_LIBRARIES})
