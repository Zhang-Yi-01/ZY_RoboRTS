find_package(PCL 1.8 REQUIRED)

# find_package(PCL)

# include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND THIRD_PART_LIBRARIES ${PCL_LIBRARIES})
