find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND THIRD_PART_LIBRARIES ${YAML_CPP_LIBRARIES})


