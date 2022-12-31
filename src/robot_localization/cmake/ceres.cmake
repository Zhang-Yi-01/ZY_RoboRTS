find_package (Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
list(APPEND THIRD_PART_LIBRARIES ${CERES_LIBRARIES})