find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND THIRD_PART_LIBRARIES ${GeographicLib_LIBRARIES})
