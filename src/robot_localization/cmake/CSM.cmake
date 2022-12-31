# CSM库配置
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)
include_directories(${csm_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})
list(APPEND THIRD_PART_LIBRARIES ${csm_LIBRARIES})
