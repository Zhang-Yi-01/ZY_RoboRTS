# find_package( g2o REQUIRED )
# include_directories( ${G2O_INCLUDE_DIRS} )
# # set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )
# set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3  )
# list(APPEND THIRD_PART_LIBRARIES ${G2O_LIBS})




# Find the header files
FIND_PATH(G2O_INCLUDE_DIR 
  g2o/core/base_vertex.h
  g2o/core/base_unary_edge.h
  g2o/core/base_binary_edge.h
  ${G2O_ROOT}/include
  $ENV{G2O_ROOT}/include
  $ENV{G2O_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  # /opt/ros/$ENV{ROS_DISTRO}/include
  NO_DEFAULT_PATH
)

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

MACRO(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ${G2O_ROOT}/lib/Debug
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Debug
    $ENV{G2O_ROOT}/lib
    # /opt/ros/$ENV{ROS_DISTRO}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "libg2o_${MYLIBRARYNAME}.so"
    PATHS
    ${G2O_ROOT}/lib/Release
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Release
    $ENV{G2O_ROOT}/lib
    # /opt/ros/$ENV{ROS_DISTRO}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "libg2o_${MYLIBRARYNAME}.so"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    NO_DEFAULT_PATH
    )
  IF(NOT ${MYLIBRARY}_DEBUG)
    IF(MYLIBRARY)
      SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    ENDIF(MYLIBRARY)
  ENDIF( NOT ${MYLIBRARY}_DEBUG)

ENDMACRO()

# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)

# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D_ADDONS types_slam3d_addons)

# G2O solvers declared found if we found at least one solver
SET(G2O_SOLVERS_FOUND "NO")
IF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
  SET(G2O_SOLVERS_FOUND "YES")
ENDIF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)

# G2O itself declared found if we found the core libraries and at least one solver
SET(G2O_FOUND "NO")
IF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
  SET(G2O_FOUND "YES")
ENDIF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)

# 加入引用文件和库文件中去
include_directories(SYSTEM ${G2O_INCLUDE_DIR})
list(APPEND THIRD_PART_LIBRARIES 
      ${G2O_TYPES_DATA}
      ${G2O_CORE_LIBRARY}
      ${G2O_STUFF_LIBRARY}
      ${G2O_SOLVER_PCG}
      ${G2O_SOLVER_CSPARSE}
      ${G2O_SOLVER_CHOLMOD}
      ${G2O_TYPES_SLAM3D}
      ${G2O_TYPES_SLAM3D_ADDONS})