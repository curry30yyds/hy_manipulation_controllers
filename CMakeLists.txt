# Minimum CMake version required
cmake_minimum_required(VERSION 3.10.2)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

# Project setup
project(hy_manipulation_controllers VERSION 1.0.0 LANGUAGES C CXX)

# Set C++ standard and build type
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

# Compiler and linker flags
set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wno-comment -Wno-deprecated-declarations -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall -Wno-sign-compare -Wno-unused -Wno-comment -Wno-deprecated-declarations -g -pthread")
add_definitions(-D_GNU_SOURCE)

# Configure version and other headers
configure_file(
  ${PROJECT_SOURCE_DIR}/config.h.in
  ${PROJECT_SOURCE_DIR}/include/hy_manipulation_controllers/utils/config.h
)
configure_file(
  ${PROJECT_SOURCE_DIR}/version.h.in
  ${PROJECT_SOURCE_DIR}/include/hy_manipulation_controllers/utils/version.h
  @ONLY
)

# Option to build tests (disabled by default)
option(COMPILE_TESTS "Build hy_manipulation_controllers tests" OFF)

# Find ROS and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  hy_hardware_interface
)

# ROS-specific setup
if(roscpp_FOUND)
  message(STATUS "ROS Found, enabling ROS support!")
  add_definitions(-DMIDDLEWARE_ROS)
  include_directories(${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
  set(ROS_LIBRARIES ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    std_msgs 
    geometry_msgs 
    sensor_msgs
    hy_hardware_interface

  )
else()
  message(WARNING "ROS not found, disabling ROS support!")
endif()

find_package(PkgConfig REQUIRED)

# Find required third-party libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV QUIET)
find_package(nlohmann_json 3.11.0 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(trac_ik_lib REQUIRED)
# Include directories
include_directories(
  include
  /usr/local/include/hy_common
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${trac_ik_lib_INCLUDE_DIRS}
)

# Link directories
link_directories(include)

# Set install prefix and RPATH
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/../install" CACHE STRING "The path to use for make install" FORCE)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

file(GLOB_RECURSE
  core_src_files
  ${PROJECT_SOURCE_DIR}/src/core/*.cc
  ${PROJECT_SOURCE_DIR}/src/motion_control/*.cc
  ${PROJECT_SOURCE_DIR}/src/utils/*.cc
)

# 确保目录存在
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/logs)

# 构建核心库
add_library(
  hy_manipulation_controllers
  SHARED
  ${core_src_files}
)
target_link_libraries(hy_manipulation_controllers
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ROS_LIBRARIES}
  hy_common
  nlohmann_json::nlohmann_json
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${trac_ik_lib_LIBRARIES}
)

# Build executable node
add_executable(test_standard_scara_controller examples/test_standard_scara_controller.cc)
target_link_libraries(test_standard_scara_controller
  hy_manipulation_controllers
  ${catkin_LIBRARIES}
)
#test
add_executable(test_kinematics_solver src/test/test_kinematics_node.cc)
target_link_libraries(test_kinematics_solver
  hy_manipulation_controllers
  ${catkin_LIBRARIES}
)
add_executable(test_cartesian_trajectory src/test/test_cartesian_trajectory.cc)
target_link_libraries(test_cartesian_trajectory
  hy_manipulation_controllers
  ${catkin_LIBRARIES}
)
add_executable(test_my_scara_controller src/test/test_my_scara_controller.cc)
target_link_libraries(test_my_scara_controller
  hy_manipulation_controllers
  ${catkin_LIBRARIES}
)
add_executable(test_drag_controller src/test/test_drag_controller.cc)
target_link_libraries(test_drag_controller
  hy_manipulation_controllers
  ${catkin_LIBRARIES}
)

add_executable(test_cartesian_pose src/test/test_cartesian_pose.cc)
target_link_libraries(test_cartesian_pose
  hy_manipulation_controllers
  ${catkin_LIBRARIES}
)