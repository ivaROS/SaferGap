cmake_minimum_required(VERSION 2.8.3)
project(potential_gap_mpc)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${ORTOOLS_ROOT})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  costmap_converter
  dynamic_reconfigure
  roscpp
  geometry_msgs
  interactive_markers
  nav_core
  nav_msgs
  mbf_costmap_core
  mbf_msgs
  mpc_local_planner_msgs
  pluginlib
  std_msgs
  teb_local_planner
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  mpc_local_planner
  potential_gap
  pips_egocylindrical
  pcl_ros
  trajectory_generator
  turtlebot_trajectory_testing
  turtlebot_trajectory_functions
  tf2_pips
  cubic_spline_smoother
  message_generation
)

add_message_files(
  FILES
  OptimalStats.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

find_package(control_box_rst QUIET COMPONENTS
  core
  communication
  controllers
  numerics
  systems
  optimization
  optimal_control
  systems
  PATH_SUFFIXES share/control_box_rst control_box_rst
  PATHS ${CMAKE_BINARY_DIR} /usr/local
)

link_directories(/home/shiyu/Documents/SDK/or-tools_x86_64_Ubuntu-20.04_cpp_v9.5.2237/lib)

find_package(casadi REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(osqp QUIET)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(EXTERNAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS roscpp mpc_local_planner potential_gap pips_egocylindrical mpc_local_planner_msgs control_box_rst teb_local_planner pcl_ros
                 trajectory_generator turtlebot_trajectory_testing turtlebot_trajectory_functions tf2_pips cubic_spline_smoother message_runtime std_msgs
  # DEPENDS
)

include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  # ${ORTOOLS_INCLUDE_DIRS}
  /home/shiyu/Documents/SDK/or-tools_x86_64_Ubuntu-20.04_cpp_v9.5.2237/include
)

add_subdirectory(3rdParty/matplotplusplus)

add_library(keyhole
   src/keyhole.cpp
)

target_link_libraries(keyhole
  # ${ORTOOLS_LIBRARIES}
  ortools
  matplot
  ${catkin_LIBRARIES}
)

add_library(pg_mpc_hc_local_planner
   src/controller_hc.cpp
   src/pg_mpc_hc_local_planner_ros.cpp
)

target_link_libraries(pg_mpc_hc_local_planner
  keyhole
  corbo_controllers
  casadi
  ${catkin_LIBRARIES}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  pg_mpc_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch cfg scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)