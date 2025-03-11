cmake_minimum_required(VERSION 2.8.3)
project(safer_gap)

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
  TimingStats.msg
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

set(ACADOS_INCLUDE_PATH /home/shiyu/Documents/SDK/acados/include CACHE PATH "Define the path which contains the include directory for acados.")
set(ACADOS_LIB_PATH /home/shiyu/Documents/SDK/acados/lib CACHE PATH "Define the path which contains the lib directory for acados.")

link_directories(
  /home/shiyu/Documents/SDK/or-tools_x86_64_Ubuntu-20.04_cpp_v9.5.2237/lib
  ${ACADOS_LIB_PATH}
)

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

###########
## Build ##
###########

set(full_model_build ${PROJECT_SOURCE_DIR}/scripts/unicycle_mobile_full_model/c_generated_code)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  # ${ORTOOLS_INCLUDE_DIRS}
  /home/shiyu/Documents/SDK/or-tools_x86_64_Ubuntu-20.04_cpp_v9.5.2237/include
  ${ACADOS_INCLUDE_PATH}
  ${ACADOS_INCLUDE_PATH}/acados
  ${ACADOS_INCLUDE_PATH}/acados_c
  ${ACADOS_INCLUDE_PATH}/blasfeo/include
  ${ACADOS_INCLUDE_PATH}/hpipm/include
  ${ACADOS_INCLUDE_PATH}/qpOASES_e/
  ${full_model_build}
)

# link to libraries
if(UNIX)
    link_libraries(acados hpipm blasfeo m -lqpOASES_e)
else()
    link_libraries(acados hpipm blasfeo -lqpOASES_e)
endif()

file(GLOB acados_ocp_solver_unicycle_keyhole_mpc
  ${full_model_build}/acados_sim_solver_unicycle_keyhole_mpc.c
  ${full_model_build}/acados_solver_unicycle_keyhole_mpc.c
  ${full_model_build}/unicycle_keyhole_mpc_model/*.c
  # ${full_model_build}/unicycle_keyhole_mpc_model/unicycle_keyhole_mpc_expl_ode_fun.c
  # ${full_model_build}/unicycle_keyhole_mpc_model/unicycle_keyhole_mpc_expl_vde_adj.c
  # ${full_model_build}/unicycle_keyhole_mpc_model/unicycle_keyhole_mpc_expl_vde_forw.c
  ${full_model_build}/unicycle_keyhole_mpc_constraints/*.c
)

add_library(unicycle_keyhole_mpc_shared_lib SHARED ${acados_ocp_solver_unicycle_keyhole_mpc})
target_link_libraries(unicycle_keyhole_mpc_shared_lib
	${ACADOS_LIB_PATH}/libacados.so
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

add_library(pg_mpc_hc_acados_local_planner
   src/controller_hc_acados.cpp
   src/pg_mpc_hc_acados_local_planner_ros.cpp
)

target_link_libraries(pg_mpc_hc_acados_local_planner
  keyhole
  corbo_controllers
  casadi
  ${catkin_LIBRARIES}
  unicycle_keyhole_mpc_shared_lib
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
