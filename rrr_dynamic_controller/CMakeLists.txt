cmake_minimum_required(VERSION 3.0.2)
project(rrr_dynamic_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  kdl_parser
  pluginlib
  roscpp
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrr_dynamic_controller
  CATKIN_DEPENDS controller_interface hardware_interface kdl_parser pluginlib roscpp urdf
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/computed_torque.cpp src/oper_space_ct.cpp src/back_stepping.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})