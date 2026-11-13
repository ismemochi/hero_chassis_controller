cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_toolbox
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs control_toolbox nav_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hero_chassis_controller
  src/hero_chassis_controller_node.cpp
  src/RosPackageTemplate.cpp
)

target_link_libraries(hero_chassis_controller ${catkin_LIBRARIES})
