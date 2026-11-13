cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_toolbox
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs control_toolbox
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hero_chassis_controller
  src/hero_chassis_controller_node.cpp
  src/RosPackageTemplate.cpp
)

# 只要这一句 ${catkin_LIBRARIES}，就能把 control_toolbox 完美链接进来！
target_link_libraries(hero_chassis_controller ${catkin_LIBRARIES})
