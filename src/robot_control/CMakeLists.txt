cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_control
  CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(controller_implementation src/controller_implementation.cpp)
add_executable(controller_implementation_node src/controller_implementation_node.cpp)
 target_link_libraries(controller_implementation_node controller_implementation ${catkin_LIBRARIES})


