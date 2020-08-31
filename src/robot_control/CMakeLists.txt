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
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
 target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})


