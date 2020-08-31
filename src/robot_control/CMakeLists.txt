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
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robot_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(robot_control src/robot_control.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(robot_control
   ${catkin_LIBRARIES}
)


