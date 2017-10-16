cmake_minimum_required(VERSION 2.8.3)
project(schunk_ft_sensor)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  socketcan_interface
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    socketcan_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(candump src/candump.cpp)
add_dependencies(candump ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(candump ${catkin_LIBRARIES})
