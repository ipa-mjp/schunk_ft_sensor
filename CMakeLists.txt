cmake_minimum_required(VERSION 2.8.3)
project(schunk_ft_sensor)
add_compile_options(-std=c++11)

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

add_executable(schunk_ft_sensor_node src/schunk_ft_sensor_node.cpp src/schunk_ft_init.cpp src/schunk_ft_callbacks.cpp src/schunk_ft_sensor_data.cpp)
add_dependencies(schunk_ft_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(schunk_ft_sensor_node ${catkin_LIBRARIES})
