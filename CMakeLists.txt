cmake_minimum_required(VERSION 2.8.3)
project(schunk_ft_sensor)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  socketcan_interface
  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(_SOURCES
	src/schunk_ft_sensor_node.cpp
	src/schunk_ft_state.cpp
    src/schunk_ft_state.cpp
    src/schunk_ft_callbacks.cpp
    src/schunk_ft_sensor_data.cpp
    )

catkin_package(CATKIN_DEPENDS
               DEPENDS geometry_msgs socketcan_interface roscpp
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})   

add_library(${PROJECT_NAME} ${_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node ${_SOURCES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})