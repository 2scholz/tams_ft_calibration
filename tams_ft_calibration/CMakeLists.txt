cmake_minimum_required(VERSION 2.8.3)
project(tams_ft_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tams_ft_calibration_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibration_node src/calibration_node.cpp)
add_dependencies(calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calibration_node
  ${catkin_LIBRARIES}
)

add_executable(calibration_procedure_node src/calibration_procedure_node.cpp)
add_dependencies(calibration_procedure_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calibration_procedure_node
  ${catkin_LIBRARIES}
)

