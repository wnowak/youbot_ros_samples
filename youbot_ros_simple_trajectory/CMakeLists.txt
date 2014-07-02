cmake_minimum_required(VERSION 2.8.3)
project(youbot_ros_simple_trajectory)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
)

catkin_package()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(youbot_ros_simple_trajectory src/youbot_ros_simple_trajectory.cpp)

target_link_libraries(youbot_ros_simple_trajectory
  ${catkin_LIBRARIES}
)

install(TARGETS youbot_ros_simple_trajectory youbot_ros_simple_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

