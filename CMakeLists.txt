cmake_minimum_required(VERSION 3.0.2)
project(serial_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(LibSerial)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_ros
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LibSerial_INCLUDE_DIRS}
)

add_executable(serial_ros_node src/serial_ros_main.cpp src/serial_ros.cpp)
target_link_libraries(serial_ros_node ${catkin_LIBRARIES} serial)