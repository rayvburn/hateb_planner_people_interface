cmake_minimum_required(VERSION 3.5)
project(hateb_planner_people_interface)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
  people_msgs
  people_msgs_utils
  hanp_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp tf tf2_ros people_msgs people_msgs_utils hanp_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tracked_persons_to_hateb_node
  src/tracked_persons_to_hateb_node.cpp
)

target_link_libraries(tracked_persons_to_hateb_node
  ${catkin_LIBRARIES}
)

install(TARGETS tracked_persons_to_hateb_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
