cmake_minimum_required(VERSION 2.8.3)
project(coverage)

find_package(catkin REQUIRED COMPONENTS
  angles
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(coverage_node 
    src/coverage_node.cpp
    src/MapGraph.cpp
    src/StcPathPlanner.cpp
)

target_link_libraries(coverage_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS coverage_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/*
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
