cmake_minimum_required(VERSION 2.8.3)
project(gazebo_vsm)

set(PKG_DEPS
  roscpp
  gazebo_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${PKG_DEPS}
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    ${PKG_DEPS}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(vsm_world_plugin src/vsm_world_plugin.cpp)
target_link_libraries(vsm_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(TARGETS vsm_world_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
)
