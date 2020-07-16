cmake_minimum_required(VERSION 2.8.3)

project(gazebo_vsm)

cmake_policy(SET CMP0054 OLD)

find_package(catkin REQUIRED COMPONENTS)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)

# VSM setup
add_subdirectory(VirtualSpaceMeshnet)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
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

add_library(${PROJECT_NAME} src/gazebo_vsm.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  yaml-cpp
  vsm
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
