cmake_minimum_required(VERSION 2.8.3)

project(gazebo_vsm)

cmake_policy(SET CMP0054 OLD)

find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(catkin REQUIRED COMPONENTS)
find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    pugixml-dev
)

###########
## Build ##
###########
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow")


# setup vsm
add_definitions(-DZMQ_BUILD_DRAFT_API=1)
include(ExternalProject)
ExternalProject_Add(libvsm
  GIT_REPOSITORY https://github.com/wzli/VirtualSpaceMeshnet.git
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX} -DCMAKE_CXX_FLAGS="-fPIC"
)
ExternalProject_Get_Property(libvsm install_dir)
SET(libvsm_SRC_DIR ${install_dir}/src/libvsm)
SET(libvsm_INCLUDE_DIRS
  ${libvsm_SRC_DIR}/include
  ${libvsm_SRC_DIR}/submodules/cppzmq
  ${libvsm_SRC_DIR}/submodules/libzmq/include
  ${libvsm_SRC_DIR}/submodules/flatbuffers/include
)

include_directories(
  include
  ${libvsm_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/gazebo_vsm.cpp)
add_dependencies(${PROJECT_NAME} libvsm)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libvsm.a
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  yaml-cpp
  pugixml
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/..
  FILES_MATCHING PATTERN "libzmq.so*"
)
