cmake_minimum_required(VERSION 2.8.3)
project(baxter_cppkdl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_python_cpp
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

# EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS system )

#include_directories(/usr/include/eigen3)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} 
)

add_executable(moveit_with_pykdl src/moveit_with_pykdl.cpp)
target_link_libraries(moveit_with_pykdl ${catkin_LIBRARIES})

add_executable(moveit_with_pykdl_left src/moveit_with_pykdl_left.cpp)
target_link_libraries(moveit_with_pykdl_left ${catkin_LIBRARIES})
