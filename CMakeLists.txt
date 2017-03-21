cmake_minimum_required(VERSION 2.8.3)
project(ardronecontrol)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable

add_executable(ardronev1 src/ardronev1.cpp)
target_link_libraries(ardronev1 ${catkin_LIBRARIES} pid)

add_executable(PoseStampedPub src/PoseStampedPub.cpp)
target_link_libraries(PoseStampedPub ${catkin_LIBRARIES})

add_executable(filter_empty_msgs src/filter_empty_msgs.cpp)
target_link_libraries(filter_empty_msgs ${catkin_LIBRARIES})

add_executable(TwistPub src/TwistPub.cpp)
target_link_libraries(TwistPub ${catkin_LIBRARIES})
# Make a PID library
add_library(pid src/pid.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
