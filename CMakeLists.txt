cmake_minimum_required(VERSION 2.8.3)
project(ardronecontrol)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

generate_dynamic_reconfigure_options(
    config/PIDset.cfg
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

add_executable(RMS_avg src/RMS_avg.cpp)
target_link_libraries(RMS_avg ${catkin_LIBRARIES})

add_executable(yawcombine src/yawcombine.cpp)
target_link_libraries(yawcombine ${catkin_LIBRARIES})

add_executable(yaw_vis src/yaw_vis.cpp)
target_link_libraries(yaw_vis ${catkin_LIBRARIES})

add_executable(roomba_node src/roomba_node.cpp)
target_link_libraries(roomba_node ${catkin_LIBRARIES})

add_executable(rebr_rpy src/rebr_rpy.cpp)
target_link_libraries(rebr_rpy ${catkin_LIBRARIES})


#add_executable(bardronev1 src/bardronev1.cpp)
#target_link_libraries(bardronev1 ${catkin_LIBRARIES})

add_dependencies(ardronev1 ${PROJECT_NAME}_gencfg)

# Make a PID library
add_library(pid src/pid.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
