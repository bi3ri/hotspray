cmake_minimum_required(VERSION 2.8.3)
project(hotspray_application)
add_definitions(-std=c++11)

#TODO this should not be needed
find_package(rosbag REQUIRED) 

find_package(catkin REQUIRED COMPONENTS
rosbag
interactive_markers
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
moveit_ros_perception
rviz_visual_tools
moveit_visual_tools
pluginlib
geometric_shapes
pcl_ros
pcl_conversions
std_msgs

roslib
roscpp
std_msgs
tf2_ros
tf2_eigen
tf2_geometry_msgs
hotspray_msgs
yak_ros_msgs
hotspray_utils
)

find_package( Eigen3 REQUIRED )
find_package( yaml-cpp REQUIRED )

find_package( Boost REQUIRED system filesystem date_time thread)

catkin_metapackage()

catkin_package(
  LIBRARIES
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    message_generation
    roscpp
    roslib
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    hotspray_utils
    roscpp
    rosbag
    std_msgs
  DEPENDS 
    Eigen3 
    yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(hotspray_application_node
  src/hotspray_application_node.cpp
  src/hotspray_application.cpp
  include/hotspray_application.hpp
)



target_link_libraries(hotspray_application_node
  ${catkin_LIBRARIES}
  #TODO this should not be needed
  ${rosbag_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(hotspray_application_node ${catkin_EXPORTED_TARGETS})


install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(
  DIRECTORY config launch data results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
