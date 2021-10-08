cmake_minimum_required(VERSION 2.8.3)
project(hotspray_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  pcl_msgs
  std_msgs
  roscpp
  shape_msgs
  noether_msgs
  trajectory_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#    FilteringConfig.msg
#    PlaneSlicerRasterGeneratorConfig.msg
#    SurfaceWalkRasterGeneratorConfig.msg
#    EigenValueEdgeGeneratorConfig.msg
#    HalfedgeEdgeGeneratorConfig.msg
#    SegmentationConfig.msg
#    ToolPathConfig.msg
#    ToolPath.msg
#    ToolPaths.msg
#)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
    ExecuteScanTrajectory.action

)
#Generate service files in the 'srv' folder
add_service_files(
 FILES
    GenerateToolpath.srv
    GenerateSprayTrajectory.srv
    GenerateScanTrajectory.srv
)
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    pcl_msgs
    std_msgs
    noether_msgs
    shape_msgs
    trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    pcl_msgs
    std_msgs
    noether_msgs
    shape_msgs
    trajectory_msgs
)
