#include <iostream>
#include <string>
#include <tf/transform_listener.h>

#include "ros/ros.h"
#include <ros/package.h>

#include "hotspray_msgs/ExecuteScanTrajectoryAction.h"
#include "hotspray_msgs/GenerateToolpath.h"
#include <actionlib/client/simple_action_client.h>

#include <cstdlib> //service lib
#include "yak_ros_msgs/GenerateMesh.h"
#include "noether_msgs/ToolPaths.h"
#include "noether_msgs/ToolPath.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "moveit_msgs/ExecuteTrajectoryAction.h"


#include <visualization_msgs/MarkerArray.h> //debug


#include <hotspray_utils/hotspray_utils.h>
#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include "tubular_toolpath_creator/GenerateTubularToolpath.h"



// typedef actionlib::SimpleActionClient<hotspray_msgs::ExecuteScanTrajectoryAction> Client;

typedef std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> moveit_action_client_ptr;
typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client_type;


class HotsprayApplication {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::ServiceClient mesh_client_;
    ros::ServiceClient toolpath_client_;
    ros::ServiceClient trajectory_client_;
    ros::ServiceClient tubular_toolpath_client_;
    moveit_action_client_ptr moveit_action_client_ptr_;

    std::string mesh_path_;
    ros::Publisher scan_pose_publisher_; 




public:
HotsprayApplication(ros::NodeHandle nh);

bool createScanTrajectory(trajectory_msgs::JointTrajectory& trajectory);

bool generateMesh();

bool generateToolpath(std::vector<geometry_msgs::PoseArray>& pose_arrays);

bool generateTubularToolpath(std::vector<geometry_msgs::PoseArray>& pose_arrays);

bool generateTrajectory(std::vector<geometry_msgs::PoseArray>& pose_arrays, trajectory_msgs::JointTrajectory& trajectory);

bool executeTrajectory(trajectory_msgs::JointTrajectory& trajectory);

void noetherMsgtoPoseArrayMsg(std::vector<noether_msgs::ToolPaths>& raster_paths, std::vector<geometry_msgs::PoseArray>& pose_arrays);

void tfToPose(const tf::StampedTransform& tf, geometry_msgs::Pose& pose);

bool run();

};

