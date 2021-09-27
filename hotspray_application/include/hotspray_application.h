#pragma once

#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>

#include <cstdlib> 

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/StdVector>

#include <std_msgs/Float64MultiArray.h>
#include <yak_ros_msgs/GenerateMesh.h>
#include <eigen_conversions/eigen_msg.h> 
#include <yak_ros_msgs/GenerateMesh.h>
#include <tubular_toolpath_creator/GenerateTubularToolpath.h>

#include <hotspray_msgs/ExecuteScanTrajectoryAction.h>
#include <hotspray_msgs/GenerateToolpath.h>
#include <hotspray_msgs/ExecuteScanTrajectoryAction.h>
#include <hotspray_msgs/GenerateSprayTrajectory.h>

typedef std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>> moveit_action_client_ptr;
typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client_type;

class HotsprayApplication {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::ServiceClient mesh_client_;
    ros::ServiceClient toolpath_client_;
    ros::ServiceClient scan_trajectory_client_;
    ros::ServiceClient spray_trajectory_client_;
    ros::ServiceClient tubular_toolpath_client_;
    moveit_action_client_ptr moveit_action_client_ptr_;

    std::string package_path_;

    ros::Publisher scan_pose_publisher_; 

    std::string yak_scan_frame_;
    std::string ply_name_;

    std::string action_;
    std::string action_file_name_;


public:
HotsprayApplication(ros::NodeHandle nh);

void createScanPoses(geometry_msgs::PoseArray& pose_array);

bool generateMesh();

bool generateToolpath(std::vector<geometry_msgs::PoseArray>& pose_arrays);

bool generateTubularToolpath(std::vector<Eigen::Isometry3d>& eigen_pose_array);

bool generateTubularToolpath(std::vector<geometry_msgs::PoseArray>& pose_array);

bool generateScanTrajectory(std::vector<geometry_msgs::PoseArray>& pose_arrays, trajectory_msgs::JointTrajectory& trajectory);

bool generateSprayTrajectory(std::vector<geometry_msgs::PoseArray>& pose_arrays, trajectory_msgs::JointTrajectory& trajectory);

bool executeTrajectory(trajectory_msgs::JointTrajectory& trajectory);

void noetherMsgtoPoseArrayMsg(std::vector<noether_msgs::ToolPaths>& raster_paths, std::vector<geometry_msgs::PoseArray>& pose_arrays);

void tfToPose(const tf::StampedTransform& tf, geometry_msgs::Pose& pose);

bool run();

};

