#pragma once
//#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <Eigen/Geometry>
#include "eigen_conversions/eigen_msg.h" //conversion posemsg -> eigen


#include <geometry_msgs/PoseArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <hotspray_utils/json.hpp>

namespace HotsprayUtils{

float euclideanDistance(const geometry_msgs::Pose& pose1,const geometry_msgs::Pose& pose2);

bool savePoseArrayMsgToJsonFile(geometry_msgs::PoseArray& pose_array, const std::string file_path);

bool loadPoseArrayMsgFromJsonFile(geometry_msgs::PoseArray& pose_array, const std::string& file_path);

bool saveTrajectoryMsgToBagFile(const trajectory_msgs::JointTrajectory& trajectory, const std::string name);

bool loadTrajectoryMsgFromBagFile(trajectory_msgs::JointTrajectory& trajectory, const std::string& name);

bool savePoseArrayMsgToBagFile(const geometry_msgs::PoseArray& pose_array, const std::string& name);

bool loadPoseArrayMsgFromBagFile(geometry_msgs::PoseArray& pose_array, const std::string& name);

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az);

void applyTranformationToPoseArray(std::vector<geometry_msgs::PoseArray>& _pose_arrays, double x, double y, double z, double ax, double ay, double az);

void applyTranformationToPoseArray(std::vector<Eigen::Isometry3d>& pose_array, double x, double y, double z, double ax, double ay, double az);

visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<Eigen::Isometry3d>& pose_array,
                                                     const std::string& frame_id,
                                                     const std::string& ns,
                                                      const std::size_t& start_id = 0,
                                                      const double& axis_scale = 0.001,
                                                      const double& axis_length = 0.01,
                                                      const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<geometry_msgs::PoseArray>& pose_arrays,
                                                     const std::string& frame_id,
                                                     const std::string& ns,
                                                      const std::size_t& start_id = 0,
                                                      const double& axis_scale = 0.001,
                                                      const double& axis_length = 0.01,
                                                      const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

nlohmann::json convertToJson(geometry_msgs::PoseArray& pose_array);

nlohmann::json convertToJson(std::vector<Eigen::Isometry3d> eigen_pose_aray);

std::vector<Eigen::Isometry3d> convertToEigenPoseArray(geometry_msgs::PoseArray& pose_array);

std::vector<Eigen::Isometry3d> convertToEigenPoseArray(nlohmann::json& json_pose_array);

std::vector<Eigen::Isometry3d> convertToEigenPoseArray(const std_msgs::Float64MultiArray& response_array);

geometry_msgs::PoseArray convertToPoseArrayMsg(nlohmann::json& json_pose_array);

geometry_msgs::PoseArray convertToPoseArrayMsg(const std::vector<Eigen::Isometry3d>& eigen_pose_array);

geometry_msgs::PoseArray convertToPoseArrayMsg(const std_msgs::Float64MultiArray& response_array);

void loadJson(std::string path, nlohmann::json& json_pose_array);

void saveJson(std::string path, nlohmann::json& json_pose_array);

}//namespace HotsprayUtils