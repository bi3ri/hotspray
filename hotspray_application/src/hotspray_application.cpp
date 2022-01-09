
#include <iostream>
#include <string>
#include <tf/transform_listener.h>

#include "ros/ros.h"
#include <ros/package.h>

#include <cstdlib> 

#include <control_msgs/FollowJointTrajectoryAction.h>
#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ur_msgs/SetIO.h"
#include "ur_msgs/SetPayload.h"
#include "ur_msgs/SetPayloadRequest.h"
#include "ur_msgs/SetPayloadResponse.h"
#include "ur_msgs/SetIORequest.h"
#include "ur_msgs/SetIOResponse.h"
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/Analog.h"
#include "std_msgs/String.h"

#include <std_msgs/Float64MultiArray.h>
#include <yak_ros_msgs/GenerateMesh.h>
#include <eigen_conversions/eigen_msg.h> 
#include <yak_ros_msgs/GenerateMesh.h>
#include <tubular_toolpath_creator/GenerateTubularToolpath.h>

#include <hotspray_msgs/ExecuteScanTrajectoryAction.h>
#include <hotspray_msgs/ExecuteScanTrajectoryAction.h>
#include <hotspray_msgs/GenerateSprayTrajectory.h>
#include <hotspray_msgs/GenerateScanTrajectory.h>
#include <hotspray_utils/hotspray_utils.h>
#include <hotspray_application.h>

const std::string EXECUTE_TRAJECTORY_ACTION = "execute_trajectory";

HotsprayApplication::HotsprayApplication(ros::NodeHandle nh) :
    nh_(nh),
    ph_("~"),
    spray_client_(nh_.serviceClient<ur_msgs::SetIO>("ur_driver/set_io")),
    mesh_client_(nh_.serviceClient<yak_ros_msgs::GenerateMesh>("/tsdf_node/generate_mesh")),
    scan_trajectory_client_(nh_.serviceClient<hotspray_msgs::GenerateScanTrajectory>("/hotspray_motion/generate_scan_trajectory")),
    spray_trajectory_client_(nh_.serviceClient<hotspray_msgs::GenerateSprayTrajectory>("/hotspray_motion/generate_spray_trajectory")),
    tubular_toolpath_client_(nh_.serviceClient<tubular_toolpath_creator::GenerateTubularToolpath>("/tubular_toolpath_creator/create_tubular_toolpath")),
    scan_pose_publisher_(nh_.advertise<visualization_msgs::MarkerArray>("scan_poses", 0)),
    moveit_action_client_ptr_(std::make_shared<client_type>("execute_trajectory", true)),
    package_path_(ros::package::getPath("hotspray_application"))

{
    ph_.getParam("yak_scan_frame", yak_scan_frame_);
    ph_.getParam("ply_name", ply_name_);
    ph_.getParam("action", action_);
    ph_.getParam("action_file_name", action_file_name_);
}

void HotsprayApplication::createScanPoses(geometry_msgs::PoseArray& scan_pose_array){
    std::cout << "This will create a new scan trajectory." << std::endl;

    tf::TransformListener tf_listener;
    tf::StampedTransform tf;

    while(1){
        std::cout << "Do you want to add the current robot position to the trajectory? (Answer: y/n)" << std::endl;
        char ch;
        do
        {
        ch = std::getchar();
        } while(ch!='y'&& ch!='n' && ch!='Y'&& ch!='N');

        if(ch == 'n' || ch == 'N')
            break;

        try{
            tf_listener.lookupTransform("/world", "/camera_orbit_frame", ros::Time(0), tf);
        }
        catch (tf::TransformException &ex) { 
            throw("Looking up Transfrom world to camera_orbit_frame failed!");
        }
        geometry_msgs::Pose pose;
        tfToPose(tf, pose);
        if(scan_pose_array.poses.size() > 0)
        {
            if(HotsprayUtils::isSamePose(pose, scan_pose_array.poses.back(), 0.0001f))
            {
                std::cout << "The following pose was NOT added to the trajectory! Distance to old pose to small!\n" << std::endl;
                continue;
            }
        }   
        scan_pose_array.poses.push_back(pose);
        std::cout << "The following pose was added to the trajectory!\n" << pose << "\n" << std::endl;
    }   
}

bool HotsprayApplication::generateMesh(){
    yak_ros_msgs::GenerateMesh mesh_srv;
    mesh_srv.request.results_dir =  package_path_ + std::string("/data/meshs");// + ply_name_;
    mesh_srv.request.results_frame = yak_scan_frame_;

    if(mesh_client_.call(mesh_srv))
    {
        ROS_INFO("Call to service generate mesh was successful");
        return 0;
    }
    else
    {
        throw("Failed to call service generate mesh!");
    }
}


bool HotsprayApplication::generateTubularToolpath(std::vector<geometry_msgs::PoseArray>& pose_arrays){
    tubular_toolpath_creator::GenerateTubularToolpath tubular_toolpath_srv;
    tubular_toolpath_srv.request.mesh_path = package_path_ + std::string("/data/meshs") + ply_name_; //TODO YAK name file

    tf::TransformListener tf_listener;
    tf::StampedTransform tf;

    if(tubular_toolpath_client_.call(tubular_toolpath_srv))
    {
        // std::vector<geometry_msgs::PoseArray> 
        // eigen_pose_array = HotsprayUtils::convertToEigenPoseArray(tubular_toolpath_srv.response.toolpath_vector_array);
        pose_arrays = tubular_toolpath_srv.response.raster_array;

        try{
            tf_listener.lookupTransform("/world", yak_scan_frame_, ros::Time(0), tf);
        }
        catch (tf::TransformException &ex) { 
            throw("Looking up Transfrom world to camera_orbit_frame failed!");
        }

        geometry_msgs::Pose transform_pose;
        tfToPose(tf, transform_pose);
        
        for(auto& pose_array : pose_arrays){
            for(auto& pose : pose_array.poses){
                Eigen::Isometry3d eigen_pose;
                tf::poseMsgToEigen(pose, eigen_pose);
                eigen_pose = eigen_pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
                eigen_pose.pretranslate(Eigen::Vector3d(transform_pose.position.x, transform_pose.position.y, transform_pose.position.z));
                tf::poseEigenToMsg(eigen_pose, pose);
            }
        }
        ROS_INFO("Call to generate tubular toolpath was successfull!");
        return 0;
    }
    else
    {
        throw("Failed to call service generate tubular toolpath!");
    }

    return -1;
}

bool HotsprayApplication::generateScanTrajectory(geometry_msgs::PoseArray pose_array, trajectory_msgs::JointTrajectory& trajectory){
    hotspray_msgs::GenerateScanTrajectory trajectory_srv;

    pose_array.header.frame_id = "world";
    trajectory_srv.request.pose_array = pose_array;

    if(scan_trajectory_client_.call(trajectory_srv))
    {
        trajectory = trajectory_srv.response.traj;
        ROS_INFO("Call to generate trajectory was successfull!");
        return 0;
    }
    else
    {
        throw("Failed to call service hotspray_motion");
    }
}

bool HotsprayApplication::generateSprayTrajectory(std::vector<geometry_msgs::PoseArray> pose_arrays, trajectory_msgs::JointTrajectory& trajectory){
    hotspray_msgs::GenerateSprayTrajectory trajectory_srv;

    for(auto& pose_array : pose_arrays){
        pose_array.header.frame_id = "world";
        trajectory_srv.request.raster_array.push_back(pose_array);
    }

    if(spray_trajectory_client_.call(trajectory_srv))
    {
        trajectory = trajectory_srv.response.traj;
        ROS_INFO("Call to generate trajectory was successfull!");
        return 0;
    }
    else
    {
        throw("Failed to call service hotspray_motion");
    }
}

bool HotsprayApplication::executeTrajectory(trajectory_msgs::JointTrajectory& trajectory){

    if(moveit_action_client_ptr_->waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO_STREAM("Connected to '"<<EXECUTE_TRAJECTORY_ACTION<<"' action");
    }
    else
    {
        throw("Failed to connect to" + EXECUTE_TRAJECTORY_ACTION+ " action");
    }
    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory.joint_trajectory = trajectory;
    std::vector<std::string> joint_names;
    nh_.getParam("/arm_controller/joints", joint_names);
    goal.trajectory.joint_trajectory.joint_names = joint_names;

    moveit_action_client_ptr_->sendGoalAndWait(goal);
    if(moveit_action_client_ptr_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Execution of trajectory was successfull!"); //TODO
        return 0;
    }else{
        throw("Execution of trajectory failed!");
    }
    return 0;
}

bool HotsprayApplication::startSpray(){
    ur_msgs::SetIO set_io;
    set_io.request.pin = 1; // pin
    set_io.request.state = 1; // state
    
    // set_io.request.FUN_SET_DIGITAL_OUT = 1; //state
    // set_io.request.FUN_SET_ANALOG_OUT = 3;

    if(spray_client_.call(set_io))
    {
        ROS_INFO("Call to service UR driver start spray was successful");
        return 0;
    }
    else
    {
        throw("Failed to call service UR driver start spray!");
    }
}

bool HotsprayApplication::stopSpray(){
    ur_msgs::SetIO set_io;
    set_io.request.fun = 1; // pin
    set_io.request.pin = 1; // pin
    set_io.request.state = 1; // state
    // set_io.request.FUN_SET_DIGITAL_OUT = 0; //state
    // set_io.request.FUN_SET_ANALOG_OUT = 0;

    if(spray_client_.call(set_io))
    {
        ROS_INFO("Call to service UR driver stop spray was successful");
        return 0;
    }
    else
    {
        throw("Failed to call service UR driver stop spray!");
    }
}

void HotsprayApplication::tfToPose(const tf::StampedTransform& tf, geometry_msgs::Pose& pose){
        pose.position.x = tf.getOrigin().x();
        pose.position.y = tf.getOrigin().y();
        pose.position.z = tf.getOrigin().z();
        pose.orientation.w = tf.getRotation().getW();
        pose.orientation.x = tf.getRotation().getX();
        pose.orientation.y = tf.getRotation().getY();
        pose.orientation.z = tf.getRotation().getZ();
}



bool HotsprayApplication::run()
{
    trajectory_msgs::JointTrajectory scan_trajectory;
    trajectory_msgs::JointTrajectory spray_trajectory;

    geometry_msgs::PoseArray scan_pose_array;
    std::vector<geometry_msgs::PoseArray> spray_raster_array;
    std::string scan_json_pose_array_path = package_path_ + "/data/scan/" + action_file_name_ + ".poses.json";
    std::string spray_json_raster_array_path = package_path_ + "/data/spray/" + action_file_name_ + ".poses.json";
    std::string scan_bag_trajectory_path = package_path_ + "/data/scan/" + action_file_name_ + ".trajectory.bag";
    std::string spray_bag_trajectory_path = package_path_ + "/data/spray/" + action_file_name_ + ".trajectory.bag";

    nlohmann::json json_pose_array;

    try{
        if(action_ == "create_scan_poses")
        {
            createScanPoses(scan_pose_array);
            HotsprayUtils::savePoseArrayMsgToJsonFile(scan_pose_array, scan_json_pose_array_path);

            ROS_INFO("\nSucessfully created and saved scan poses!");
            return 0;

        }else if(action_ == "load_scan_poses")
        {
            HotsprayUtils::loadJson(scan_json_pose_array_path, json_pose_array);
            scan_pose_array = HotsprayUtils::convertToPoseArrayMsg(json_pose_array);

            visualization_msgs::MarkerArray scan_markers;
            std::vector<geometry_msgs::PoseArray> scan_pose_arrays = {scan_pose_array};
            scan_markers = HotsprayUtils::convertToAxisMarkers(scan_pose_arrays, "world", "scan_poses");
            scan_pose_publisher_.publish(scan_markers);
            generateScanTrajectory(scan_pose_array, scan_trajectory);
            HotsprayUtils::saveTrajectoryMsgToBagFile(scan_trajectory, scan_bag_trajectory_path);

            executeTrajectory(scan_trajectory);

            generateMesh();

            generateTubularToolpath(spray_raster_array);

            nlohmann::json spray_json_raster_array = HotsprayUtils::convertToJson(spray_raster_array);
            HotsprayUtils::saveJson(spray_json_raster_array_path, spray_json_raster_array);

            auto spray_markers = HotsprayUtils::convertToAxisMarkers(spray_raster_array, "world", "spray_poses");
            scan_pose_publisher_.publish(spray_markers);

            generateSprayTrajectory(spray_raster_array, spray_trajectory);
            HotsprayUtils::saveTrajectoryMsgToBagFile(spray_trajectory, spray_bag_trajectory_path);

            executeTrajectory(spray_trajectory);

            return 0;

        }else if(action_ == "load_scan_trajectory")
        {
            HotsprayUtils::loadPoseArrayMsgFromJsonFile(scan_pose_array, scan_json_pose_array_path);
            HotsprayUtils::loadTrajectoryMsgFromBagFile(scan_trajectory, action_file_name_);

            std::vector<geometry_msgs::PoseArray> scan_pose_arrays = {scan_pose_array};
            visualization_msgs::MarkerArray scan_markers;
            scan_markers = HotsprayUtils::convertToAxisMarkers(scan_pose_arrays, "world", "scan_poses");
            scan_pose_publisher_.publish(scan_markers);

            executeTrajectory(scan_trajectory);

            generateMesh();

            generateTubularToolpath(spray_raster_array);

            nlohmann::json spray_json_raster_array = HotsprayUtils::convertToJson(spray_raster_array);
            HotsprayUtils::saveJson(spray_json_raster_array_path, spray_json_raster_array);

            auto spray_markers = HotsprayUtils::convertToAxisMarkers(spray_raster_array, "world", "spray_poses");
            scan_pose_publisher_.publish(spray_markers);

            generateSprayTrajectory(spray_raster_array, spray_trajectory);
            HotsprayUtils::saveTrajectoryMsgToBagFile(spray_trajectory, spray_bag_trajectory_path);

            executeTrajectory(spray_trajectory);
            return 0;
        }else if(action_ == "create_spray_poses")
        {
            generateTubularToolpath(spray_raster_array);
            
            nlohmann::json spray_json_raster_array = HotsprayUtils::convertToJson(spray_raster_array);
            HotsprayUtils::saveJson(spray_json_raster_array_path, spray_json_raster_array);

            auto spray_markers = HotsprayUtils::convertToAxisMarkers(spray_raster_array, "world", "spray_poses");
            scan_pose_publisher_.publish(spray_markers);

            generateSprayTrajectory(spray_raster_array, spray_trajectory);
            HotsprayUtils::saveTrajectoryMsgToBagFile(spray_trajectory, spray_bag_trajectory_path);

            executeTrajectory(spray_trajectory);
            return 0;
        }else if(action_ == "load_spray_poses")
        {
            HotsprayUtils::loadPoseArrayMsgFromJsonFile(spray_raster_array, spray_json_raster_array_path);

            visualization_msgs::MarkerArray spray_markers;
            spray_markers = HotsprayUtils::convertToAxisMarkers(spray_raster_array, "world", "scan_poses");
            scan_pose_publisher_.publish(spray_markers);

            generateSprayTrajectory(spray_raster_array, spray_trajectory);
            HotsprayUtils::saveTrajectoryMsgToBagFile(spray_trajectory, spray_bag_trajectory_path);

            executeTrajectory(spray_trajectory);
            return 0;
        }else if(action_ == "load_spray_trajectory")
        {
            HotsprayUtils::loadPoseArrayMsgFromJsonFile(spray_raster_array, spray_json_raster_array_path);
            HotsprayUtils::loadTrajectoryMsgFromBagFile(spray_trajectory, spray_bag_trajectory_path);

            visualization_msgs::MarkerArray spray_markers;
            spray_markers = HotsprayUtils::convertToAxisMarkers(spray_raster_array, "world", "spray_poses");
            scan_pose_publisher_.publish(spray_markers);

            executeTrajectory(spray_trajectory);
            return 0;
        }
    }catch(const char* msg)
    {
        ROS_ERROR_STREAM(msg);
    }
    
        throw("Hotspray action %s is not supported!", action_file_name_.c_str());
    return -1;
}

