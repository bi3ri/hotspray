#pragma once
#include <ros/ros.h>

#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include "hotspray_msgs/GenerateScanTrajectory.h"
#include <hotspray_utils/hotspray_utils.h>

#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include <visualization_msgs/MarkerArray.h>
#include "eigen_conversions/eigen_msg.h" 
#include <tesseract_common/macros.h>
#include <ros/ros.h>

//#include <tesseract_ros_examples/freespace_hybrid_example.h>
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
//#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

// Core ros functionality like ros::init and spin
#include <tesseract_rosutils/plotting.h>

class HotsprayMotionServer
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle ph_;
        ros::ServiceServer plan_scan_trajectory_service_;
        ros::ServiceServer plan_spray_trajectory_service_;
        ros::Publisher vis_pub_;
        tesseract_environment::Environment::Ptr env_;           
        tesseract_monitoring::EnvironmentMonitor::Ptr monitor_; 
        tesseract_planning::ManipulatorInfo mi_;
        tesseract_rosutils::ROSPlottingPtr plotter_;
        Eigen::VectorXd home_joint_pos_;
        std::vector<std::string> joint_names_;

        bool debug_;

    public: 
        HotsprayMotionServer(ros::NodeHandle); //, std::string config_path);

        bool generateScanTrajectory(hotspray_msgs::GenerateScanTrajectory::Request &req, hotspray_msgs::GenerateScanTrajectory::Response &res);
        
        bool generateSprayTrajectory(hotspray_msgs::GenerateSprayTrajectory::Request &req, hotspray_msgs::GenerateSprayTrajectory::Response &res);

        tesseract_planning::CompositeInstruction createScanProgram(const geometry_msgs::PoseArray & _pose_arrys, const tesseract_planning::ManipulatorInfo& manipulator);

        tesseract_planning::CompositeInstruction createSprayProgram(const std::vector<geometry_msgs::PoseArray, std::allocator<geometry_msgs::PoseArray>> & _pose_arrys,const tesseract_planning::ManipulatorInfo& manipulator);

        void toMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj);

    tesseract_common::VectorIsometry3d sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                    double z_freedom,
                                                    double rx_freedom,
                                                    double ry_freedom,
                                                    double rz_freedom,
                                                    double z_resolution,
                                                    double rx_resolution,
                                                    double ry_resolution,
                                                    double rz_resolution
                                                    );

};