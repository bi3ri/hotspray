#include <ros/ros.h>


//#include <hotspray_motion/include/hotspray_motion_config.hpp>
#include "hotspray_msgs/GenerateSprayTrajectory.h"


#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include <visualization_msgs/MarkerArray.h>
#include "eigen_conversions/eigen_msg.h" //conversion posemsg -> eigen

#include <tesseract_common/macros.h>
//TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
//TESSERACT_COMMON_IGNORE_WARNINGS_POP

//#include <tesseract_ros_examples/freespace_hybrid_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_environment/core/commands.h>
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



class HotsprayMotionServer
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle ph_;
        ros::ServiceServer plan_trajectory_service_;
        ros::Publisher vis_pub_;
        //visualization_msgs::MarkerArray toolpath_markers_;
        //hotspay_motion_config::pathPlanningConfig m_path_planning_config;
        tesseract_environment::Environment::Ptr env_;           /**< @brief Tesseract Manager Class */
        bool rviz_;
        bool plotting_;
        tesseract_monitoring::EnvironmentMonitor::Ptr monitor_; /**< @brief Tesseract Monitor */
        double range_;
        double planning_time_;
        Eigen::VectorXd home_joint_pos_;
        std::vector<std::string> joint_names_;


    public: 
        HotsprayMotionServer(ros::NodeHandle); //, std::string config_path);

        bool generateScanTrajectory(hotspray_msgs::GenerateSprayTrajectory::Request &req, hotspray_msgs::GenerateSprayTrajectory::Response &res);

        bool generateSprayTrajectory(hotspray_msgs::GenerateSprayTrajectory::Request &req,
                hotspray_msgs::GenerateSprayTrajectory::Response &res);

        void createProgramm(tesseract_planning::CompositeInstruction& program, const std::vector<geometry_msgs::PoseArray, std::allocator<geometry_msgs::PoseArray>> & _pose_arrys);


        /**
         * @brief Generate a JointTrajectory Message that contains only trajectory joints
         * @param traj_msg The output JointTrajectory Message
         * @param joint_names The joint names corresponding to the trajectory
         * @param traj The joint trajectory
         */
        void toMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj);


};