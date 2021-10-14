#include <ros/ros.h>


//#include <hotspray_motion/include/hotspray_motion_config.hpp>
#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include "hotspray_msgs/GenerateScanTrajectory.h"


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

#include <hotspray_utils/hotspray_utils.h>




// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;
class HotsprayMotionServer
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle ph_;
        ros::ServiceServer plan_scan_trajectory_service_;
        ros::ServiceServer plan_spray_trajectory_service_;



        ros::Publisher vis_pub_;
        //visualization_msgs::MarkerArray toolpath_markers_;
        tesseract_environment::Environment::Ptr env_;           /**< @brief Tesseract Manager Class */
        bool rviz_;
        bool plotting_;
        tesseract_monitoring::EnvironmentMonitor::Ptr monitor_; /**< @brief Tesseract Monitor */
        double range_;
        double planning_time_;
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
                                                  double rz_resolution,
                                                  std::vector<Eigen::Isometry3d>& eigen_samples
                                                  );

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPose(const geometry_msgs::Pose& pose);



descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose);

trajectory_msgs::JointTrajectory createDescartesTrajectory(geometry_msgs::PoseArray& pose_array);

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);


trajectory_msgs::JointTrajectory toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay);
};