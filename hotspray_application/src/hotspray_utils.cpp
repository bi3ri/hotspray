
//#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
//#include "eigen_conversions/eigen_msg.h" //conversion posemsg -> eigen

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <Eigen/Geometry>


#include <geometry_msgs/PoseArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <trajectory_msgs/JointTrajectory.h>



bool saveTrajectoryToFile(JointTrajectory trajectory){

 rosbag::Bag bag;
    bag.open(path + "/config/traj.bag", rosbag::bagmode::Write);


    std::vector<moveit_msgs::RobotTrajectory> trajectories;
      bag.write("scan_trajectory", ros::Time::now(), my_plan.trajectory_);

}



bool loadTrajectoryFromFile();

bool savePoseArrayToFile();

bool loadPoseArrayFromFile();

bool convertFromNoetherToHotsprayFrame();

visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<geometry_msgs::PoseArray>& pose_arrays,
                                                const std::string& frame_id,
                                                const std::string& ns,
                                                const std::size_t& start_id = 0,
                                                const double& axis_scale = 0.001,
                                                const double& axis_length = 0.03,
                                                const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<Eigen::Isometry3d>& pose_array,
                                                const std::string& frame_id,
                                                const std::string& ns,
                                                const std::size_t& start_id = 0,
                                                const double& axis_scale = 0.001,
                                                const double& axis_length = 0.03,
                                                const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
