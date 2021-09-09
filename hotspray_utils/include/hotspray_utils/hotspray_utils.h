
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


bool saveTrajectoryToFile(const trajectory_msgs::JointTrajectory& trajectory, const std::string name);

bool loadTrajectoryFromFile(trajectory_msgs::JointTrajectory& trajectory, const std::string& name);


bool savePoseArrayToFile(const geometry_msgs::PoseArray& pose_array, const std::string& name);


bool loadPoseArrayFromFile(geometry_msgs::PoseArray& pose_array, const std::string& name);

void convertResponseArrayToPoseArray(const std_msgs::Float64MultiArray& response_array, std::vector<Eigen::Isometry3d>& pose_array);

void convertResponseArrayToPoseArray(const std_msgs::Float64MultiArray& response_array, std::vector<geometry_msgs::PoseArray> &pose_array);


Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az);

void applyTranformationToPoseArray(std::vector<geometry_msgs::PoseArray>& _pose_arrays, double x, double y, double z, double ax, double ay, double az);

visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<Eigen::Isometry3d>& pose_array,
                                                     const std::string& frame_id,
                                                     const std::string& ns,
                                                      const std::size_t& start_id = 1,
                                                      const double& axis_scale = 0.001,
                                                      const double& axis_length = 0.03,
                                                      const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));



visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<geometry_msgs::PoseArray>& pose_arrays,
                                                     const std::string& frame_id,
                                                     const std::string& ns,
                                                      const std::size_t& start_id = 1,
                                                      const double& axis_scale = 0.001,
                                                      const double& axis_length = 0.03,
                                                      const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

void convertJsonToPoseArrayMsg(nlohmann::json& json_pose_array, geometry_msgs::PoseArray& pose_array);

void convertPoseArrayMsgToJson(geometry_msgs::PoseArray& pose_array, nlohmann::json& json_pose_array);

void loadJson(std::string path, nlohmann::json& json_pose_array);

void saveJson(std::string path, nlohmann::json& json_pose_array);
/*
visualization_msgs::MarkerArray
HotsparyMotion::convertToArrowMarkers(const noether_msgs::ToolPaths& toolpaths,
                      const std::string& frame_id,
                      const std::string& ns,
                      const std::size_t start_id,
                      const float arrow_diameter,
                      const float point_size,
                      const std::tuple<float, float, float, float, float, float>& offset)
{
  visualization_msgs::MarkerArray markers_msgs;
  visualization_msgs::Marker arrow_marker, points_marker;
  const geometry_msgs::Pose pose_msg = pose3DtoPoseMsg(offset);

  // configure arrow marker
  arrow_marker.action = arrow_marker.ADD;
  std::tie(arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b, arrow_marker.color.a) =
      std::make_tuple(1.0, 1.0, 0.2, 1.0);
  arrow_marker.header.frame_id = frame_id;
  arrow_marker.type = arrow_marker.ARROW;
  arrow_marker.id = start_id;
  arrow_marker.lifetime = ros::Duration(0);
  arrow_marker.ns = ns;
  std::tie(arrow_marker.scale.x, arrow_marker.scale.y, arrow_marker.scale.z) =
      std::make_tuple(arrow_diameter, 4.0 * arrow_diameter, 4.0 * arrow_diameter);

  // configure point marker
  points_marker = arrow_marker;
  points_marker.type = points_marker.POINTS;
  points_marker.ns = ns;
  points_marker.pose = pose_msg;
  std::tie(points_marker.color.r, points_marker.color.g, points_marker.color.b, points_marker.color.a) =
      std::make_tuple(0.1, .8, 0.2, 1.0);
  std::tie(points_marker.scale.x, points_marker.scale.y, points_marker.scale.z) =
      std::make_tuple(point_size, point_size, point_size);

  auto transformPoint = [](const geometry_msgs::Pose& pose_msg,
                           const geometry_msgs::Point& p_msg) -> geometry_msgs::Point {
    auto new_p_msg = p_msg;
    Eigen::Isometry3d transform;
    Eigen::Vector3d p, new_p;
    tf::poseMsgToEigen(pose_msg, transform);
    tf::pointMsgToEigen(p_msg, p);
    new_p = transform * p;
    tf::pointEigenToMsg(new_p, new_p_msg);
    return new_p_msg;
  };

  int id_counter = static_cast<int>(start_id);
  for (const noether_msgs::ToolPath& tool_path : toolpaths.paths)
  {
    for (const geometry_msgs::PoseArray& segment : tool_path.segments)
    {
      points_marker.points.clear();
      points_marker.points.push_back(segment.poses.front().position);
      for (std::size_t i = 1; i < segment.poses.size(); i++)
      {
        arrow_marker.points.clear();
        geometry_msgs::Point p_start = transformPoint(pose_msg, segment.poses[i - 1].position);
        geometry_msgs::Point p_end = transformPoint(pose_msg, segment.poses[i].position);
        arrow_marker.points.push_back(p_start);
        arrow_marker.points.push_back(p_end);
        arrow_marker.id = (++id_counter);
        markers_msgs.markers.push_back(arrow_marker);
      }
      points_marker.points.push_back(segment.poses.back().position);

      points_marker.id = (++id_counter);
      markers_msgs.markers.push_back(points_marker);
    }
  }


  return markers_msgs;
}

  */

}//namespace HotsprayUtils