
//#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <Eigen/Geometry>
#include "eigen_conversions/eigen_msg.h" //conversion posemsg -> eigen


#include <geometry_msgs/PoseArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <hotspray_utils/hotspray_utils.h>


bool HotsprayUtils::savePoseArrayMsgToJsonFile(geometry_msgs::PoseArray& pose_array, const std::string file_path){
  nlohmann::json json_pose_array = convertToJson(pose_array);
  saveJson(file_path, json_pose_array);
  return 0;
}

bool HotsprayUtils::loadPoseArrayMsgFromJsonFile(geometry_msgs::PoseArray& pose_array, const std::string& file_path){
  nlohmann::json json_pose_array;
  loadJson(file_path, json_pose_array);
  pose_array = convertToPoseArrayMsg(json_pose_array);
  return 0;
}

bool HotsprayUtils::loadPoseArrayMsgFromJsonFile(std::vector<geometry_msgs::PoseArray>& pose_arrays, const std::string& file_path){
  nlohmann::json json_pose_array;
  loadJson(file_path, json_pose_array);
  pose_arrays = convertToVectorPoseArrayMsg(json_pose_array);
  return 0;
}

float HotsprayUtils::euclideanDistance(const geometry_msgs::Pose& pose1,const geometry_msgs::Pose& pose2){
  float distance_x = pose1.position.x - pose2.position.x;
  float distance_y = pose1.position.y - pose2.position.y;
  float distance_z = pose1.position.z - pose2.position.z;
  return std::sqrt(std::pow(distance_x, 2) + std::pow(distance_y, 2) + std::pow(distance_z, 2));
}

bool HotsprayUtils::isSamePose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, const float epsilon){
  if((pose1.position.x - pose2.position.x) < epsilon &&
    (pose1.position.y - pose2.position.y) < epsilon &&
    (pose1.position.z - pose2.position.z) < epsilon &&
    (pose1.orientation.x - pose2.orientation.x) < epsilon &&
    (pose1.orientation.y - pose2.orientation.y) < epsilon &&
    (pose1.orientation.z - pose2.orientation.z) < epsilon &&
    (pose1.orientation.w - pose2.orientation.w) < epsilon)
      return true;
  return false;
}

nlohmann::json HotsprayUtils::convertToJson(geometry_msgs::PoseArray& pose_array){
  nlohmann::json json_pose_array;
  int idx = 1;
  for(const geometry_msgs::Pose& pose : pose_array.poses){
    nlohmann::json json_pose = 
      {
        {"id", idx++},
        {"position",
          {
            {"x", pose.position.x},
            {"y", pose.position.y},
            {"z", pose.position.z}
          }
        },
        {"orientation",
          {
            {"x", pose.orientation.x},
            {"y", pose.orientation.y}, 
            {"z", pose.orientation.z},
            {"w", pose.orientation.w}
          }
        }
      };
  json_pose_array.push_back(json_pose);
  } 
  return json_pose_array;
}

nlohmann::json HotsprayUtils::convertToJson(std::vector<geometry_msgs::PoseArray>& pose_arrays){
  nlohmann::json json_pose_arrays;

  for(auto& pose_array : pose_arrays){

    auto json_pose_array = convertToJson(pose_array);

    json_pose_arrays.push_back(json_pose_array);
  } 
  return json_pose_arrays;
}

nlohmann::json HotsprayUtils::convertToJson(std::vector<Eigen::Isometry3d> eigen_pose_aray){
  geometry_msgs::PoseArray pose_array = convertToPoseArrayMsg(eigen_pose_aray);
  return convertToJson(pose_array);
}

std::vector<Eigen::Isometry3d> HotsprayUtils::convertToEigenPoseArray(nlohmann::json& json_pose_array){
  std::vector<Eigen::Isometry3d> eigen_pose_array;
  for(int i = 0; i < json_pose_array.size(); i++){
    geometry_msgs::Pose pose;
    pose.position.x = json_pose_array[i]["position"]["x"];
    pose.position.y = json_pose_array[i]["position"]["y"];
    pose.position.z = json_pose_array[i]["position"]["z"];
    pose.orientation.x = json_pose_array[i]["orientation"]["x"];
    pose.orientation.y = json_pose_array[i]["orientation"]["y"];
    pose.orientation.z = json_pose_array[i]["orientation"]["z"];
    pose.orientation.w = json_pose_array[i]["orientation"]["w"];

    Eigen::Isometry3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);
    eigen_pose_array.push_back(eigen_pose);
  }
  return eigen_pose_array;
}

std::vector<Eigen::Isometry3d> HotsprayUtils::convertToEigenPoseArray(const geometry_msgs::PoseArray& pose_array){
std::vector<Eigen::Isometry3d> eigen_pose_array;
  for(auto& pose : pose_array.poses){
    Eigen::Isometry3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);
    eigen_pose_array.push_back(eigen_pose);
  }
  return eigen_pose_array;
}

geometry_msgs::PoseArray HotsprayUtils::convertToPoseArrayMsg(nlohmann::json& json_pose_array){
  geometry_msgs::PoseArray pose_array;

  for(int i = 0; i < json_pose_array.size(); i++){
    geometry_msgs::Pose pose;
    pose.position.x = json_pose_array[i]["position"]["x"];
    pose.position.y = json_pose_array[i]["position"]["y"];
    pose.position.z = json_pose_array[i]["position"]["z"];
    pose.orientation.x = json_pose_array[i]["orientation"]["x"];
    pose.orientation.y = json_pose_array[i]["orientation"]["y"];
    pose.orientation.z = json_pose_array[i]["orientation"]["z"];
    pose.orientation.w = json_pose_array[i]["orientation"]["w"];
    pose_array.poses.push_back(pose);
  }

  return pose_array;
}

std::vector<geometry_msgs::PoseArray> HotsprayUtils::convertToVectorPoseArrayMsg(nlohmann::json& json_pose_arrays){
  std::vector<geometry_msgs::PoseArray> pose_arrays;

for (auto& json_pose_array : json_pose_arrays) {
  geometry_msgs::PoseArray pose_array;
    for(int i = 0; i < json_pose_array.size(); i++){
      geometry_msgs::Pose pose;
      pose.position.x = json_pose_array[i]["position"]["x"];
      pose.position.y = json_pose_array[i]["position"]["y"];
      pose.position.z = json_pose_array[i]["position"]["z"];
      pose.orientation.x = json_pose_array[i]["orientation"]["x"];
      pose.orientation.y = json_pose_array[i]["orientation"]["y"];
      pose.orientation.z = json_pose_array[i]["orientation"]["z"];
      pose.orientation.w = json_pose_array[i]["orientation"]["w"];
      pose_array.poses.push_back(pose);
    }
  pose_arrays.push_back(pose_array);
  }
  return pose_arrays;
}

void HotsprayUtils::loadJson(std::string path, nlohmann::json& json_pose_array){
  std::ifstream ifs(path);
  json_pose_array = nlohmann::json::parse(ifs);
}

void HotsprayUtils::saveJson(std::string path, nlohmann::json& json_pose_array){
  std::ofstream of(path);
  of << std::setw(4) << json_pose_array << std::endl;
}

bool HotsprayUtils::saveTrajectoryMsgToBagFile(const trajectory_msgs::JointTrajectory& trajectory, const std::string path){
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    bag.write("trajectory", ros::Time::now(), trajectory);
    bag.close();
    return 0;
}

bool HotsprayUtils::loadTrajectoryMsgFromBagFile(trajectory_msgs::JointTrajectory& trajectory, const std::string& path){
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);

    boost::shared_ptr<trajectory_msgs::JointTrajectory> tmp_traj_ptr;

    for (const rosbag::MessageInstance& m : rosbag::View(bag)) 
    {
        tmp_traj_ptr = m.instantiate<trajectory_msgs::JointTrajectory>();
    }

    trajectory = *tmp_traj_ptr;
    bag.close();
    return 0;
}

bool HotsprayUtils::savePoseArrayMsgToBagFile(const geometry_msgs::PoseArray& pose_array, const std::string& path){
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    bag.write("pose_array", ros::Time::now(), pose_array);
    bag.close();
    return 0;
}

bool HotsprayUtils::loadPoseArrayMsgFromBagFile(geometry_msgs::PoseArray& pose_array, const std::string& path){
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);

    boost::shared_ptr<geometry_msgs::PoseArray> tmp_pose_array;

    for (const rosbag::MessageInstance& m : rosbag::View(bag)) 
    {
        tmp_pose_array = m.instantiate<geometry_msgs::PoseArray>();
    }

    pose_array = *tmp_pose_array;

    bag.close();
    return 0;
}

Eigen::Affine3d HotsprayUtils::create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

void HotsprayUtils::applyTranformationToPoseArray(std::vector<geometry_msgs::PoseArray>& _pose_arrays, double x, double y, double z, double ax, double ay, double az){
    Eigen::Isometry3d eigen_pose;
    //Eigen::Affine3d r = create_rotation_matrix(ax, ay, az);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(x, y, z)));
    //Eigen::Matrix4d m = (t * r).matrix(); 

    Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
    pattern_origin.translation() = Eigen::Vector3d(0.2, 0.2, 0.8);
  
    for(auto& pose_arrys : _pose_arrays)
        {
        for(auto& pose : pose_arrys.poses){
            tf::poseMsgToEigen(pose, eigen_pose);
            std::cout << pose << std::endl;
            // eigen_pose = eigen_pose.translation() + Eigen::Vector3d(0.2, 0.2, 0.8); //* pattern_origin;
            //eigen_pose = eigen_pose * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()); // this flips the tool around so that Z is down
            //eigen_pose.translate(Eigen::Vector3d(0.2, 0.2, 0.8));
            eigen_pose = eigen_pose *  pattern_origin;
            tf::poseEigenToMsg(eigen_pose, pose);
            std::cout << pose << std::endl;

        }
    }

}

void HotsprayUtils::applyTranformationToPoseArray(std::vector<Eigen::Isometry3d>& pose_array, double x, double y, double z, double ax, double ay, double az){
    Eigen::Affine3d r = create_rotation_matrix(ax, ay, az);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(x, y, z)));
    Eigen::Matrix4d m = (t * r).matrix(); 

    for(auto& pose : pose_array)
    {
      pose = pose * m;
    }

}

visualization_msgs::MarkerArray HotsprayUtils::convertToAxisMarkers(const std::vector<Eigen::Isometry3d>& pose_array,
                                                     const std::string& frame_id,
                                                     const std::string& ns,
                                                     const std::size_t& start_id,
                                                     const double& axis_scale,
                                                     const double& axis_length,
                                                    const std::tuple<float, float, float, float, float, float>& offset)
{
  using namespace Eigen;

  visualization_msgs::MarkerArray markers;

  auto create_line_marker = [&](const int id,
                                const std::tuple<float, float, float, float>& rgba) -> visualization_msgs::Marker {
    visualization_msgs::Marker line_marker;
    line_marker.action = line_marker.ADD;
    std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = rgba;
    line_marker.header.frame_id = frame_id;
    line_marker.type = line_marker.LINE_LIST;
    line_marker.id = id;
    line_marker.lifetime = ros::Duration(0);
    line_marker.ns = ns;
    std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(axis_scale, 0.0, 0.0);
    //line_marker.pose = pose3DtoPoseMsg(offset);
    return line_marker;
  };

  // markers for each axis line
  int marker_id = start_id;
  visualization_msgs::Marker x_axis_marker = create_line_marker(marker_id, std::make_tuple(1.0, 0.0, 0.0, 1.0)); //red
  visualization_msgs::Marker y_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 1.0, 0.0, 1.0)); //green
  visualization_msgs::Marker z_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 0.0, 1.0, 1.0)); //blue

  auto add_axis_line = [](const Isometry3d& eigen_pose,
                          const Vector3d& dir,
                          const geometry_msgs::Point& p1,
                          visualization_msgs::Marker& marker) {
    geometry_msgs::Point p2;
    Eigen::Vector3d line_endpoint;

    // axis endpoint
    line_endpoint = eigen_pose * dir;
    std::tie(p2.x, p2.y, p2.z) = std::make_tuple(line_endpoint.x(), line_endpoint.y(), line_endpoint.z());

    // adding line
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  };

  for (const Eigen::Isometry3d& eigen_pose : pose_array)
  {
    geometry_msgs::Point p1;
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg( eigen_pose , pose);

    std::tie(p1.x, p1.y, p1.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);
    
    add_axis_line(eigen_pose, Vector3d::UnitX() * axis_length, p1, x_axis_marker);
    add_axis_line(eigen_pose, Vector3d::UnitY() * axis_length, p1, y_axis_marker);
    add_axis_line(eigen_pose, Vector3d::UnitZ() * axis_length, p1, z_axis_marker);
  }

  markers.markers.push_back(x_axis_marker);
  markers.markers.push_back(y_axis_marker);
  markers.markers.push_back(z_axis_marker);
  return markers;
}

geometry_msgs::PoseArray HotsprayUtils::convertToPoseArrayMsg(const std::vector<Eigen::Isometry3d>& eigen_pose_array){
  geometry_msgs::PoseArray pose_array;

  for(auto& eigen_pose : eigen_pose_array){
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(eigen_pose, pose);
    pose_array.poses.push_back(pose);
  }
  return pose_array;
}

visualization_msgs::MarkerArray HotsprayUtils::convertToAxisMarkers(const std::vector<geometry_msgs::PoseArray>& pose_arrays,
                                                     const std::string& frame_id,
                                                     const std::string& ns,
                                                     const std::size_t& start_id,
                                                     const double& axis_scale,
                                                     const double& axis_length,
                                                    const std::tuple<float, float, float, float, float, float>& offset)
{
  using namespace Eigen;

  visualization_msgs::MarkerArray markers;

  auto create_line_marker = [&](const int id,
                                const std::tuple<float, float, float, float>& rgba) -> visualization_msgs::Marker {
    visualization_msgs::Marker line_marker;
    line_marker.action = line_marker.ADD;
    std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = rgba;
    line_marker.header.frame_id = frame_id;
    line_marker.type = line_marker.LINE_LIST;
    line_marker.id = id;
    line_marker.lifetime = ros::Duration(0);
    line_marker.ns = ns;
    std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(axis_scale, 0.0, 0.0);
    //line_marker.pose = pose3DtoPoseMsg(offset);
    return line_marker;
  };

  // markers for each axis line
  int marker_id = start_id;
  visualization_msgs::Marker x_axis_marker = create_line_marker(marker_id, std::make_tuple(1.0, 0.0, 0.0, 1.0));
  visualization_msgs::Marker y_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 1.0, 0.0, 1.0));
  visualization_msgs::Marker z_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 0.0, 1.0, 1.0));

  auto add_axis_line = [](const Isometry3d& eigen_pose,
                          const Vector3d& dir,
                          const geometry_msgs::Point& p1,
                          visualization_msgs::Marker& marker) {
    geometry_msgs::Point p2;
    Eigen::Vector3d line_endpoint;

    // axis endpoint
    line_endpoint = eigen_pose * dir;
    std::tie(p2.x, p2.y, p2.z) = std::make_tuple(line_endpoint.x(), line_endpoint.y(), line_endpoint.z());

    // adding line
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  };

    for(const geometry_msgs::PoseArray& pose_array : pose_arrays){
      for (const geometry_msgs::Pose& pose : pose_array.poses)
      {
        Eigen::Isometry3d eigen_pose;
        tf::poseMsgToEigen(pose, eigen_pose);

        geometry_msgs::Point p1;
        std::tie(p1.x, p1.y, p1.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);

        add_axis_line(eigen_pose, Vector3d::UnitX() * axis_length, p1, x_axis_marker);
        add_axis_line(eigen_pose, Vector3d::UnitY() * axis_length, p1, y_axis_marker);
        add_axis_line(eigen_pose, Vector3d::UnitZ() * axis_length, p1, z_axis_marker);
      }
    }

  markers.markers.push_back(x_axis_marker);
  markers.markers.push_back(y_axis_marker);
  markers.markers.push_back(z_axis_marker);
  return markers;
}

static Eigen::Matrix3d computeRotation(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz)
{
  Eigen::Matrix3d m;
  m.setIdentity();
  m.col(0) = vx.normalized();
  m.col(1) = vy.normalized();
  m.col(2) = vz.normalized();
  return m;
}

