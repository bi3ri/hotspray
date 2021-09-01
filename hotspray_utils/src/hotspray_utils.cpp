
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

bool HotsprayUtils::saveTrajectoryToFile(const trajectory_msgs::JointTrajectory& trajectory, const std::string name){
    std::string path = ros::package::getPath("hotspray_application");
    rosbag::Bag bag;
    bag.open(path + "/trajectories/" + name + ".bag", rosbag::bagmode::Write);
    bag.write(name, ros::Time::now(), trajectory);
    bag.close();
    return 0;
}

bool HotsprayUtils::loadTrajectoryFromFile(trajectory_msgs::JointTrajectory& trajectory, const std::string& name){
    std::string path = ros::package::getPath("hotspray_application");
    rosbag::Bag bag;
    bag.open(path + "/trajectories/" + name + ".trajectory.bag", rosbag::bagmode::Read);

    boost::shared_ptr<trajectory_msgs::JointTrajectory> tmp_traj_ptr;

    for (const rosbag::MessageInstance& m : rosbag::View(bag)) //TODO: keine schleife?
    {
        tmp_traj_ptr = m.instantiate<trajectory_msgs::JointTrajectory>();
    }

    trajectory = *tmp_traj_ptr;
    bag.close();
    return 0;
}

bool HotsprayUtils::savePoseArrayToFile(const geometry_msgs::PoseArray& pose_array, const std::string& name){
    std::string path = ros::package::getPath("hotspray_application");
    rosbag::Bag bag;
    bag.open(path + "/poses/" + name + ".posearray.bag", rosbag::bagmode::Write);
    bag.write(name, ros::Time::now(), pose_array);
    bag.close();
    return 0;
}

bool HotsprayUtils::loadPoseArrayFromFile(geometry_msgs::PoseArray& pose_array, const std::string& name){
    std::string path = ros::package::getPath("hotspray_application");
    rosbag::Bag bag;
    bag.open(path + "/poses/" + name + ".posearray.bag", rosbag::bagmode::Read);

    boost::shared_ptr<geometry_msgs::PoseArray> tmp_pose_array;

    for (const rosbag::MessageInstance& m : rosbag::View(bag)) //TODO: keine schleife?
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
  visualization_msgs::Marker x_axis_marker = create_line_marker(++marker_id, std::make_tuple(1.0, 0.0, 0.0, 1.0)); //red
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
  visualization_msgs::Marker x_axis_marker = create_line_marker(++marker_id, std::make_tuple(1.0, 0.0, 0.0, 1.0));
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

void HotsprayUtils::convertResponseArrayToPoseArray(const std_msgs::Float64MultiArray& response_array, std::vector<Eigen::Isometry3d>& pose_array){

    int number_of_poses = response_array.data.size() / 12;
        
    std::cout << "number of poses" << number_of_poses << std::endl;

    geometry_msgs::PoseArray pose_array_;

    int idx = 0;

    for(int i = 0; i < number_of_poses; i++){
        geometry_msgs::Pose pose;
        Eigen::Isometry3d eigen_pose;

        Eigen::Vector3d p, vx, vy, vz;

        idx = i * 12;

        p.data()[0] = response_array.data[idx+0];
        p.data()[1] = response_array.data[idx+1];
        p.data()[2] = response_array.data[idx+2];

        vx.data()[0] = response_array.data[idx+3];
        vx.data()[1] = response_array.data[idx+4];
        vx.data()[2] = response_array.data[idx+5];

        vy.data()[0] = response_array.data[idx+6];
        vy.data()[1] = response_array.data[idx+7];
        vy.data()[2] = response_array.data[idx+8];

        vz.data()[0] = response_array.data[idx+9];
        vz.data()[1] = response_array.data[idx+10];
        vz.data()[2] = response_array.data[idx+11];

        eigen_pose = Eigen::Translation3d(p) * Eigen::AngleAxisd(computeRotation(vx, vy, vz));

        pose_array.push_back(eigen_pose);
    }

}

void HotsprayUtils::convertResponseArrayToPoseArray(const std_msgs::Float64MultiArray& response_array, std::vector<geometry_msgs::PoseArray> &pose_array){

    int number_of_poses = response_array.data.size() / 12;
        
    std::cout << "number of poses" << number_of_poses << std::endl;

    geometry_msgs::PoseArray pose_array_;

    int idx = 0;

    for(int i = 0; i < number_of_poses; i++){
        geometry_msgs::Pose pose;
        Eigen::Isometry3d eigen_pose;

        Eigen::Vector3d p, vx, vy, vz;

        idx = i * 12;

        p.data()[0] = response_array.data[idx+0];
        p.data()[1] = response_array.data[idx+1];
        p.data()[2] = response_array.data[idx+2];

        vx.data()[0] = response_array.data[idx+3];
        vx.data()[1] = response_array.data[idx+4];
        vx.data()[2] = response_array.data[idx+5];

        vy.data()[0] = response_array.data[idx+6];
        vy.data()[1] = response_array.data[idx+7];
        vy.data()[2] = response_array.data[idx+8];

        vz.data()[0] = response_array.data[idx+9];
        vz.data()[1] = response_array.data[idx+10];
        vz.data()[2] = response_array.data[idx+11];

        eigen_pose = Eigen::Translation3d(p) * Eigen::AngleAxisd(computeRotation(vx, vy, vz));

        tf::poseEigenToMsg(eigen_pose, pose);
        pose_array_.poses.push_back(pose);
    }
    pose_array.push_back(pose_array_);
}


/*
visualization_msgs::MarkerArray
HotsprayUtils::convertToArrowMarkers(const noether_msgs::ToolPaths& toolpaths,
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
