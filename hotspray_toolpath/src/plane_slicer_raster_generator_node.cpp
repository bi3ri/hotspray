#include <ros/ros.h>
#include <XmlRpcException.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <noether_conversions/noether_conversions.h>
#include <tool_path_planner/plane_slicer_raster_generator.h>
#include <tool_path_planner/utilities.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <console_bridge/console.h>
#include <noether_msgs/GenerateToolPathsAction.h>

#include "hotspray_msgs/GenerateToolpath.h"

using RGBA = std::tuple<double, double, double, double>;

static const double GOAL_WAIT_PERIOD = 300.0;  // sec
static const std::string DEFAULT_FRAME_ID = "tsdf_origin";
static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_tool_paths";
static const std::string RASTER_LINES_MARKERS_TOPIC = "raster_lines";
static const std::string RASTER_POSES_MARKERS_TOPIC = "raster_poses";
static const std::string RASTER_PATH_NS = "raster_";
static const std::string INPUT_MESH_NS = "input_mesh";
static const RGBA RAW_MESH_RGBA = std::make_tuple(0.6, 0.6, 1.0, 1.0);

std::size_t countPathPoints(const noether_msgs::ToolPaths& tool_paths)
{
  std::size_t point_count = 0;

  for (const auto& path : tool_paths.paths)
    for (const auto& segment : path.segments)
      point_count += segment.poses.size();

  return point_count;
}

class PlaneSlicerExample
{
public:
  PlaneSlicerExample(ros::NodeHandle nh) : nh_(nh), ac_(GENERATE_TOOL_PATHS_ACTION)
  {
    raster_lines_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(RASTER_LINES_MARKERS_TOPIC, 1);
    raster_poses_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(RASTER_POSES_MARKERS_TOPIC, 1);
    //http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
    service_ = nh_.advertiseService("generate_toolpath", &PlaneSlicerExample::generateToolpath, this);
  }

  ~PlaneSlicerExample() {}

  bool loadConfig(tool_path_planner::PlaneSlicerRasterGenerator::Config& config)
  {
    ros::NodeHandle ph("~");
    XmlRpc::XmlRpcValue cfg;
    if (!ph.getParam("plane_slicer_rastering", cfg))
    {
      return false;
    }

    std::vector<std::string> field_names = { "raster_spacing", "point_spacing",    "raster_rot_offset",
                                             "min_hole_size",  "min_segment_size", "search_radius" };
    if (!std::all_of(field_names.begin(), field_names.end(), [&cfg](const std::string& f) { return cfg.hasMember(f); }))
    {
      ROS_ERROR("Failed to find one or more members");
    }

    std::size_t idx = 0;
    try
    {
      config.raster_spacing = static_cast<double>(cfg[field_names[idx++]]);
      config.point_spacing = static_cast<double>(cfg[field_names[idx++]]);
      config.raster_rot_offset = static_cast<double>(cfg[field_names[idx++]]);
      config.min_hole_size = static_cast<double>(cfg[field_names[idx++]]);
      config.min_segment_size = static_cast<double>(cfg[field_names[idx++]]);
      config.search_radius = static_cast<double>(cfg[field_names[idx++]]);
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR_STREAM(e.getMessage());
    }
    return true;
  }

  bool generateToolpath(hotspray_msgs::GenerateToolpath::Request &req, hotspray_msgs::GenerateToolpath::Response &res)
  {
    using namespace noether_conversions;

    // loading parameters
    ros::NodeHandle ph("~");
    std::string mesh_file = req.mesh_path;

    ROS_INFO("Got mesh file %s", mesh_file.c_str());

    // get configuration
    tool_path_planner::PlaneSlicerRasterGenerator::Config config;
    if (!loadConfig(config))
    {
      ROS_WARN("Failed to load configuration, using default parameters");
    }

    markers_publish_timer_ = nh_.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent& e) {
      if (!line_markers_.markers.empty())
      {
        raster_lines_markers_pub_.publish(line_markers_);
      }

      if (!poses_markers_.markers.empty())
      {
        raster_poses_markers_pub_.publish(poses_markers_);
      }
    });

    shape_msgs::Mesh mesh_msg;
    if (!noether_conversions::loadPLYFile(mesh_file, mesh_msg))
    {
      ROS_ERROR("Failed to read file %s", mesh_file.c_str());
      res.success = false;
      return false;
    }
    line_markers_.markers.push_back(createMeshMarker(mesh_file, INPUT_MESH_NS, DEFAULT_FRAME_ID, RAW_MESH_RGBA));

    // waiting for server
    if (!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("Action server %s was not found", GENERATE_TOOL_PATHS_ACTION.c_str());
      res.success = true;
      return false;
    }

    // sending request
    noether_msgs::GenerateToolPathsGoal goal;
    noether_msgs::ToolPathConfig config_msg;
    config_msg.type = noether_msgs::ToolPathConfig::PLANE_SLICER_RASTER_GENERATOR;
    tool_path_planner::toPlaneSlicerConfigMsg(config_msg.plane_slicer_generator, config);

    goal.path_configs.push_back(config_msg);
    goal.surface_meshes.push_back(mesh_msg);
    goal.proceed_on_failure = true;
    ac_.sendGoal(goal);
    ros::Time start_time = ros::Time::now();
    if (!ac_.waitForResult(ros::Duration(GOAL_WAIT_PERIOD)))
    {
      ROS_ERROR("Failed to generate edges from mesh");
      res.success = false;
      return false;
    }

    noether_msgs::GenerateToolPathsResultConstPtr result = ac_.getResult();
    if (!result->success)
    {
      ROS_ERROR("Failed to generate edges from mesh");
      res.success = false;
      return false;
    }

    const std::vector<noether_msgs::ToolPaths>& raster_paths = result->tool_paths;
    res.tool_paths = result->tool_paths;
    res.tool_path_validities = result->tool_path_validities;
    ROS_INFO("Found %lu rasters", raster_paths.size());

    for (std::size_t i = 0; i < raster_paths.size(); i++)
    {

      ROS_INFO("Raster %lu contains %lu points", i+1, countPathPoints(raster_paths[i]));

      std::string ns = RASTER_PATH_NS + std::to_string(i);
      visualization_msgs::MarkerArray edge_path_axis_markers =
          convertToAxisMarkers(raster_paths[i], DEFAULT_FRAME_ID, ns);

      visualization_msgs::MarkerArray edge_path_line_markers =
          convertToArrowMarkers(raster_paths[i], DEFAULT_FRAME_ID, ns);

      poses_markers_.markers.insert(
          poses_markers_.markers.end(), edge_path_axis_markers.markers.begin(), edge_path_axis_markers.markers.end());
      line_markers_.markers.insert(
          line_markers_.markers.end(), edge_path_line_markers.markers.begin(), edge_path_line_markers.markers.end());
    }
    res.success = true;
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Timer markers_publish_timer_;
  ros::Publisher raster_lines_markers_pub_;
  ros::Publisher raster_poses_markers_pub_;
  visualization_msgs::MarkerArray line_markers_;
  visualization_msgs::MarkerArray poses_markers_;
  actionlib::SimpleActionClient<noether_msgs::GenerateToolPathsAction> ac_;
  ros::ServiceServer service_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_slicer_example");
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  ros::NodeHandle nh("~"); // ~ changes namespace relative to name of node for published services/topics
  ros::AsyncSpinner spinner(2);
  spinner.start();
  PlaneSlicerExample plane_slicer_example(nh);
  ros::waitForShutdown();
  return 0;
}
