#include <hotspray_motion/hotspray_motion_server.h>
#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include "hotspray_msgs/GenerateScanTrajectory.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "eigen_conversions/eigen_msg.h" 
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <chrono>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/macros.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_common/resource_locator.h>

#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/interface_utils.h>
#include <tesseract_motion_planners/trajopt/serialize.h>
#include <tesseract_motion_planners/trajopt/deserialize.h>

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>

#include <tesseract_motion_planners/descartes/deserialize.h>
#include <tesseract_motion_planners/descartes/serialize.h>
#include <tesseract_process_managers/taskflow_generators/cartesian_taskflow.h>

#include <tesseract_time_parameterization/iterative_spline_parameterization.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>

#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/serialize.h>
#include <tesseract_motion_planners/ompl/deserialize.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>

#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string MONITOR_NAMESPACE = "tesseract_ros_examples";

const std::string PROCESS_PROFILE = "PROCESS";
const std::string FREESPACE_PROFILE = tesseract_planning::DEFAULT_PROFILE_KEY;
const std::string TRANSITION_PROFILE = "TRANSITION";

using namespace tesseract_planning;

HotsprayMotionServer::HotsprayMotionServer(ros::NodeHandle nh) : //, std::string config_path) :
    nh_(nh),
    ph_("~"),
    plan_scan_trajectory_service_(ph_.advertiseService("generate_scan_trajectory", &HotsprayMotionServer::generateScanTrajectory, this)),
    plan_spray_trajectory_service_(ph_.advertiseService("generate_spray_trajectory", &HotsprayMotionServer::generateSprayTrajectory, this)),
    vis_pub_(ph_.advertise<visualization_msgs::MarkerArray>("toolpath_marker", 0 )),
    env_(std::make_shared<tesseract_environment::Environment>())
{
  nh_.getParam("arm_controller/joints", joint_names_);

  home_joint_pos_ = Eigen::VectorXd::Constant(6, 1, 0);
  home_joint_pos_(0) = 2.35619;
  home_joint_pos_(1) = -1.5708;
  home_joint_pos_(2) = 1.5708;
  home_joint_pos_(3) = -1.5708;
  home_joint_pos_(4) = -1.5708;
  home_joint_pos_(5) = 0;

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  env_->init(urdf_xml_string, srdf_xml_string, locator);

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, MONITOR_NAMESPACE);
  monitor_->startPublishingEnvironment(); //tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(monitor_->getSceneGraph()->getRoot());

  env_->setState(joint_names_, home_joint_pos_);
}

void HotsprayMotionServer::toMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract_common::JointTrajectory& traj)
{
  for (const auto& js : traj)
  {
    assert(js.joint_names.size() == static_cast<unsigned>(js.position.size()));

    trajectory_msgs::JointTrajectoryPoint js_msg;
    js_msg.positions.resize(static_cast<size_t>(js.position.size()));
    js_msg.velocities.resize(static_cast<size_t>(js.velocity.size()));
    js_msg.accelerations.resize(static_cast<size_t>(js.acceleration.size()));

    for (int i = 0; i < js.position.size(); ++i)
      js_msg.positions[static_cast<size_t>(i)] = js.position(i);

    for (int i = 0; i < js.velocity.size(); ++i)
      js_msg.velocities[static_cast<size_t>(i)] = js.velocity(i);

    for (int i = 0; i < js.acceleration.size(); ++i)
      js_msg.accelerations[static_cast<size_t>(i)] = js.acceleration(i);

    js_msg.time_from_start = ros::Duration(js.time);
    traj_msg.points.push_back(js_msg);
  }
  traj_msg.joint_names = traj[0].joint_names;
  traj_msg.header.frame_id = "0";
  traj_msg.header.stamp = ros::Time(0);
}

tesseract_common::VectorIsometry3d HotsprayMotionServer::sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                  double z_freedom,
                                                  double rx_freedom,
                                                  double ry_freedom,
                                                  double rz_freedom,
                                                  double z_resolution,
                                                  double rx_resolution,
                                                  double ry_resolution,
                                                  double rz_resolution)
{
  tesseract_common::VectorIsometry3d samples;

  int rx_size = static_cast<int>(std::ceil(2.0 * rx_freedom / rx_resolution)) + 1;
  int ry_size = static_cast<int>(std::ceil(2.0 * ry_freedom / ry_resolution)) + 1;
  int rz_size = static_cast<int>(std::ceil(2.0 * rz_freedom / rz_resolution)) + 1;
  int z_size = static_cast<int>(std::ceil(2.0 * z_freedom / z_resolution)) + 1;

  Eigen::VectorXd x_angles = Eigen::VectorXd::LinSpaced(rx_size, -rx_freedom, rx_freedom);
  Eigen::VectorXd y_angles = Eigen::VectorXd::LinSpaced(ry_size, -ry_freedom, ry_freedom);
  Eigen::VectorXd z_angles = Eigen::VectorXd::LinSpaced(rz_size, -rz_freedom, rz_freedom);
  Eigen::VectorXd z_distances = Eigen::VectorXd::LinSpaced(z_size, -z_freedom, z_freedom);

  for (long i = 0; i < static_cast<long>(z_angles.size()); ++i)
  {
    Eigen::Isometry3d rz_sample;
    rz_sample = tool_pose * Eigen::AngleAxisd(z_angles(i), Eigen::Vector3d(0, 0, 1));
    for (long j = 0; j < static_cast<long>(x_angles.size()); ++j)
    {
      Eigen::Isometry3d rx_rz_sample;
      rx_rz_sample = rz_sample * Eigen::AngleAxisd(x_angles(j), Eigen::Vector3d(1, 0, 0));
      for (long k = 0; k < static_cast<long>(y_angles.size()); ++k)
      {
        Eigen::Isometry3d ry_rx_rz_sample;
        ry_rx_rz_sample = rx_rz_sample * Eigen::AngleAxisd(y_angles(k), Eigen::Vector3d(0, 1, 0));

        for (long l = 0; l < static_cast<long>(z_distances.size()); ++l)
        {
          Eigen::Isometry3d z_sample = ry_rx_rz_sample;
          // z_sample.translate(Eigen::Vector3d(0, 0, z_distances(l)));
          z_sample.translation() = ry_rx_rz_sample * Eigen::Vector3d(0, 0, z_distances(l));

          samples.push_back(z_sample);
          // eigen_samples.push_back(z_sample);
        }
      }
    }
  }
  return samples;
}

CompositeInstruction HotsprayMotionServer::createScanProgram( 
                                          const geometry_msgs::PoseArray& pose_array,
                                          const ManipulatorInfo& manipulator
                                          )

{                                      
  CompositeInstruction program("scan_program", CompositeInstructionOrder::ORDERED, manipulator);

  // Start Joint Position for the program
  Waypoint home_wp = JointWaypoint(joint_names_, home_joint_pos_);
  PlanInstruction start_instruction(home_wp, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  Eigen::Isometry3d eigen_pose1;
  tf::poseMsgToEigen(pose_array.poses[0], eigen_pose1);
  Waypoint home_wp1 = CartesianWaypoint(eigen_pose1);

  Eigen::Isometry3d eigen_pose2;
  tf::poseMsgToEigen(pose_array.poses[1], eigen_pose2);
  Waypoint first_raster_wp = CartesianWaypoint(eigen_pose2);

  // Define from start composite instruction
  PlanInstruction transition_from_start0(home_wp1, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  transition_from_start0.setDescription("from_start_plan");
  PlanInstruction transition_from_start1(first_raster_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  transition_from_start1.setDescription("from_start_plan");

  CompositeInstruction from_start(FREESPACE_PROFILE);
  from_start.setDescription("from_start");
  from_start.push_back(transition_from_start0);
  from_start.push_back(transition_from_start1);

  program.push_back(from_start);

  // Create raster segement
  CompositeInstruction raster_segment(PROCESS_PROFILE);
  raster_segment.setDescription("Scan Path");
  for(unsigned long int i = 2; i < pose_array.poses.size(); i++)
  {
    Eigen::Isometry3d eigen_pose;
    tf::poseMsgToEigen(pose_array.poses[i], eigen_pose);
    
    Waypoint raster_waypoint = CartesianWaypoint(eigen_pose);
    raster_segment.push_back(PlanInstruction(raster_waypoint, PlanInstructionType::LINEAR, PROCESS_PROFILE));
  }
  program.push_back(raster_segment);

  PlanInstruction transition_to_end_plan(home_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  transition_to_end_plan.setDescription("transition_to_end_plan");

  CompositeInstruction transition_to_end(FREESPACE_PROFILE);
  transition_to_end.setDescription("transition_from_end");
  transition_to_end.push_back(transition_to_end_plan);
  program.push_back(transition_to_end);
  return program;
  }

bool HotsprayMotionServer::generateScanTrajectory(hotspray_msgs::GenerateScanTrajectory::Request &req,
hotspray_msgs::GenerateScanTrajectory::Response &res)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  // Create manipulator information for program
  mi_.manipulator = "manipulator";
  mi_.tcp_frame = "camera_orbit_frame"; //ToolCenterPoint("tcp_frame", false);
  mi_.working_frame = "world";
  mi_.manipulator_ik_solver = "URInvKin";

  std::string trajopt_process_default_composite_profile_path;
  std::string trajopt_freespace_default_composite_profile_path;
  std::string ompl_plan_profile_path;

  ph_.getParam("scan_trajopt_process_default_composite_profile_path", trajopt_process_default_composite_profile_path);
  ph_.getParam("scan_trajopt_freespace_default_composite_profile_path", trajopt_freespace_default_composite_profile_path);
  ph_.getParam("scan_ompl_plan_profile_path", ompl_plan_profile_path);

  auto ompl_profile = std::make_shared<OMPLDefaultPlanProfile>(omplPlanFromXMLFile(ompl_plan_profile_path));
  auto trajopt_freespace_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  trajopt_freespace_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 0);
  trajopt_freespace_plan_profile->joint_coeff = Eigen::VectorXd::Constant(6, 1, 0);

  auto trajopt_freespace_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_freespace_default_composite_profile_path));
  auto trajopt_process_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  auto trajopt_process_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_process_default_composite_profile_path));

  auto iterative_spline_parameterization_profile_process = std::make_shared<IterativeSplineParameterizationProfile>();
  iterative_spline_parameterization_profile_process->max_velocity_scaling_factor = 0.1;
  iterative_spline_parameterization_profile_process->max_acceleration_scaling_factor = 0.1;

  auto iterative_spline_parameterization_profile_freespace = std::make_shared<IterativeSplineParameterizationProfile>();
  iterative_spline_parameterization_profile_freespace->max_velocity_scaling_factor = 1.0;
  iterative_spline_parameterization_profile_freespace->max_acceleration_scaling_factor = 1.0;

  // Create Program
  CompositeInstruction program = createScanProgram(req.pose_array, mi_);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<tesseract_planning_server::ROSProcessEnvironmentCache>(monitor_), 8);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_PLANNER_NAME;

  // request.instructions = Instruction(program);
  request.instructions = Instruction(program);

  // Add profiles to planning server
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();

  profiles->addProfile<TrajOptPlanProfile>(FREESPACE_PROFILE, trajopt_freespace_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(FREESPACE_PROFILE, trajopt_freespace_composite_profile);
  profiles->addProfile<OMPLPlanProfile>(FREESPACE_PROFILE, ompl_profile);
  profiles->addProfile<IterativeSplineParameterizationProfile>(FREESPACE_PROFILE, iterative_spline_parameterization_profile_freespace);

  profiles->addProfile<TrajOptPlanProfile>(PROCESS_PROFILE, trajopt_process_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(PROCESS_PROFILE, trajopt_process_composite_profile);
  profiles->addProfile<IterativeSplineParameterizationProfile>(PROCESS_PROFILE, iterative_spline_parameterization_profile_process);

  // Print Diagnostics
  request.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();


  // Plot Process Trajectory
  const auto& ci = response.results->as<CompositeInstruction>();
  tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
  tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
  auto state_solver = env_->getStateSolver();
  plotter_->plotMarker(tesseract_visualization::ToolpathMarker(toolpath));

  trajectory_msgs::JointTrajectory traj_msg;
  toMsg(traj_msg, trajectory);
  res.traj = traj_msg;

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

  ROS_INFO("Computation of scan trajectory took %d seconds!", duration);
  ROS_INFO("Final trajectory is collision free");


  plotter_->waitForInput();
  plotter_->plotTrajectory(trajectory, *state_solver);
  return true;
}

CompositeInstruction HotsprayMotionServer::createSprayProgram( 
                                          const std::vector<geometry_msgs::PoseArray, 
                                          std::allocator<geometry_msgs::PoseArray>>& raster_array,
                                          const ManipulatorInfo& manipulator
                                          )
{
  CompositeInstruction program("spray_programm", CompositeInstructionOrder::ORDERED, manipulator);

  // Start Joint Position for the program
  Waypoint home_wp = JointWaypoint(joint_names_, home_joint_pos_);
  PlanInstruction start_instruction(home_wp, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  Eigen::Isometry3d eigen_start_pose;
  geometry_msgs::Pose start_pose;
  start_pose.position.x = -0.199146;
  start_pose.position.y = 0.676904;
  start_pose.position.z = 0.862851;
  start_pose.orientation.x = 9.95752e-07;
  start_pose.orientation.y = 0.70711;
  start_pose.orientation.z = 0.707103;
  start_pose.orientation.w = 3.81413e-06;
  tf::poseMsgToEigen(start_pose, eigen_start_pose);
  Waypoint start_wp = CartesianWaypoint(eigen_start_pose);

  PlanInstruction plan_transition_from_start0(start_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  plan_transition_from_start0.setDescription("transition_from_start");

  Eigen::Isometry3d first_spray_pose;
  tf::poseMsgToEigen(raster_array[0].poses[0], first_spray_pose);
  Waypoint first_spray_wp = CartesianWaypoint(first_spray_pose);

  PlanInstruction plan_transition_from_start1(first_spray_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  plan_transition_from_start1.setDescription("transition_from_start");
  CompositeInstruction transition_from_start(FREESPACE_PROFILE);
  transition_from_start.setDescription("transition_from_start");
  transition_from_start.push_back(plan_transition_from_start0);
  transition_from_start.push_back(plan_transition_from_start1);

  program.push_back(transition_from_start);

  for(unsigned long int j = 0; j < raster_array.size(); j++)
  {
    geometry_msgs::PoseArray raster = raster_array[j];
    if(j != 0)
    {
      Eigen::Isometry3d eigen_pose;
      tf::poseMsgToEigen(raster.poses[0], eigen_pose);
      Waypoint first_raster_wp = CartesianWaypoint(eigen_pose);

      PlanInstruction plan_transition_between_raster(first_raster_wp, PlanInstructionType::FREESPACE, TRANSITION_PROFILE);
      plan_transition_between_raster.setDescription("transition_between_raster_segment_" + std::to_string(j));
      CompositeInstruction transition_between_raster(TRANSITION_PROFILE);
      transition_between_raster.setDescription("transition_between_raster_segment_" + std::to_string(j));
      transition_between_raster.push_back(plan_transition_between_raster);
      program.push_back(transition_between_raster);
    }

    // Create raster segement
    CompositeInstruction raster_segment(PROCESS_PROFILE);
    raster_segment.setDescription("Raster #" + std::to_string(j + 1));
    for(unsigned long int i = 1; i < raster.poses.size(); i++)
    {
      Eigen::Isometry3d eigen_pose;
      tf::poseMsgToEigen(raster.poses[i], eigen_pose);
      
      Waypoint raster_waypoint = CartesianWaypoint(eigen_pose);
      raster_segment.push_back(PlanInstruction(raster_waypoint, PlanInstructionType::LINEAR, PROCESS_PROFILE));
    }
    program.push_back(raster_segment);
  }

  PlanInstruction plan_f1(home_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  plan_f1.setDescription("transition_from_end_plan");

  CompositeInstruction transition(FREESPACE_PROFILE);
  transition.setDescription("transition_from_end");
  transition.push_back(plan_f1);
  program.push_back(transition);
  return program;
}

bool HotsprayMotionServer::generateSprayTrajectory(hotspray_msgs::GenerateSprayTrajectory::Request &req,
hotspray_msgs::GenerateSprayTrajectory::Response &res)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  // Create manipulator information for program
  mi_.manipulator = "manipulator";
  mi_.tcp_frame = "tcp_frame"; //ToolCenterPoint("tcp_frame", false);
  mi_.working_frame = "world";
  mi_.manipulator_ik_solver = "URInvKin";

  std::string trajopt_process_default_composite_profile_path;
  std::string trajopt_freespace_default_composite_profile_path;
  std::string ompl_plan_transition_profile_path;
  std::string ompl_plan_freespace_profile_path;

  ph_.getParam("spray_trajopt_process_default_composite_profile_path", trajopt_process_default_composite_profile_path);
  ph_.getParam("spray_trajopt_freespace_default_composite_profile_path", trajopt_freespace_default_composite_profile_path);
  ph_.getParam("spray_ompl_transition_plan_profile_path", ompl_plan_transition_profile_path);
  ph_.getParam("spray_ompl_freespace_plan_profile_path", ompl_plan_freespace_profile_path);

  // std::vector<Eigen::Isometry3d> spray_eigen_samples;
  // auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfileF>();
  // descartes_plan_profile->use_redundant_joint_solutions = true;
  // descartes_plan_profile->enable_edge_collision = false;
  // descartes_plan_profile->enable_collision = false;
  // descartes_plan_profile->allow_collision = true;
  // descartes_plan_profile->num_threads = 10;

  // double z_freedom, rx_freedom, ry_freedom, rz_freedom, z_resolution, rz_resolution, rx_resolution, ry_resolution;
  // ph_.getParam("spray_z_freedom", z_freedom);
  // ph_.getParam("spray_rx_freedom", rx_freedom);
  // ph_.getParam("spray_ry_freedom", ry_freedom);
  // ph_.getParam("spray_rz_freedom", rz_freedom);
  // ph_.getParam("spray_z_resolution", z_resolution);
  // ph_.getParam("spray_rx_resolution", rx_resolution);
  // ph_.getParam("spray_ry_resolution", ry_resolution);
  // ph_.getParam("spray_rz_resolution", rz_resolution);

  // descartes_plan_profile->target_pose_sampler = [&](const Eigen::Isometry3d& tool_pose) {
  //   return HotsprayMotionServer::sampleToolAxis(tool_pose,
  //                                               z_freedom,
  //                                               (rx_freedom*M_PI/180),
  //                                               (ry_freedom*M_PI/180),
  //                                               (rz_freedom*M_PI/180),
  //                                               z_resolution,
  //                                               rx_resolution,
  //                                               ry_resolution,
  //                                               rz_resolution);
  // };

  auto ompl_freespace_profile = std::make_shared<OMPLDefaultPlanProfile>(omplPlanFromXMLFile(ompl_plan_freespace_profile_path));
  auto ompl_transiton_profile = std::make_shared<OMPLDefaultPlanProfile>(omplPlanFromXMLFile(ompl_plan_transition_profile_path));

  auto trajopt_freespace_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  trajopt_freespace_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 0);
  trajopt_freespace_plan_profile->joint_coeff = Eigen::VectorXd::Constant(6, 1, 0);
  auto trajopt_freespace_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_freespace_default_composite_profile_path));

  auto trajopt_process_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>(); 
  trajopt_process_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
  trajopt_process_plan_profile->cartesian_coeff(3) = 2;
  trajopt_process_plan_profile->cartesian_coeff(4) = 2;
  trajopt_process_plan_profile->cartesian_coeff(5) = 0;
  auto trajopt_process_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_process_default_composite_profile_path));
  auto trajopt_process_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  trajopt_process_solver_profile->opt_info.cnt_tolerance= 1.0;


  // Create Program
  CompositeInstruction program = createSprayProgram(req.raster_array, mi_);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<tesseract_planning_server::ROSProcessEnvironmentCache>(monitor_), 8);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_FT_PLANNER_NAME;
  request.instructions = Instruction(program);

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();

  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(TRANSITION_PROFILE, trajopt_freespace_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(TRANSITION_PROFILE, trajopt_freespace_composite_profile);
  profiles->addProfile<OMPLPlanProfile>(TRANSITION_PROFILE, ompl_transiton_profile);

  profiles->addProfile<TrajOptPlanProfile>(FREESPACE_PROFILE, trajopt_freespace_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(FREESPACE_PROFILE, trajopt_freespace_composite_profile);
  profiles->addProfile<OMPLPlanProfile>(FREESPACE_PROFILE, ompl_freespace_profile);

  profiles->addProfile<TrajOptPlanProfile>(PROCESS_PROFILE, trajopt_process_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(PROCESS_PROFILE, trajopt_process_composite_profile);
  profiles->addProfile<TrajOptSolverProfile>(PROCESS_PROFILE, trajopt_process_solver_profile);

  // Print Diagnostics
  request.instructions.print("Program: ");

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  const auto& ci = response.results->as<CompositeInstruction>();
  tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
  tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
  auto state_solver = env_->getStateSolver();

  auto end_time = std::chrono::high_resolution_clock::now();

  trajectory_msgs::JointTrajectory traj_msg;
  toMsg(traj_msg, trajectory);
  res.traj = traj_msg;

  auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

  ROS_INFO("Final trajectory is collision free");
  ROS_INFO("Computation of spray trajectory took %d seconds!", duration);

  plotter_->waitForInput();
  plotter_->plotTrajectory(trajectory, *state_solver);
  return true;
}