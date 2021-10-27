#include <hotspray_motion/hotspray_motion_server.h>

#include <ros/ros.h>

#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include "hotspray_msgs/GenerateScanTrajectory.h"

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <visualization_msgs/MarkerArray.h>
#include "eigen_conversions/eigen_msg.h" //conversion posemsg -> eigen
#include <Eigen/Geometry>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_common/macros.h>

#include <tesseract_environment/core/utils.h>
#include <tesseract_environment/core/commands.h>
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
#include <tesseract_scene_graph/resource_locator.h>

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
#include <tesseract_process_managers/taskflow_generators/descartes_taskflow.h>

#include <tesseract_process_managers/taskflow_generators/cartesian_taskflow.h>


#include <tesseract_time_parameterization/iterative_spline_parameterization.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include <tesseract_process_managers/task_generators/time_optimal_trajectory_generation_task_generator.h>

#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/serialize.h>
#include <tesseract_motion_planners/ompl/deserialize.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>

#include <tinyxml2.h>

#include <tesseract_kinematics/ur/ur_inv_kin.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>


#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

const std::string PROCESS_PROFILE = "PROCESS";
const std::string FREESPACE_PROFILE = tesseract_planning::DEFAULT_PROFILE_KEY;
// const std::string FREESPACE_PROFILE = "FREESPACE";

using namespace tesseract_planning;


HotsprayMotionServer::HotsprayMotionServer(ros::NodeHandle nh) : //, std::string config_path) :
    nh_(nh),
    ph_("~"),
    plan_scan_trajectory_service_(ph_.advertiseService("generate_scan_trajectory", &HotsprayMotionServer::generateScanTrajectory, this)),
    plan_spray_trajectory_service_(ph_.advertiseService("generate_spray_trajectory", &HotsprayMotionServer::generateSprayTrajectory, this)),
    vis_pub_(ph_.advertise<visualization_msgs::MarkerArray>("toolpath_marker", 0 )),
    env_(std::make_shared<tesseract_environment::Environment>()),
    rviz_(true),
    plotting_(true)

{
    ph_.getParam("debug", debug_);
    nh_.getParam("arm_controller/joints", joint_names_);

    home_joint_pos_ = Eigen::VectorXd::Constant(6, 1, 0);
    home_joint_pos_(0) = 2.35619;
    home_joint_pos_(1) = -1.5708;
    home_joint_pos_(2) = 1.5708;
    home_joint_pos_(3) = -1.5708;
    home_joint_pos_(4) = -1.5708;
    home_joint_pos_(5) = 0;

    // home_joint_pos_ = Eigen::VectorXd::Constant(7, 1, 0);
    // home_joint_pos_(0) = 1.8850;
    // home_joint_pos_(1) = 0.6364;
    // home_joint_pos_(2) = -0.0492;
    // home_joint_pos_(3) = -0.9372;
    // home_joint_pos_(4) = 0.4753;
    // home_joint_pos_(5) = 0.0347;
    // home_joint_pos_(6) = 0.1856;
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
  traj_msg.header.stamp = ros::Time(0); //ros::Time::now();

}

tesseract_common::VectorIsometry3d HotsprayMotionServer::sampleToolAxis(const Eigen::Isometry3d& tool_pose,
                                                  double z_freedom,
                                                  double rx_freedom,
                                                  double ry_freedom,
                                                  double rz_freedom,
                                                  double z_resolution,
                                                  double rx_resolution,
                                                  double ry_resolution,
                                                  double rz_resolution,
                                                  std::vector<Eigen::Isometry3d>& eigen_samples
                                                  )
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
  std::cout << "samlpe size: " << samples.size() << std::endl;
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
  PlanInstruction transition_from_start(home_wp1, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  transition_from_start.setDescription("from_start_plan");
  PlanInstruction transition_from_start2(first_raster_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  transition_from_start.setDescription("from_start_plan");
  CompositeInstruction from_start(FREESPACE_PROFILE);
  from_start.setDescription("from_start");
  from_start.push_back(transition_from_start);
  from_start.push_back(transition_from_start2);

  program.push_back(from_start);
  // program.push_back(transition_from_start);


  // Create raster segement
  CompositeInstruction raster_segment(PROCESS_PROFILE);
  raster_segment.setDescription("Raster #");
  for(unsigned long int i = 2; i < pose_array.poses.size(); i++)
  {
    Eigen::Isometry3d eigen_pose;
    tf::poseMsgToEigen(pose_array.poses[i], eigen_pose);
    
    Waypoint raster_waypoint = CartesianWaypoint(eigen_pose);
    raster_segment.push_back(PlanInstruction(raster_waypoint, PlanInstructionType::LINEAR, PROCESS_PROFILE));
  }
  program.push_back(raster_segment);

  PlanInstruction plan_f1(home_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  plan_f1.setDescription("transition_from_end_plan");

  CompositeInstruction transition(FREESPACE_PROFILE);
  transition.setDescription("transition_from_end");
  transition.push_back(plan_f1);
  program.push_back(transition);
  // program.push_back(plan_f1);

  return program;
  }


bool HotsprayMotionServer::generateScanTrajectory(hotspray_msgs::GenerateScanTrajectory::Request &req,
hotspray_msgs::GenerateScanTrajectory::Response &res)
{
  // using CartesianWaypoint;
  // using CompositeInstruction;
  // using CompositeInstructionOrder;
  // using Instruction;
  // using ManipulatorInfo;
  // using PlanInstruction;
  // using PlanInstructionType;
  // using ProcessPlanningFuture;
  // using ProcessPlanningRequest;
  // using ProcessPlanningServer;
  // using StateWaypoint;
  // using ToolCenterPoint;
  // using Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<tesseract_environment::OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(); //tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  env_->setState(joint_names_, home_joint_pos_);

  // Create manipulator information for program
  ManipulatorInfo mi;
  mi.manipulator = "manipulator";
  mi.tcp = ToolCenterPoint("camera_orbit_frame", false);
  mi.working_frame = "world";
  mi.manipulator_ik_solver = "URInvKin";

  auto fwd_env_solver = env_->getManipulatorManager()->getFwdKinematicSolver("manipulator");

  auto inv_kin = std::make_shared<tesseract_kinematics::URInvKin>();

  inv_kin->init("manipulator",
                              tesseract_kinematics::UR5Parameters,
                              fwd_env_solver->getBaseLinkName(),
                              fwd_env_solver->getTipLinkName(),
                              fwd_env_solver->getJointNames(),
                              fwd_env_solver->getLinkNames(),
                              fwd_env_solver->getActiveLinkNames(),
                              fwd_env_solver->getLimits());

  env_->getManipulatorManager()->addInvKinematicSolver(inv_kin);
  env_->getManipulatorManager()->setDefaultInvKinematicSolver("manipulator", "URInvKin");

  std::string trajopt_process_default_composite_profile_path;
  std::string trajopt_process_default_plan_profile_path;
  std::string trajopt_freespace_default_composite_profile_path;
  std::string trajopt_freespace_default_plan_profile_path;
  std::string descartes_plan_profile_path;
  std::string ompl_plan_profile_path;

  ph_.getParam("scan_trajopt_process_default_composite_profile_path", trajopt_process_default_composite_profile_path);
  ph_.getParam("scan_trajopt_process_default_plan_profile_path", trajopt_process_default_plan_profile_path);
  ph_.getParam("scan_trajopt_freespace_default_composite_profile_path", trajopt_freespace_default_composite_profile_path);
  ph_.getParam("scan_trajopt_freespace_default_plan_profile_path", trajopt_freespace_default_plan_profile_path);
  ph_.getParam("scan_descartes_plan_profile_path", descartes_plan_profile_path);
  ph_.getParam("scan_ompl_plan_profile_path", ompl_plan_profile_path);

  std::vector<Eigen::Isometry3d> spray_eigen_samples;
  auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfileF>();//descartesPlanFromXMLFile(descartes_plan_profile_path));
  descartes_plan_profile->use_redundant_joint_solutions = true;
  descartes_plan_profile->enable_edge_collision = true;
  descartes_plan_profile->enable_collision = true;
  descartes_plan_profile->allow_collision = false;
  descartes_plan_profile->num_threads = 12;

  double z_freedom, rx_freedom, ry_freedom, rz_freedom, z_resolution, rz_resolution, rx_resolution, ry_resolution;
  ph_.getParam("scan_z_freedom", z_freedom);
  ph_.getParam("scan_rx_freedom", rx_freedom);
  ph_.getParam("scan_ry_freedom", ry_freedom);
  ph_.getParam("scan_rz_freedom", rz_freedom);
  ph_.getParam("scan_z_resolution", z_resolution);
  ph_.getParam("scan_rx_resolution", rx_resolution);
  ph_.getParam("scan_ry_resolution", ry_resolution);
  ph_.getParam("scan_rz_resolution", rz_resolution);

  descartes_plan_profile->target_pose_sampler = [&](const Eigen::Isometry3d& tool_pose) {
    return HotsprayMotionServer::sampleToolAxis(tool_pose,
                                                z_freedom,
                                                (rx_freedom*M_PI/180),
                                                (ry_freedom*M_PI/180),
                                                (rz_freedom*M_PI/180),
                                                z_resolution,
                                                rx_resolution,
                                                ry_resolution,
                                                rz_resolution,
                                                spray_eigen_samples);
  };
  // double ompl_range;
  // double planning_time;
  // ph_.getParam("ompl_range", ompl_range);
  // ph_.getParam("planning_time", planning_time);
  // Create OMPL Profile
  auto ompl_profile = std::make_shared<OMPLDefaultPlanProfile>(omplPlanFromXMLFile(ompl_plan_profile_path));
  // auto ompl_planner_config = std::make_shared<RRTConnectConfigurator>();
  // auto ompl_planner_config = std::make_shared<RRTstarConfigurator>();
  auto ompl_planner_config = std::make_shared<tesseract_planning::TRRTConfigurator>();
  // auto ompl_planner_config = std::make_shared<tesseract_planning::PRMConfigurator>();
  // auto ompl_planner_config = std::make_shared<tesseract_planning::ESTConfigurator>();



  // ompl_planner_config->range = 0.0;
  ompl_profile->planning_time = 20.0;

  ompl_profile->planners = { ompl_planner_config };


  // toXMLFile(*ompl_profile, ompl_plan_profile_path);

  auto trajopt_freespace_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>(trajOptPlanFromXMLFile(trajopt_freespace_default_plan_profile_path));
  auto trajopt_freespace_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_freespace_default_composite_profile_path));
  auto trajopt_freespace_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();

  auto trajopt_process_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>(trajOptPlanFromXMLFile(trajopt_process_default_plan_profile_path));
  auto trajopt_process_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_process_default_composite_profile_path));
  auto trajopt_process_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  trajopt_process_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_process_solver_profile->opt_info.max_iter = 200;
  trajopt_process_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_process_solver_profile->opt_info.min_trust_box_size = 1e-3;
  trajopt_process_solver_profile->opt_info.cnt_tolerance= 10.0;

  trajopt_freespace_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_freespace_solver_profile->opt_info.max_iter = 200;
  trajopt_freespace_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_freespace_solver_profile->opt_info.min_trust_box_size = 1e-3;
  // trajopt_freespace_solver_profile->opt_info.cnt_tolerance= 9999.0;

  // ompl_plan_profile->optimize = true;
  // toXMLFile(ompl_plan_profile, "/home/bi3ri/hotspray_ws/ompl.xml");

  auto iterative_spline_parameterization_profile = std::make_shared<IterativeSplineParameterizationProfile>();

  iterative_spline_parameterization_profile->max_velocity_scaling_factor = 0.5;
  iterative_spline_parameterization_profile->max_acceleration_scaling_factor = 0.5;

  // Create Program
  CompositeInstruction program = createScanProgram(req.pose_array, mi);

  // Create a seed
  auto cur_state = env_->getCurrentState();
  // CompositeInstruction seed = generateSeed(program, cur_state, env_);

  auto sample_markers = HotsprayUtils::convertToAxisMarkers(spray_eigen_samples, "world", "spray_poses");
  vis_pub_.publish(sample_markers);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 12);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = process_planner_names::RASTER_CT_DT_PLANNER_NAME;
  // request.name = process_planner_names::CARTESIAN_PLANNER_NAME;

  // request.name = process_planner_names::

  // request.instructions = Instruction(program);
  request.instructions = Instruction(program);
  // request.profile = true;

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  profiles->addProfile<SimplePlannerPlanProfile>(FREESPACE_PROFILE, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>(PROCESS_PROFILE, default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>("scan_programm", default_simple_plan_profile);
  profiles->addProfile<SimplePlannerPlanProfile>("DEFAULT", default_simple_plan_profile);
  
  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(FREESPACE_PROFILE, trajopt_freespace_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(FREESPACE_PROFILE, trajopt_freespace_composite_profile);
  // profiles->addProfile<TrajOptSolverProfile>(FREESPACE_PROFILE, trajopt_freespace_solver_profile);
  profiles->addProfile<OMPLPlanProfile>(FREESPACE_PROFILE, ompl_profile);
  profiles->addProfile<IterativeSplineParameterizationProfile>("FREESPACE", iterative_spline_parameterization_profile);

  profiles->addProfile<TrajOptPlanProfile>(PROCESS_PROFILE, trajopt_process_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(PROCESS_PROFILE, trajopt_process_composite_profile);
  profiles->addProfile<TrajOptSolverProfile>(PROCESS_PROFILE, trajopt_process_solver_profile);
  profiles->addProfile<DescartesPlanProfile<float>>(PROCESS_PROFILE, descartes_plan_profile);
  profiles->addProfile<IterativeSplineParameterizationProfile>(PROCESS_PROFILE, iterative_spline_parameterization_profile);

  // profiles->addProfile<TrajOptPlanProfile>("scan_program", trajopt_plan_profile);
  // profiles->addProfile<TrajOptCompositeProfile>("scan_program", trajopt_composite_profile);
  // profiles->addProfile<TrajOptSolverProfile>("scan_program", trajopt_solver_profile);

  auto test12 = planning_server.getAvailableProcessPlanners();
  auto test11232 = planning_server.getProfiles();

  // Print Diagnostics
  request.instructions.print("Program: ");
  // request.plan_profile_remapping = true;

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  plotter->waitForInput();
  const auto& ci = response.results->as<CompositeInstruction>();
  tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
  plotter->plotTrajectory(trajectory, env_->getStateSolver());
  
  ROS_INFO("Final trajectory is collision free");
  trajectory_msgs::JointTrajectory traj_msg;
  toMsg(traj_msg, trajectory);
  res.traj = traj_msg;

  ROS_INFO("Final trajectory is collision free");
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

  // Create initial segement

  // Define from start composite instruction
  Eigen::VectorXd start_joint_pos1 = Eigen::VectorXd::Constant(6, 1, 0);
  start_joint_pos1(0) = 0.890118;
  start_joint_pos1(1) = -1.95477;
  start_joint_pos1(2) = 2.46091;
  start_joint_pos1(3) = -2.04204;
  start_joint_pos1(4) = -1.6057;
  start_joint_pos1(5) = -0.0872665;

  Eigen::VectorXd start_joint_pos2 = Eigen::VectorXd::Constant(6, 1, 0);
  start_joint_pos2(0) = 0.977384;
  start_joint_pos2(1) = -2.0944;
  start_joint_pos2(2) = 2.3911;
  start_joint_pos2(3) = -1.8675;
  start_joint_pos2(4) = -1.5708;
  start_joint_pos2(5) = -0.610865;

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

  Waypoint first_joint_wp = JointWaypoint(joint_names_, start_joint_pos1);
  Waypoint second_joint_wp = JointWaypoint(joint_names_, start_joint_pos2);
  Waypoint start_wp = CartesianWaypoint(eigen_start_pose);

  PlanInstruction transition_from_start0(start_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  PlanInstruction transition_from_start1(first_joint_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
  PlanInstruction transition_from_start2(second_joint_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);

  transition_from_start1.setDescription("from_start_plan");
  CompositeInstruction from_start(FREESPACE_PROFILE);
  from_start.setDescription("from_start");
  from_start.push_back(transition_from_start0);
  from_start.push_back(transition_from_start1);
  from_start.push_back(transition_from_start2);
  program.push_back(from_start);

  // Create initial raster segement
  CompositeInstruction raster_segment(PROCESS_PROFILE);
  raster_segment.setDescription("Raster #" + std::to_string(1));
  for(unsigned long int i = 0; i < raster_array[0].poses.size(); i++)
  {
    Eigen::Isometry3d eigen_pose;
    tf::poseMsgToEigen(raster_array[0].poses[i], eigen_pose);
    
    Waypoint raster_waypoint = CartesianWaypoint(eigen_pose);
    raster_segment.push_back(PlanInstruction(raster_waypoint, PlanInstructionType::LINEAR, PROCESS_PROFILE));
    std::cout << i << std::endl;
  }
  program.push_back(raster_segment);


  // Create noninitial segements
  for(unsigned long int j = 1; j < raster_array.size(); j++)
  {
    auto& raster = raster_array[j];
    Eigen::Isometry3d eigen_pose;
    tf::poseMsgToEigen(raster.poses[0], eigen_pose);
    Waypoint first_raster_wp = CartesianWaypoint(eigen_pose);

    PlanInstruction transition_between_raster(first_raster_wp, PlanInstructionType::FREESPACE, FREESPACE_PROFILE);
    transition_between_raster.setDescription("transition_between_raster_segement");
    CompositeInstruction transition(PROCESS_PROFILE);
    transition.setDescription("transition_between_raster_segement");
    transition.push_back(transition_between_raster);
    program.push_back(transition);

    // Create raster segement
    CompositeInstruction raster_segment(PROCESS_PROFILE);
    raster_segment.setDescription("Raster #" + std::to_string(j + 1));
    for(unsigned long int i = 1; i < raster.poses.size(); i++)
    {
      Eigen::Isometry3d eigen_pose;
      tf::poseMsgToEigen(raster.poses[i], eigen_pose);
      
      Waypoint raster_waypoint = CartesianWaypoint(eigen_pose);
      raster_segment.push_back(PlanInstruction(raster_waypoint, PlanInstructionType::LINEAR, PROCESS_PROFILE));
      std::cout << i << std::endl;
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
  // using CartesianWaypoint;
  // using CompositeInstruction;
  // using CompositeInstructionOrder;
  // using Instruction;
  // using ManipulatorInfo;
  // using PlanInstruction;
  // using PlanInstructionType;
  // using ProcessPlanningFuture;
  // using ProcessPlanningRequest;
  // using ProcessPlanningServer;
  // using StateWaypoint;
  // using ToolCenterPoint;
  // using Waypoint;
  using tesseract_planning_server::ROSProcessEnvironmentCache;

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<tesseract_environment::OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment(); //tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(monitor_->getSceneGraph()->getRoot());
  if (rviz_)
    plotter->waitForConnection();

  env_->setState(joint_names_, home_joint_pos_);

  // Create manipulator information for program
  ManipulatorInfo mi;
  mi.manipulator = "manipulator";
  mi.tcp = ToolCenterPoint("tcp_frame", false);
  mi.working_frame = "world";
  mi.manipulator_ik_solver = "URInvKin";

  auto fwd_env_solver = env_->getManipulatorManager()->getFwdKinematicSolver("manipulator");

  auto inv_kin = std::make_shared<tesseract_kinematics::URInvKin>();

  inv_kin->init("manipulator",
                              tesseract_kinematics::UR5Parameters,
                              fwd_env_solver->getBaseLinkName(),
                              fwd_env_solver->getTipLinkName(),
                              fwd_env_solver->getJointNames(),
                              fwd_env_solver->getLinkNames(),
                              fwd_env_solver->getActiveLinkNames(),
                              fwd_env_solver->getLimits());

  env_->getManipulatorManager()->addInvKinematicSolver(inv_kin);
  env_->getManipulatorManager()->setDefaultInvKinematicSolver("manipulator", "URInvKin");

  std::string trajopt_process_default_composite_profile_path;
  std::string trajopt_process_default_plan_profile_path;
  std::string trajopt_freespace_default_composite_profile_path;
  std::string trajopt_freespace_default_plan_profile_path;
  std::string descartes_plan_profile_path;
  std::string ompl_plan_profile_path;

  ph_.getParam("spray_trajopt_process_default_composite_profile_path", trajopt_process_default_composite_profile_path);
  ph_.getParam("spray_trajopt_process_default_plan_profile_path", trajopt_process_default_plan_profile_path);
  ph_.getParam("spray_trajopt_freespace_default_composite_profile_path", trajopt_freespace_default_composite_profile_path);
  ph_.getParam("spray_trajopt_freespace_default_plan_profile_path", trajopt_freespace_default_plan_profile_path);
  ph_.getParam("spray_descartes_plan_profile_path", descartes_plan_profile_path);
  ph_.getParam("spray_ompl_plan_profile_path", ompl_plan_profile_path);

  std::vector<Eigen::Isometry3d> spray_eigen_samples;
  auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfileF>();//descartesPlanFromXMLFile(descartes_plan_profile_path));
  descartes_plan_profile->use_redundant_joint_solutions = true;
  descartes_plan_profile->enable_edge_collision = false;
  descartes_plan_profile->enable_collision = false;
  descartes_plan_profile->allow_collision = true;
  descartes_plan_profile->num_threads = 10;

  double z_freedom, rx_freedom, ry_freedom, rz_freedom, z_resolution, rz_resolution, rx_resolution, ry_resolution;
  ph_.getParam("spray_z_freedom", z_freedom);
  ph_.getParam("spray_rx_freedom", rx_freedom);
  ph_.getParam("spray_ry_freedom", ry_freedom);
  ph_.getParam("spray_rz_freedom", rz_freedom);
  ph_.getParam("spray_z_resolution", z_resolution);
  ph_.getParam("spray_rx_resolution", rx_resolution);
  ph_.getParam("spray_ry_resolution", ry_resolution);
  ph_.getParam("spray_rz_resolution", rz_resolution);

  descartes_plan_profile->target_pose_sampler = [&](const Eigen::Isometry3d& tool_pose) {
    return HotsprayMotionServer::sampleToolAxis(tool_pose,
                                                z_freedom,
                                                (rx_freedom*M_PI/180),
                                                (ry_freedom*M_PI/180),
                                                (rz_freedom*M_PI/180),
                                                z_resolution,
                                                rx_resolution,
                                                ry_resolution,
                                                rz_resolution,
                                                spray_eigen_samples);
  };
  // double ompl_range;
  // double planning_time;
  // ph_.getParam("ompl_range", ompl_range);
  // ph_.getParam("planning_time", planning_time);
  // Create OMPL Profile

  // toXMLFile(*ompl_profile, ompl_plan_profile_path);

  auto ompl_profile = std::make_shared<OMPLDefaultPlanProfile>(omplPlanFromXMLFile(ompl_plan_profile_path));
  // auto ompl_planner_config = std::make_shared<RRTConnectConfigurator>();
  // auto ompl_planner_config = std::make_shared<RRTstarConfigurator>();
  auto ompl_planner_config1 = std::make_shared<tesseract_planning::TRRTConfigurator>();
  auto ompl_planner_config2 = std::make_shared<tesseract_planning::PRMstarConfigurator>();
  auto ompl_planner_config3 = std::make_shared<tesseract_planning::BiTRRTConfigurator>();
  auto ompl_planner_config4 = std::make_shared<tesseract_planning::LBKPIECE1Configurator>();
  auto ompl_planner_config5 = std::make_shared<tesseract_planning::SBLConfigurator>();
  auto ompl_planner_config6 = std::make_shared<tesseract_planning::ESTConfigurator>();
  auto ompl_planner_config7 = std::make_shared<tesseract_planning::RRTConnectConfigurator>();


  ompl_planner_config1->range = 0.0;
  // ompl_planner_config2->range = 0.0;
  ompl_planner_config3->range = 0.0;
  ompl_planner_config4->range = 0.0;
  ompl_planner_config5->range = 0.0;
  ompl_planner_config6->range = 0.0;
  ompl_planner_config7->range = 0.0;


  ompl_profile->planning_time = 20.0;
  ompl_profile->max_solutions = 99;
  ompl_profile->optimize = true;
  // ompl_profile->



  // ompl_profile->planners = { ompl_planner_config1, ompl_planner_config2,  ompl_planner_config3, ompl_planner_config4, ompl_planner_config5, ompl_planner_config6, ompl_planner_config7};
  ompl_profile->planners = { ompl_planner_config1, ompl_planner_config1, ompl_planner_config1, ompl_planner_config1, ompl_planner_config1, ompl_planner_config1, ompl_planner_config1};


  auto trajopt_freespace_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>(); //trajOptPlanFromXMLFile(trajopt_freespace_default_plan_profile_path));
  auto trajopt_freespace_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_freespace_default_composite_profile_path));
  auto trajopt_freespace_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();

  auto trajopt_process_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>(trajOptPlanFromXMLFile(trajopt_process_default_plan_profile_path));
  auto trajopt_process_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>(trajOptCompositeFromXMLFile(trajopt_process_default_composite_profile_path));
  auto trajopt_process_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  // trajopt_process_solver_profile->convex_solver = sco::ModelType::OSQP;
  // trajopt_process_solver_profile->opt_info.max_iter = 200;
  // trajopt_process_solver_profile->opt_info.min_approx_improve = 1e-3;
  // trajopt_process_solver_profile->opt_info.min_trust_box_size = 1e-3;
  trajopt_process_solver_profile->opt_info.cnt_tolerance= 10.0;
  // trajopt_process_solver_profile->opt_info.

  trajopt_freespace_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_freespace_solver_profile->opt_info.max_iter = 200;
  trajopt_freespace_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_freespace_solver_profile->opt_info.min_trust_box_size = 1e-3;
  // trajopt_freespace_solver_profile->opt_info.cnt_tolerance= 9999.0;

  // ompl_plan_profile->optimize = true;
  // toXMLFile(ompl_plan_profile, "/home/bi3ri/hotspray_ws/ompl.xml");

  auto iterative_spline_parameterization_profile = std::make_shared<IterativeSplineParameterizationProfile>();

  iterative_spline_parameterization_profile->max_velocity_scaling_factor = 0.5;
  iterative_spline_parameterization_profile->max_acceleration_scaling_factor = 0.5;

  // Create Program
  CompositeInstruction program = createSprayProgram(req.raster_array, mi);

  // Create a seed
  auto cur_state = env_->getCurrentState();
  // CompositeInstruction seed = generateSeed(program, cur_state, env_);

  auto sample_markers = HotsprayUtils::convertToAxisMarkers(spray_eigen_samples, "world", "spray_poses");
  vis_pub_.publish(sample_markers);

  // Create Process Planning Server
  ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 12);
  planning_server.loadDefaultProcessPlanners();

  // Create Process Planning Request
  ProcessPlanningRequest request;
  // request.name = process_planner_names::RASTER_CT_PLANNER_NAME;
  request.name = process_planner_names::RASTER_FT_PLANNER_NAME;

  // request.name = process_planner_names::DESCARTES_PLANNER_NAME;

  // request.name = process_planner_names::

  // request.instructions = Instruction(program);
  request.instructions = Instruction(program);
  // request.profile = true;

  // Add profiles to planning server
  auto default_simple_plan_profile = std::make_shared<SimplePlannerLVSPlanProfile>();
  ProfileDictionary::Ptr profiles = planning_server.getProfiles();
  // profiles->addProfile<SimplePlannerPlanProfile>(FREESPACE_PROFILE, default_simple_plan_profile);
  // profiles->addProfile<SimplePlannerPlanProfile>(PROCESS_PROFILE, default_simple_plan_profile);
  // profiles->addProfile<SimplePlannerPlanProfile>("scan_programm", default_simple_plan_profile);
  // profiles->addProfile<SimplePlannerPlanProfile>("DEFAULT", default_simple_plan_profile);
  
  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(FREESPACE_PROFILE, trajopt_freespace_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(FREESPACE_PROFILE, trajopt_freespace_composite_profile);
  // profiles->addProfile<TrajOptSolverProfile>(FREESPACE_PROFILE, trajopt_freespace_solver_profile);
  profiles->addProfile<OMPLPlanProfile>(FREESPACE_PROFILE, ompl_profile);
  // profiles->addProfile<IterativeSplineParameterizationProfile>("FREESPACE", iterative_spline_parameterization_profile);

  profiles->addProfile<TrajOptPlanProfile>(PROCESS_PROFILE, trajopt_process_plan_profile);
  profiles->addProfile<TrajOptCompositeProfile>(PROCESS_PROFILE, trajopt_process_composite_profile);
  profiles->addProfile<TrajOptSolverProfile>(PROCESS_PROFILE, trajopt_process_solver_profile);
  // profiles->addProfile<DescartesPlanProfile<float>>(PROCESS_PROFILE, descartes_plan_profile);
  // profiles->addProfile<IterativeSplineParameterizationProfile>(PROCESS_PROFILE, iterative_spline_parameterization_profile);

  // profiles->addProfile<TrajOptPlanProfile>("scan_program", trajopt_plan_profile);
  // profiles->addProfile<TrajOptCompositeProfile>("scan_program", trajopt_composite_profile);
  // profiles->addProfile<TrajOptSolverProfile>("scan_program", trajopt_solver_profile);

  // Print Diagnostics
  request.instructions.print("Program: ");
  // request.plan_profile_remapping = true;

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  // Plot Process Trajectory
  plotter->waitForInput();
  const auto& ci = response.results->as<CompositeInstruction>();
  tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
  plotter->plotTrajectory(trajectory, env_->getStateSolver());
  
  ROS_INFO("Final trajectory is collision free");
  trajectory_msgs::JointTrajectory traj_msg;
  toMsg(traj_msg, trajectory);
  res.traj = traj_msg;

  ROS_INFO("Final trajectory is collision free");
  return true;
}