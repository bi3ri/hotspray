
//#include <hotspray_motion/hotspray_motion_config.hpp>

#include <hotspray_motion/hotspray_motion_server.h>


//#include <hotspray_motion/include/hotspray_motion_server.h>
//#include <hotspray_motion/hotspray_motion_server.h>

#include <ros/ros.h>



#include "hotspray_msgs/GenerateSprayTrajectory.h"
#include <control_msgs/FollowJointTrajectoryAction.h>


#include <visualization_msgs/MarkerArray.h>
#include "eigen_conversions/eigen_msg.h" //conversion posemsg -> eigen
#include <Eigen/Geometry>


#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
// #include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>


TESSERACT_COMMON_IGNORE_WARNINGS_POP

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

#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>


#include <tinyxml2.h>

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

HotsprayMotionServer::HotsprayMotionServer(ros::NodeHandle nh) : //, std::string config_path) :
    nh_(nh),
    ph_("~"),
    plan_trajectory_service_(ph_.advertiseService("generate_trajectory", &HotsprayMotionServer::generateSprayTrajectory, this)),
    vis_pub_(ph_.advertise<visualization_msgs::MarkerArray>( "toolpath_marker", 0 )),
    env_(std::make_shared<tesseract_environment::Environment>()),
    rviz_(true),
    plotting_(true)
{
    nh_.getParam("arm_controller/joints", joint_names_);

      // joint_names_.push_back("joint_a1");
      // joint_names_.push_back("joint_a2");
      // joint_names_.push_back("joint_a3");
      // joint_names_.push_back("joint_a4");
      // joint_names_.push_back("joint_a5");
      // joint_names_.push_back("joint_a6");
      // joint_names_.push_back("joint_a7");

    home_joint_pos_ = Eigen::VectorXd::Constant(6, 1, 0);
    home_joint_pos_(0) = 2.25;
    home_joint_pos_(1) = -1.50;
    home_joint_pos_(2) = 1.50;
    home_joint_pos_(3) = 0.0;
    home_joint_pos_(4) = 1.50;
    home_joint_pos_(5) = -0.85;

    // home_joint_pos_ = Eigen::VectorXd::Constant(7, 1, 0);
    // home_joint_pos_(0) = 1.8850;
    // home_joint_pos_(1) = 0.6364;
    // home_joint_pos_(2) = -0.0492;
    // home_joint_pos_(3) = -0.9372;
    // home_joint_pos_(4) = 0.4753;
    // home_joint_pos_(5) = 0.0347;
    // home_joint_pos_(6) = 0.1856;
}


void HotsprayMotionServer::createProgramm(tesseract_planning::CompositeInstruction& program, 
                                          const std::vector<geometry_msgs::PoseArray, 
                                          std::allocator<geometry_msgs::PoseArray>>& pose_arrys)
{
        //  Eigen::VectorXd joint_pos(6);


    // Start and End Joint Position for the program

    // geometry_msgs::Pose pose;
    // pose.orientation.x = 0.251757;
    // // pose.orientation.x = 0.203607;

    // pose.orientation.y = 0.459116;
    // pose.orientation.z = 0.719463;
    // pose.orientation.y = 0.509178;
    // pose.orientation.z = 0.535936;
    // pose.orientation.w = 0.492669;
    // Eigen::Isometry3d eigen_pose;
    // tf::poseMsgToEigen(pose, eigen_pose);


    // tesseract_planning::Waypoint home_wp = tesseract_planning::CartesianWaypoint(eigen_pose);

    tesseract_planning::Waypoint home_wp = tesseract_planning::JointWaypoint(joint_names_, home_joint_pos_);

    //Waypoint wp1 = StateWaypoint(joint_names, joint_end_pos);

    tesseract_planning::PlanInstruction start_instruction(home_wp, tesseract_planning::PlanInstructionType::START, "FREESPACE");
    
    program.setStartInstruction(start_instruction);

    // Plan freespace from start

    std::vector<Eigen::Isometry3d> eigen_pose_array;

    Eigen::VectorXd lower_bound(6);
    lower_bound[0] = 0;
    lower_bound[1] = 0;
    lower_bound[2] = 0;
    lower_bound[3] = 0;
    lower_bound[4] = 0;
    // lower_bound[5] = -1;
    lower_bound[5] = -180;


    Eigen::VectorXd upper_bound(6);
    upper_bound[0] = 0;
    upper_bound[1] = 0;
    upper_bound[2] = 0;
    upper_bound[3] = 0;
    upper_bound[4] = 0;
    // upper_bound[5] = 3.14; (Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle())*
    upper_bound[5] = 180;
    std::cout << "wo bin ich????\n\n\n" << std::endl;

  for(const auto& pose_arry : pose_arrys)
    {
      for(const auto& pose : pose_arry.poses){
        std::cout << "adding pose to programm \n" << pose << std::endl;
        Eigen::Isometry3d eigen_pose;
        tf::poseMsgToEigen(pose, eigen_pose);
        eigen_pose_array.push_back(eigen_pose);
        
        tesseract_planning::CartesianWaypoint cw;
        cw.waypoint = eigen_pose;
        // cw.upper_tolerance = upper_bound;
        // cw.lower_tolerance = lower_bound;

        tesseract_planning::Waypoint wp = cw;

        tesseract_planning::PlanInstruction plan(wp, tesseract_planning::PlanInstructionType::FREESPACE, "CARTESIAN");

        program.push_back(plan);
      }
    }

    tesseract_planning::Waypoint home_wp2 = tesseract_planning::JointWaypoint(joint_names_, home_joint_pos_);

    tesseract_planning::PlanInstruction plan_end(home_wp, tesseract_planning::PlanInstructionType::FREESPACE, "FREESPACE");
    program.push_back(plan_end);
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


//tesseract_ros_examples puzzle piece example
bool HotsprayMotionServer::generateSprayTrajectory(hotspray_msgs::GenerateSprayTrajectory::Request &req,
hotspray_msgs::GenerateSprayTrajectory::Response &res)
{
  using tesseract_planning::CartesianWaypoint;
  using tesseract_planning::CompositeInstruction;
  using tesseract_planning::CompositeInstructionOrder;
  using tesseract_planning::Instruction;
  using tesseract_planning::ManipulatorInfo;
  using tesseract_planning::PlanInstruction;
  using tesseract_planning::PlanInstructionType;
  using tesseract_planning::ProcessPlanningFuture;
  using tesseract_planning::ProcessPlanningRequest;
  using tesseract_planning::ProcessPlanningServer;
  using tesseract_planning::StateWaypoint;
  using tesseract_planning::ToolCenterPoint;
  using tesseract_planning::Waypoint;
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
  mi.manipulator = "manipulator_tcp";
  mi.tcp = ToolCenterPoint("tcp_frame", false);
  mi.working_frame = "world";
  // mi.tcp = ToolCenterPoint("grinder_frame", true);  // true - indicates this is an external TCP

  // Create Program
  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, mi);
  createProgramm(program, req.poses);

    // Create Process Planning Server
    ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
    planning_server.loadDefaultProcessPlanners();

    // Create a trajopt taskflow without post collision checking
    /// @todo This matches the original example, but should update to include post collision check 
    const std::string new_planner_name = "TRAJOPT_NO_POST_CHECK";
    tesseract_planning::TrajOptTaskflowParams params;
    params.enable_post_contact_discrete_check = false;
    params.enable_post_contact_continuous_check = true;
    // params.enable_time_parameterization = true;
    planning_server.registerProcessPlanner(new_planner_name,
                                          std::make_unique<tesseract_planning::TrajOptTaskflow>(params));


  // Create TrajOpt Profile
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 10);
  // trajopt_plan_profile->cartesian_coeff(5) = 9;


  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->collision_constraint_config.enabled = false;
  trajopt_composite_profile->collision_cost_config.enabled = true;
  trajopt_composite_profile->collision_cost_config.safety_margin = 0.025;
  trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
  trajopt_composite_profile->collision_cost_config.coeff = 20;
  
  auto test = tesseract_planning::toXMLFile(*trajopt_composite_profile, "/home/bi3ri/hotspray_ws/vontesseract.xml");


  auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
  trajopt_solver_profile->opt_info.max_iter = 200;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
  auto test = tesseract_planning::toXMLFile(*trajopt_solver_profile, "/home/bi3ri/hotspray_ws/vontesseract1.xml");

  tinyxml2::XMLDocument xmldocument;
  tinyxml2::XMLNode* root_node = xmldocument.NewElement("Hotspray Montion Planning Pipeline");
  xmldocument.InsertFirstChild(root_node);
  std::cout << "the fuck" << std::endl;

  auto plan_profile_xml = trajopt_plan_profile->toXML(xmldocument);
  std::cout << "the fuck1" << std::endl;

  root_node->InsertEndChild(plan_profile_xml);
  std::cout << "the fuck2" << std::endl;

  auto composite_profile_xml = trajopt_composite_profile->toXML(xmldocument);
  root_node->InsertEndChild(composite_profile_xml);
  std::cout << "the fuck3" << std::endl;

  auto solver_profile_xml = trajopt_solver_profile->toXML(xmldocument);
  std::cout << "the fuck4.5" << std::endl;

  // root_node->InsertEndChild(solver_profile_xml);
  // std::cout << "the fuck4" << std::endl;

  xmldocument.SaveFile("/home/bi3ri/hotspray_ws/test1.xml");
  std::cout << "the fuck5" << std::endl;

  
  // Add profile to Dictionary
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN", trajopt_plan_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                         trajopt_composite_profile);
  planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                      trajopt_solver_profile);


  // Create Process Planning Request
  ProcessPlanningRequest request;
  request.name = new_planner_name;
  request.instructions = Instruction(program);


  // Create Naive Seed
  /** @todo Need to improve simple planners to support external tcp definitions */
  tesseract_planning::CompositeInstruction naive_seed;
  {
    auto lock = monitor_->lockEnvironmentRead();
    naive_seed = tesseract_planning::generateNaiveSeed(program, *(monitor_->getEnvironment()));
  }
  request.seed = Instruction(naive_seed);

  // Print Diagnostics
  request.instructions.print("Program: ");
  request.seed.print("Seed: ");

  if (rviz_)
    plotter->waitForInput();

  // Solve process plan
  ProcessPlanningFuture response = planning_server.run(request);
  planning_server.waitForAll();

  tesseract_common::JointTrajectory trajectory;

  // Plot Process Trajectory
  if (rviz_ && plotter != nullptr && plotter->isConnected())
  {
    plotter->waitForInput();
    const auto& ci = response.results->as<tesseract_planning::CompositeInstruction>();
    trajectory = tesseract_planning::toJointTrajectory(ci);
    plotter->plotTrajectory(trajectory, env_->getStateSolver());
  }

  ROS_INFO("Final trajectory is collision free");

  // namespace tesseract_ros_examples
    
    ROS_INFO("Final trajectory is collision free");
    trajectory_msgs::JointTrajectory traj_msg;
    toMsg(traj_msg, trajectory);
    res.traj = traj_msg;
  return true;
}



/**
 * @brief Generate a JointTrajectory Message that contains only trajectory joints
 * @param traj_msg The output JointTrajectory Message
 * @param joint_names The joint names corresponding to the trajectory
 * @param traj The joint trajectory
 */
// void toMsg(std::vector<tesseract_msgs::JointState>& traj_msg, const tesseract_common::JointTrajectory& traj);

