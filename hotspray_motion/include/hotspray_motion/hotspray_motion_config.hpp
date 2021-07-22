


#include <descartes_light/descartes_light.h>



#include <tesseract_common/types.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>


#include <yaml-cpp/yaml.h>


#include <Eigen/Eigen>
#include <vector>





namespace hotspay_motion_config
{


struct descartesConfig
{
  using Ptr = std::shared_ptr<descartesConfig>;

  double axial_step = M_PI / 36;
  double collision_safety_margin = 0.01;

  bool allow_collisions = false;

  Eigen::Isometry3d tool_offset = Eigen::Isometry3d::Identity();

  std::vector<double> joint_min_vals;
};

struct trajoptFreespaceConfig
{
  bool smooth_velocities = true;
  bool smooth_accelerations = true;
  bool smooth_jerks = true;

  //tesseract_motion_planners::CollisionCostConfig coll_cst_cfg;
  //tesseract_motion_planners::CollisionConstraintConfig coll_cnt_cfg;

  trajopt::InitInfo::Type init_type = trajopt::InitInfo::GIVEN_TRAJ;

  double longest_valid_segment_fraction = 0.01;
  double longest_valid_segment_length = 0.5;

  tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::CLOSEST;

  std::vector<std::tuple<std::string, std::string, double, double>> special_collision_cost;
  std::vector<std::tuple<std::string, std::string, double, double>> special_collision_constraint;
};

struct pathPlanningConfig
{
  descartesConfig descartes_config;

    // trajopt_surface_config;

  //omplConfig ompl_config;

  //trajoptFreespaceConfig trajopt_freespace_config;
  bool use_trajopt_freespace = true;
  bool use_trajopt_surface = true;
  bool default_to_descartes = false;
  bool default_to_ompl = true;

  //tesseract::Tesseract::Ptr tesseract_local;

  //tesseract

  bool use_start = false;
  //tesseract_motion_planners::JointWaypoint::Ptr start_pose;
  bool use_end = false;
  //tesseract_motion_planners::JointWaypoint::Ptr end_pose;
  bool simplify_start_end_freespace = true;

  std::vector<geometry_msgs::PoseArray> rasters;  // In world frame

    
  std::string manipulator = "manipulator";

  std::string world_frame = "world";
  std::string robot_base_frame = "base_link";
  std::string tool0_frame = "tool0";
  std::string tcp_frame;

  Eigen::Isometry3d tool_offset = Eigen::Isometry3d::Identity();

  bool add_approach_and_retreat = false;
  double approach_distance = 0.05;
  double retreat_distance = 0.05;

  bool required_tool_vel = false;
  double tool_speed = 0.03;    // m/s
  double max_joint_vel = 0.2;  // rad/s
  double max_joint_vel_mult = 1.0;
  double max_surface_dist = 0.1;
  double max_rotation_rate = 3.0;  // rad/m
  double max_joint_acc = 0.5;      // rad/s^2

  size_t minimum_raster_length = 2;
  double reachable_radius = 1.0;

  bool trajopt_verbose_output = false;

  bool use_gazebo_sim_timing = false;

  bool combine_strips = false;
  bool global_descartes = true;
};



bool loadPathPlanningConfig(const std::string& yaml_fp, pathPlanningConfig& motion_planner_config){


  YAML::Node full_yaml_node = YAML::LoadFile(yaml_fp);
  if (!full_yaml_node)
  {
    ROS_ERROR("Failed to load into YAML from file %s", yaml_fp.c_str());
    return false;
  }


    try
  {
    // DESCARTES CONFIG
    descartesConfig descartes_config;
    YAML::Node descartes_yaml = full_yaml_node["descartes"];
    descartes_config.axial_step = descartes_yaml["axial_step"].as<double>();
    descartes_config.collision_safety_margin = descartes_yaml["collision_safety_margin"].as<double>();
    std::vector<double> xyzrpy = descartes_yaml["additional_tool_offset"].as<std::vector<double>>();
    descartes_config.tool_offset = Eigen::Translation3d(Eigen::Vector3d(xyzrpy[0], xyzrpy[1], xyzrpy[2])) *
                                   Eigen::AngleAxisd(xyzrpy[3], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(xyzrpy[4], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(xyzrpy[5], Eigen::Vector3d::UnitZ());
    descartes_config.allow_collisions = false;
    descartes_config.joint_min_vals = descartes_yaml["joint_min_vals"].as<std::vector<double>>();



    /*
    // TRAJOPT SURFACE CONFIG
    YAML::Node trajopt_surface_yaml = full_yaml_node["trajopt_surface"];
    trajopt_surface_config.smooth_velocities = trajopt_surface_yaml["smooth_velocities"].as<bool>();
    trajopt_surface_config.smooth_accelerations = trajopt_surface_yaml["smooth_accelerations"].as<bool>();
    trajopt_surface_config.smooth_jerks = trajopt_surface_yaml["smooth_jerks"].as<bool>();
    trajopt_surface_config.longest_valid_segment_fraction =
        trajopt_surface_yaml["longest_valid_segment_fraction"].as<double>();
    trajopt_surface_config.longest_valid_segment_length =
        trajopt_surface_yaml["longest_valid_segment_length"].as<double>();
    std::vector<double> xyzrpy_surface_co = trajopt_surface_yaml["surface_coefficients"].as<std::vector<double>>();
    Eigen::VectorXd surface_coeffs(6);
    surface_coeffs << xyzrpy_surface_co[0], xyzrpy_surface_co[1], xyzrpy_surface_co[2], xyzrpy_surface_co[3],
        xyzrpy_surface_co[4], xyzrpy_surface_co[5];
    trajopt_surface_config.surface_coeffs = surface_coeffs;
    trajopt_surface_config.waypoints_critical = trajopt_surface_yaml["waypoints_critical"].as<bool>();
    // trajopt surface collision configs
    tesseract_motion_planners::CollisionCostConfig coll_cost_config_srfc;
    coll_cost_config_srfc.enabled = trajopt_surface_yaml["collision_cost"]["enabled"].as<bool>();
    if (coll_cost_config_srfc.enabled)
    {
      coll_cost_config_srfc.buffer_margin = trajopt_surface_yaml["collision_cost"]["buffer_margin"].as<double>();
    }
    trajopt_surface_config.coll_cst_cfg = coll_cost_config_srfc;
    tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config_srfc;
    coll_cnt_config_srfc.enabled = trajopt_surface_yaml["collision_constraint"]["enabled"].as<bool>();
    if (coll_cnt_config_srfc.enabled)
    {
      coll_cnt_config_srfc.safety_margin = trajopt_surface_yaml["collision_constraint"]["safety_margin"].as<double>();
    }
    trajopt_surface_config.coll_cnt_cfg = coll_cnt_config_srfc;
    // trajopt surface special collision pairs
    if (trajopt_surface_yaml["special_collision_costs"])
    {
      YAML::Node special_costs = trajopt_surface_yaml["special_collision_costs"];
      for (const auto& cost_node : special_costs)
      {
        std::string link1 = cost_node["link1"].as<std::string>();
        std::string link2 = cost_node["link2"].as<std::string>();
        double distance = cost_node["distance"].as<double>();
        double cost = cost_node["cost"].as<double>();
        trajopt_surface_config.special_collision_cost.push_back({ link1, link2, distance, cost });
      }
    }
    if (trajopt_surface_yaml["special_collision_constraints"])
    {
      YAML::Node special_constraints = trajopt_surface_yaml["special_collision_constraints"];
      for (const auto& constraint_node : special_constraints)
      {
        std::string link1 = constraint_node["link1"].as<std::string>();
        std::string link2 = constraint_node["link2"].as<std::string>();
        double distance = constraint_node["distance"].as<double>();
        double cost = constraint_node["cost"].as<double>();
        trajopt_surface_config.special_collision_constraint.push_back({ link1, link2, distance, cost });
      }
    }
    */

/*
    // OMPL CONFIG
    YAML::Node ompl_yaml = full_yaml_node["ompl"];
    ompl_config.collision_safety_margin = ompl_yaml["collision_safety_margin"].as<double>();
    ompl_config.planning_time = ompl_yaml["planning_time"].as<double>();
    ompl_config.simplify = ompl_yaml["simplify"].as<bool>();
    ompl_config.range = ompl_yaml["range"].as<double>();
    ompl_config.num_threads = ompl_yaml["num_threads"].as<int>();
    ompl_config.max_solutions = ompl_yaml["max_solutions"].as<int>();
    ompl_config.n_output_states = ompl_yaml["default_n_output_states"].as<double>();
    ompl_config.longest_valid_segment_fraction = ompl_yaml["longest_valid_segment_fraction"].as<double>();
    ompl_config.longest_valid_segment_length = ompl_yaml["longest_valid_segment_length"].as<double>();
    ompl_config.retry_failure = ompl_yaml["retry_failure"].as<bool>();
    ompl_config.retry_distance = ompl_yaml["retry_distance"].as<double>();
    ompl_config.retry_at_zero = ompl_yaml["retry_at_zero"].as<bool>();

    // TRAJOPT FREESPACE CONFIG
    YAML::Node trajopt_freespace_yaml = full_yaml_node["trajopt_freespace"];
    trajopt_freespace_config.smooth_velocities = trajopt_freespace_yaml["smooth_velocities"].as<bool>();
    trajopt_freespace_config.smooth_accelerations = trajopt_freespace_yaml["smooth_accelerations"].as<bool>();
    trajopt_freespace_config.smooth_jerks = trajopt_freespace_yaml["smooth_jerks"].as<bool>();
    trajopt_freespace_config.longest_valid_segment_fraction =
        trajopt_freespace_yaml["longest_valid_segment_fraction"].as<double>();
    trajopt_freespace_config.longest_valid_segment_length =
        trajopt_freespace_yaml["longest_valid_segment_length"].as<double>();
    // trajopt freespace collision configs
    tesseract_motion_planners::CollisionCostConfig coll_cost_config_fs;
    coll_cost_config_fs.enabled = trajopt_freespace_yaml["collision_cost"]["enabled"].as<bool>();
    if (coll_cost_config_fs.enabled)
    {
      coll_cost_config_fs.buffer_margin = trajopt_freespace_yaml["collision_cost"]["buffer_margin"].as<double>();
    }
    trajopt_freespace_config.coll_cst_cfg = coll_cost_config_fs;
    tesseract_motion_planners::CollisionConstraintConfig coll_cnt_config_fs;
    coll_cnt_config_fs.enabled = trajopt_freespace_yaml["collision_constraint"]["enabled"].as<bool>();
    if (coll_cnt_config_fs.enabled)
    {
      coll_cnt_config_fs.safety_margin = trajopt_freespace_yaml["collision_constraint"]["safety_margin"].as<double>();
    }
    trajopt_freespace_config.coll_cnt_cfg = coll_cnt_config_fs;
    // trajopt freespace special collision pairs
    if (trajopt_freespace_yaml["special_collision_costs"])
    {
      YAML::Node special_costs = trajopt_freespace_yaml["special_collision_costs"];
      for (const auto& cost_node : special_costs)
      {
        std::string link1 = cost_node["link1"].as<std::string>();
        std::string link2 = cost_node["link2"].as<std::string>();
        double distance = cost_node["distance"].as<double>();
        double cost = cost_node["cost"].as<double>();
        trajopt_freespace_config.special_collision_cost.push_back({ link1, link2, distance, cost });
      }
    }
    if (trajopt_freespace_yaml["special_collision_constraints"])
    {
      YAML::Node special_constraints = trajopt_freespace_yaml["special_collision_constraints"];
      for (const auto& constraint_node : special_constraints)
      {
        std::string link1 = constraint_node["link1"].as<std::string>();
        std::string link2 = constraint_node["link2"].as<std::string>();
        double distance = constraint_node["distance"].as<double>();
        double cost = constraint_node["cost"].as<double>();
        trajopt_freespace_config.special_collision_constraint.push_back({ link1, link2, distance, cost });
      }
    }
*/

    // GENERAL CONFIG
    YAML::Node general_yaml = full_yaml_node["general"];
    motion_planner_config.use_trajopt_freespace = general_yaml["use_trajopt_freespace"].as<bool>();
    motion_planner_config.use_trajopt_surface = general_yaml["use_trajopt_surface"].as<bool>();
    motion_planner_config.default_to_descartes = general_yaml["default_to_descartes"].as<bool>();
    motion_planner_config.default_to_ompl = general_yaml["default_to_ompl"].as<bool>();
    motion_planner_config.simplify_start_end_freespace = general_yaml["simplify_start_end_freespace"].as<bool>();
    motion_planner_config.manipulator = general_yaml["manipulator"].as<std::string>();
    motion_planner_config.world_frame = general_yaml["world_frame"].as<std::string>();
    motion_planner_config.robot_base_frame = general_yaml["robot_base_frame"].as<std::string>();
    motion_planner_config.tool0_frame = general_yaml["tool0_frame"].as<std::string>();
    motion_planner_config.required_tool_vel = general_yaml["required_tool_vel"].as<bool>();
    motion_planner_config.max_joint_vel = general_yaml["max_joint_vel"].as<double>();
    motion_planner_config.max_joint_vel_mult = general_yaml["max_joint_vel_mult"].as<double>();
    motion_planner_config.max_surface_dist = general_yaml["max_surface_dist"].as<double>();
    motion_planner_config.max_rotation_rate = general_yaml["max_rotation_rate"].as<double>();
    motion_planner_config.max_joint_acc = general_yaml["max_joint_acc"].as<double>();
    motion_planner_config.add_approach_and_retreat = general_yaml["add_approach_and_retreat"].as<bool>();
    motion_planner_config.minimum_raster_length = general_yaml["minimum_raster_length"].as<std::size_t>();
    motion_planner_config.reachable_radius = general_yaml["reachable_radius"].as<double>();
    motion_planner_config.trajopt_verbose_output = general_yaml["trajopt_verbose_output"].as<bool>();
    motion_planner_config.combine_strips = general_yaml["combine_strips"].as<bool>();
    motion_planner_config.global_descartes = general_yaml["global_descartes"].as<bool>();
    motion_planner_config.descartes_config = descartes_config;
    motion_planner_config.trajopt_surface_config = trajopt_surface_config;
    //motion_planner_config.ompl_config = ompl_config;
    //motion_planner_config.trajopt_freespace_config = trajopt_freespace_config;
    return true;
  }
  catch (YAML::InvalidNode& e)
  {
    ROS_ERROR("Invalid node while parsing yaml %s, %s", yaml_fp.c_str(), e.what());
    return false;
  }
  catch (YAML::BadConversion& e)
  {
    ROS_ERROR("failed to parse yaml %s, %s", yaml_fp.c_str(), e.what());
    return false;
  }
  catch (YAML::KeyNotFound& e)
  {
    ROS_ERROR("Key not found while parsing %s, %s", yaml_fp.c_str(), e.what());
    return false;
  }

}











} // namespace hotspay_motion_config