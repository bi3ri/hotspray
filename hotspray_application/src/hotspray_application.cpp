
#include <hotspray_application.hpp>
#include "eigen_conversions/eigen_msg.h" //conversion posemsg -> eigen

#include "std_msgs/Float64MultiArray.h"
#include <Eigen/StdVector>
#include <eigen_conversions/eigen_msg.h>


const std::string YAK_SCAN_FRAME = "tsdf_origin";
const std::string PLY_NAME = "/results_mesh.ply";

const std::string EXECUTE_TRAJECTORY_ACTION = "execute_trajectory";

HotsprayApplication::HotsprayApplication(ros::NodeHandle nh) :
    nh_(nh),
    ph_("~"),
    mesh_client_(nh_.serviceClient<yak_ros_msgs::GenerateMesh>("/tsdf_node/generate_mesh")),
    toolpath_client_(nh_.serviceClient<hotspray_msgs::GenerateToolpath>("/plane_slicer_example/generate_toolpath")),
    trajectory_client_(nh_.serviceClient<hotspray_msgs::GenerateSprayTrajectory>("/hotspray_motion/generate_trajectory")),
    tubular_toolpath_client_(nh_.serviceClient<tubular_toolpath_creator::GenerateTubularToolpath>("/tubular_toolpath_creator/create_tubular_toolpath")),
    scan_pose_publisher_(nh_.advertise<visualization_msgs::MarkerArray>("scan_poses", 1)),
    moveit_action_client_ptr_(std::make_shared<client_type>("execute_trajectory",true)),
    mesh_path_(ros::package::getPath("hotspray_application") + "/meshs")
{
}

bool HotsprayApplication::createScanTrajectory(trajectory_msgs::JointTrajectory& trajectory){
    std::cout << "This will create a new scan trajectory. Please insert a file name!" << std::endl;
    std::string file_name;
    std::getline(std::cin, file_name);

    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose;
    tf::TransformListener tf_listener;
    tf::StampedTransform tf;

    while(1){

        std::cout << "Do you want to add the current robot position to the trajectory? (Answer: y/n)" << std::endl;
        char ch;
        do
        {
        ch = std::getchar();
        } while(ch!='y'&& ch!='n' && ch!='Y'&& ch!='N');

        if(ch == 'n' || ch == 'N')
            break;

        try{
        tf_listener.lookupTransform("/world", "/tcp_frame", ros::Time(0), tf);
        }
        catch (tf::TransformException &ex) { 
            ROS_ERROR("hallo");
        }
        
        this->tfToPose(tf, pose);

        pose_array.poses.push_back(pose);

        std::cout << "The following pose was added to the trajectory!\n" << pose << "\n" << std::endl;
    }

    std::vector<geometry_msgs::PoseArray> pose_arrays = {pose_array};

    this->generateTrajectory(pose_arrays, trajectory);

    HotsprayUtils::saveTrajectoryToFile(trajectory, file_name);
    HotsprayUtils::savePoseArrayToFile(pose_array, file_name);
    
    std::cout << "\n\n\n\n\n\n\n Sucessfully saved trajectory and poses!" << std::endl;

    return 0;
}

bool HotsprayApplication::generateMesh(){
    yak_ros_msgs::GenerateMesh mesh_srv;
    mesh_srv.request.results_dir =  mesh_path_ ;//+ PLY_NAME;
    mesh_srv.request.results_frame = YAK_SCAN_FRAME;

    if(mesh_client_.call(mesh_srv))
    {
        ROS_INFO(" success");
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to call service generate mesh");
        return -1;
    }
}

bool HotsprayApplication::generateToolpath(std::vector<geometry_msgs::PoseArray>& pose_arrays){
    hotspray_msgs::GenerateToolpath toolpath_srv;
    toolpath_srv.request.mesh_path = mesh_path_ + PLY_NAME;
    //toolpath_srv.request.mesh_path = "/home/bieri/hotspray_ws/src/noether/noether_examples/data/raw_mesh.ply";

    if(toolpath_client_.call(toolpath_srv))
    {
        std::vector<noether_msgs::ToolPaths> raster_paths = toolpath_srv.response.tool_paths;
        noetherMsgtoPoseArrayMsg(raster_paths, pose_arrays);
        ROS_INFO("toolpath gen success");
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to call service generate toolpath");
        return -1;
    }

    this->noetherMsgtoPoseArrayMsg(toolpath_srv.response.tool_paths, pose_arrays);

    return 1;
}

bool HotsprayApplication::generateTubularToolpath(std_msgs::Float64MultiArray& pose_vector_array){
    tubular_toolpath_creator::GenerateTubularToolpath tubular_toolpath_srv;
    tubular_toolpath_srv.request.mesh_path = mesh_path_ + PLY_NAME;


    if(tubular_toolpath_client_.call(tubular_toolpath_srv))
    {
        // std::vector<geometry_msgs::PoseArray> 
        pose_vector_array = tubular_toolpath_srv.response.toolpath_vector_array;

        // noetherMsgtoPoseArrayMsg(raster_paths, pose_arrays);
        ROS_INFO("toolpath gen success");
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to call service generate toolpath");
        return -1;
    }

    // this->noetherMsgtoPoseArrayMsg(tubular_toolpath_srv.response.tool_paths, pose_arrays);

    return 1;
}

bool HotsprayApplication::generateTrajectory(std::vector<geometry_msgs::PoseArray>& pose_arrays, trajectory_msgs::JointTrajectory& trajectory){
    hotspray_msgs::GenerateSprayTrajectory trajectory_srv;

    for(geometry_msgs::PoseArray pose_array : pose_arrays){
        pose_array.header.frame_id = "tsdf_origin";
        trajectory_srv.request.poses.push_back(pose_array);
    }

    if(trajectory_client_.call(trajectory_srv))
    {
        trajectory = trajectory_srv.response.traj;
        ROS_INFO("trajectory gen success");
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to call service hotspray_motion");
        return -1;
    }
}

bool HotsprayApplication::executeTrajectory(trajectory_msgs::JointTrajectory& trajectory){

    if(moveit_action_client_ptr_->waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO_STREAM("Connected to '"<<EXECUTE_TRAJECTORY_ACTION<<"' action");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to '"<<EXECUTE_TRAJECTORY_ACTION<<"' action");
        exit(-1);
    }
    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory.joint_trajectory = trajectory;
    std::vector<std::string> joint_names;
    nh_.getParam("/arm_controller/joints", joint_names);
    goal.trajectory.joint_trajectory.joint_names = joint_names;

    moveit_action_client_ptr_->sendGoalAndWait(goal);
        ROS_INFO("trajectory ex success"); //TODO

    return 0;
}

bool HotsprayApplication::run()
{
    trajectory_msgs::JointTrajectory scan_trajectory;


    //create poses
    //createScanTrajectory(scan_trajectory);
    // HotsprayUtils::loadTrajectoryFromFile(scan_trajectory, "test");

    //load poses
    // geometry_msgs::PoseArray pose_array;

    // HotsprayUtils::loadPoseArrayFromFile(pose_array, "coil");
    //     for(int i = 0; i < 3; i++){
    //         pose_array.poses.push_back(pose_array.poses[i]);
    // }
    // std::vector<geometry_msgs::PoseArray> scan_pose_arrays = {pose_array};

    // generateTrajectory(scan_pose_arrays, scan_trajectory);

    // executeTrajectory(scan_trajectory);

    // generateMesh();

    std_msgs::Float64MultiArray pose_vector_array;
    generateTubularToolpath(pose_vector_array);

    std::vector<geometry_msgs::PoseArray> spray_pose_array;
    HotsprayUtils::convertResponseArrayToPoseArray(pose_vector_array, spray_pose_array);

    std::vector<Eigen::Isometry3d> eigen_pose_array;
    HotsprayUtils::convertResponseArrayToPoseArray(pose_vector_array, eigen_pose_array);


    // Eigen::Affine3d r = HotsprayUtils::create_rotation_matrix(0, 180, 0);
    // Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(0.2, 0.2, 0.55)));
    // Eigen::Matrix4d m = (r * t).matrix(); 

    // Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
    // pattern_origin.translation() = Eigen::Vector3d(0.2, 0.2, 0.55);QQ

    // Eigen::Isometry3d eigen_pose;
    // int i =0;
    // for(auto& pose_arrys : spray_pose_arrays)
    // {
    //     for(auto& pose : pose_arrys.poses){
    //         tf::poseMsgToEigen(pose, eigen_pose);
    //         // std::cout << pose << std::endl;
    //         // if (i%2 == 1)
    //         //     eigen_pose = eigen_pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

    //         // eigen_pose = eigen_pose * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
        
    //         // eigen_pose = eigen_pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    //         //eigen_pose = eigen_pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()); // this flips the tool around so that Z is down
    //         // eigen_pose.pretranslate(Eigen::Vector3d(0.2, 0.2, 0.55));
    //         //eigen_pose.rotate(r);
    //         tf::poseEigenToMsg(eigen_pose, pose);
    //         // std::cout << pose << std::endl;
    //     }
    //     i++;
    // }

    // HotsprayUtils::applyTranformationToPoseArray(spray_pose_arrays, 0.2, 0.2, 0.6, 0, 0, 0);

    visualization_msgs::MarkerArray scan_markers;
    scan_markers = HotsprayUtils::convertToAxisMarkers(spray_pose_array, "world", "scan_poses");
    scan_pose_publisher_.publish(scan_markers);


    trajectory_msgs::JointTrajectory trajectory;
    generateTrajectory(spray_pose_array, trajectory);

    executeTrajectory(trajectory);

    return 0;
}

void HotsprayApplication::noetherMsgtoPoseArrayMsg(std::vector<noether_msgs::ToolPaths>& raster_paths, std::vector<geometry_msgs::PoseArray>& pose_array){

    for(noether_msgs::ToolPaths& raster_path : raster_paths){
        for(noether_msgs::ToolPath& path : raster_path.paths){
            for(geometry_msgs::PoseArray& segments : path.segments){
                segments.header.frame_id = "tsdf_origin";
                pose_array.push_back(segments);
            }
        }
    }

}

void HotsprayApplication::tfToPose(const tf::StampedTransform& tf, geometry_msgs::Pose& pose){
        pose.position.x = tf.getOrigin().x();
        pose.position.y = tf.getOrigin().y();
        pose.position.z = tf.getOrigin().z();
        pose.orientation.w = tf.getRotation().getW();
        pose.orientation.x = tf.getRotation().getX();
        pose.orientation.y = tf.getRotation().getY();
        pose.orientation.z = tf.getRotation().getZ();
}

