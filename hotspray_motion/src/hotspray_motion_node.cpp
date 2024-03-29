#include <ros/ros.h>
#include <hotspray_motion/hotspray_motion_server.h>


void sendDebugMsg(HotsprayMotionServer& motion_server, std::string debug_poses_path){
    hotspray_msgs::GenerateSprayTrajectory::Request req;
    hotspray_msgs::GenerateSprayTrajectory::Response res;

    std::vector<geometry_msgs::PoseArray> pose_arrays;
    HotsprayUtils::loadPoseArrayMsgFromJsonFile(pose_arrays, debug_poses_path);

    req.raster_array = pose_arrays;
    motion_server.generateSprayTrajectory(req, res);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hotspray_motion");
    ros::NodeHandle nh;
    HotsprayMotionServer motion_server(nh);

    bool debug;
    std::string debug_poses_path;
    nh.getParam("hotspray_motion/debug", debug);
    nh.getParam("hotspray_motion/debug_poses_path", debug_poses_path);
    if (debug){
        sendDebugMsg(motion_server, debug_poses_path);
        return 0;
    }
    ros::spin(); 

    return 0;
}