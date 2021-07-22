#include <ros/ros.h>
#include <hotspray_motion/hotspray_motion_server.h>




int main(int argc, char** argv)
{
    ros::init(argc, argv, "hotspray_motion");
    ros::NodeHandle nh;
    HotsprayMotionServer motion_server(nh);
    ros::AsyncSpinner spinner(2); 
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
