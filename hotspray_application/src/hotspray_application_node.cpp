
#include <hotspray_application.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hotspray_application");
    ros::NodeHandle nh;

    HotsprayApplication app(nh);
    app.run();
}