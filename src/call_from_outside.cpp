#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "tss_outside");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("es");
    std_srvs::Empty srv;
    if (client.call(srv)) {
        ROS_INFO("OK");
    } else {
        ROS_ERROR("ERR");
        return 1;
    }
    return 0;
}
