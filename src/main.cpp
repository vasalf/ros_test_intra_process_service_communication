#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_self_service");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("OK");
    return 0;
}