#include <ros/ros.h>
#include <std_srvs/Empty.h>

bool accept_request(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response & res) {
    ROS_INFO("Accepted!");
    return true;
}

bool ping_myself(ros::NodeHandle& nh) {
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("es");
    std_srvs::Empty srv;
    if (!client.isValid()) {
        ROS_INFO("The client is not valid.");
        return false;
    }
    return client.call(srv);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_self_service");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("es", &accept_request);

    ros::Rate rate(2);
    while (ros::ok()) {
        ros::spinOnce();
        ROS_INFO("I try...");
        if (ping_myself(nh)) {
            ROS_INFO("This worked!");
        }
        ROS_INFO("Then I sleep");
        
        rate.sleep();
    }

    return 0;
}
