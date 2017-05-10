/*
Copyright (c) 2017 by Vasily Alferov

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <thread>
#include <random>

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

void exec_thread(int qrate, ros::NodeHandle &nh) {
    ros::Rate rate(qrate);
    while (ros::ok()) {
        ros::spinOnce();
        if (!ping_myself(nh))
            ROS_ERROR("Fail...");
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_self_service");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("es", &accept_request);

    const int num_threads = 20;
    std::thread ts[num_threads];
    std::random_device rnd;
    std::uniform_int_distribution<int> dist(1, 10);
    for (int i = 0; i < num_threads; i++) {
        ts[i] = std::thread(exec_thread, dist(rnd), std::ref(nh));
    }
    for (int i = 0; i < num_threads; i++) {
        ts[i].join();
    }

    return 0;
}
