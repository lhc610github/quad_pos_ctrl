#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "state_estimate.h"
#include "controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "quad_pos_ctrl_node");
    //ros::NodeHandle node("~");
    //State_Estimate estimator(node, 1);
    int id = 3;
    Controller controller(id);
	ros::AsyncSpinner spinner(4 /* threads */);
    spinner.start();
    ROS_INFO("quad_pos_ctrl_node start");
    ros::waitForShutdown();
    ROS_INFO("quad_pos_ctrl_node stop");
    spinner.stop();
    return 0;
}