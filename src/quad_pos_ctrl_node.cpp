#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "state_estimate.h"
#include "controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "quad_pos_ctrl_node");
    ros::NodeHandle node("~");
    //State_Estimate estimator(node, 1);
    Controller controller(node, 1);
    ros::spin();
    return 0;
}