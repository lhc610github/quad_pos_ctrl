#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>

std::string logger_file_name = "/home/lhc/work/demo_ws/src/quad_pos_ctrl/src/logger/mocap_data.csv";
std::ofstream logger;

void mocap_cb(const geometry_msgs::PoseStamped& msg) {
        logger << msg.header.stamp.toNSec() << ',';
        logger << msg.pose.position.x << ',';
        logger << msg.pose.position.y << ',';
        logger << msg.pose.position.z << ',';
        logger << msg.pose.orientation.w << ',';
        logger << msg.pose.orientation.x << ',';
        logger << msg.pose.orientation.y << ',';
        logger << msg.pose.orientation.z << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pub_node");
    ros::NodeHandle node; 
    ros::Subscriber mocap_sub = node.subscribe("/mocap_data_rigid5",10,mocap_cb);
    logger.open(logger_file_name.c_str(), std::ios::out);
    if (logger.is_open()) {
        logger << "timestamp" << ',';
        logger << "mocap_x(ned)" << ',';
        logger << "mocap_y(ned)" << ',';
        logger << "mocap_z(ned)" << ',';
        logger << "mocap_q_w" << ',';
        logger << "mocap_q_x" << ',';
        logger << "mocap_q_y" << ',';
        logger << "mocap_q_z" << std::endl;
        ros::spin();
    }
    return 0;
}