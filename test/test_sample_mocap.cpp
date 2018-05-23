#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include "diff_intigral_cal.h"
#include "sliding_differentiation.h"
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>

std::string logger_file_name = "/home/lhc/work/demo_ws/src/quad_pos_ctrl/test/logger/mocap_data.csv";
std::ofstream logger;
Diff_State<Eigen::Vector3d> Pos_differ_(120.0f,60.0f);
Diff_State<Eigen::Vector3d> Vel_differ_(60.0f,30.0f);
Sliding_diff<Eigen::Vector3d> sliding_differ_(4);

void mocap_cb(const geometry_msgs::PoseStamped& msg) {
        logger << msg.header.stamp.toNSec() << ',';
        logger << msg.pose.position.x << ',';
        logger << msg.pose.position.y << ',';
        logger << msg.pose.position.z << ',';
        logger << msg.pose.orientation.w << ',';
        logger << msg.pose.orientation.x << ',';
        logger << msg.pose.orientation.y << ',';
        logger << msg.pose.orientation.z << ',';
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Vector3d acc;
        pos << msg.pose.position.x,msg.pose.position.y,msg.pose.position.z;

        /* linear differ */
        Pos_differ_.update(pos, msg.header.stamp);
        Eigen::Vector3d temp;
        if (Pos_differ_.get_diff(temp)) {
            vel = temp;
            Vel_differ_.update(vel, msg.header.stamp);
            if (Vel_differ_.get_diff(temp)) {
                acc = temp;
            } else {
                acc = Eigen::Vector3d::Zero();
            }
        } else {
            vel = Eigen::Vector3d::Zero();
            acc = Eigen::Vector3d::Zero();
        }
        logger << vel(0) << ',';
        logger << vel(1) << ',';
        logger << vel(2) << ',';
        logger << acc(0) << ',';
        logger << acc(1) << ',';
        logger << acc(2) << ',';

        /* sliding differ */
        if(sliding_differ_.get_status()) {
            sliding_differ_.update(pos, msg.header.stamp);
            sliding_differ_.get_diff(pos,0);
            sliding_differ_.get_diff(vel,1);
            sliding_differ_.get_diff(acc,2);
        } else {
            sliding_differ_.init_diff(pos, msg.header.stamp);
            vel = Eigen::Vector3d::Zero();
            acc = Eigen::Vector3d::Zero();
        }
        logger << pos(0) << ',';
        logger << pos(1) << ',';
        logger << pos(2) << ',';
        logger << vel(0) << ',';
        logger << vel(1) << ',';
        logger << vel(2) << ',';
        logger << acc(0) << ',';
        logger << acc(1) << ',';
        logger << acc(2) << std::endl;

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
        logger << "mocap_q_z" << ',';
        logger << "vel_x(linear_diff)" << ',';
        logger << "vel_y(linear_diff)" << ',';
        logger << "vel_z(linear_diff)" << ',';
        logger << "acc_x(linear_diff)" << ',';
        logger << "acc_y(linear_diff)" << ',';
        logger << "acc_z(linear_diff)" << ',';
        logger << "pos_x(sliding_diff)" << ',';
        logger << "pos_y(sliding_diff)" << ',';
        logger << "pos_z(sliding_diff)" << ',';
        logger << "vel_x(sliding_diff)" << ',';
        logger << "vel_y(sliding_diff)" << ',';
        logger << "vel_z(sliding_diff)" << ',';
        logger << "acc_x(sliding_diff)" << ',';
        logger << "acc_y(sliding_diff)" << ',';
        logger << "acc_z(sliding_diff)" << std::endl;
        ros::spin();
    }
    return 0;
}