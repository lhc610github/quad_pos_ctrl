
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pub_node");
    ros::NodeHandle node;
    ros::Publisher test_pub = node.advertise<geometry_msgs::PoseStamped>("/mocap_data_rigid1",10);
    ros::Rate _rate(120);
    ros::Time start_stamp = ros::Time::now();
    while(ros::ok()) {
        geometry_msgs::PoseStamped msg;
        float t = (ros::Time::now() - start_stamp).toSec();
        msg.pose.position.x = 1*sin(t*2*3.14 / 10);
        msg.pose.position.y = 1*cos(t*2*3.14 / 10);
        msg.pose.position.z = 0.5f;
        msg.pose.orientation.w = 1.0f;
        msg.pose.orientation.x = 0.0f;
        msg.pose.orientation.y = 0.0f;
        msg.pose.orientation.z = 0.0f;
        msg.header.stamp = ros::Time::now();
        test_pub.publish(msg);
        ros::spinOnce();
        _rate.sleep();
    }
    return 0;
}