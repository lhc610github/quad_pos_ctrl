
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "quad_pos_ctrl/SetTakeoffLand.h"
#include "quad_pos_ctrl/ctrl_ref.h"
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_takeoff_node2");
    ros::NodeHandle node;
    ros::ServiceClient takeoff_land_srv = node.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node1/controller/takeoff_land");
    ros::Publisher ctrl_ref_pub = node.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node1/controller/ctrl_ref",10);

    quad_pos_ctrl::SetTakeoffLand set_takeoff_land;
    set_takeoff_land.request.takeoff = true;
    set_takeoff_land.request.takeoff_altitude = 0.5f;
    takeoff_land_srv.call(set_takeoff_land);
    ROS_INFO("takeoff test");
    sleep(5);

    quad_pos_ctrl::ctrl_ref ctrl_ref_msg;
    /*ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 0.0f;
    ctrl_ref_msg.pos_ref[1] = 2.0f;
    ctrl_ref_msg.pos_ref[2] = -0.7f;
    ctrl_ref_msg.yaw_ref = -1.57f;
    ctrl_ref_msg.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover1 test");*/

    //sleep(5);

    ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 0.0f;
    ctrl_ref_msg.pos_ref[1] = 0.0f;
    ctrl_ref_msg.pos_ref[2] = -0.7f;
    ctrl_ref_msg.yaw_ref = 0.0f;
    ctrl_ref_msg.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover1 test");

    sleep(1);

    ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 1.0f;
    ctrl_ref_msg.pos_ref[1] = 0.0f;
    ctrl_ref_msg.pos_ref[2] = -0.7f;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover2 test");

    sleep(1);

    ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 0.0f;
    ctrl_ref_msg.pos_ref[1] = 0.0f;
    ctrl_ref_msg.pos_ref[2] = -0.7f;
    ctrl_ref_msg.yaw_ref = -1.0f;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover3 test");

    sleep(100);

    ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 0.0f;
    ctrl_ref_msg.pos_ref[1] = 0.0f;
    ctrl_ref_msg.pos_ref[2] = -0.5f;
    ctrl_ref_msg.yaw_ref = 0.0f;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover4 test");

    sleep(5);

    // ctrl_ref_msg.pos_ref[0] = -1.5f;
    // ctrl_ref_msg.pos_ref[1] = 3.0f;
    // ctrl_ref_msg.pos_ref[2] = -1.5f;
    // ctrl_ref_msg.yaw_ref = -1.57f;
    // ctrl_ref_pub.publish(ctrl_ref_msg);
    // ROS_INFO("hover3 test");

    // sleep(5);

    // ctrl_ref_msg.pos_ref[0] = -1.5f;
    // ctrl_ref_msg.pos_ref[1] = -3.0f;
    // ctrl_ref_msg.pos_ref[2] = -1.5f;
    // ctrl_ref_msg.yaw_ref = -1.57f;
    // ctrl_ref_pub.publish(ctrl_ref_msg);
    // ROS_INFO("hover4 test");

    // sleep(5);

    // ctrl_ref_msg.pos_ref[0] = 1.5f;
    // ctrl_ref_msg.pos_ref[1] = -3.0f;
    // ctrl_ref_msg.pos_ref[2] = -1.5f;
    // ctrl_ref_msg.yaw_ref = -1.57f;
    // ctrl_ref_pub.publish(ctrl_ref_msg);
    // ROS_INFO("hover5 test");

    // sleep(5);
    
    // ctrl_ref_msg.pos_ref[0] = 1.5f;
    // ctrl_ref_msg.pos_ref[1] = 3.0f;
    // ctrl_ref_msg.pos_ref[2] = -1.5f;
    // ctrl_ref_msg.yaw_ref = -1.57f;
    // ctrl_ref_pub.publish(ctrl_ref_msg);
    // ROS_INFO("hover5 test");

    // sleep(5);

    // ctrl_ref_msg.pos_ref[0] = 0.0f;
    // ctrl_ref_msg.pos_ref[1] = 3.0f;
    // ctrl_ref_msg.pos_ref[2] = -1.5f;
    // ctrl_ref_msg.yaw_ref = -1.57f;
    // ctrl_ref_pub.publish(ctrl_ref_msg);
    // ROS_INFO("hover5 test");

    // sleep(5);

    // ctrl_ref_msg.pos_ref[0] = 0.0f;
    // ctrl_ref_msg.pos_ref[1] = 0.0f;
    // ctrl_ref_msg.pos_ref[2] = -1.5f;
    // ctrl_ref_msg.yaw_ref = -1.57f;
    // ctrl_ref_pub.publish(ctrl_ref_msg);
    // ROS_INFO("hover5 test");

    // sleep(5);

    set_takeoff_land.request.takeoff = false;
    takeoff_land_srv.call(set_takeoff_land);
    ROS_INFO("land test");

    return 0;
}
