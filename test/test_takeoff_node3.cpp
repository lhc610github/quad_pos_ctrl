
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "quad_pos_ctrl/SetTakeoffLand.h"
#include "quad_pos_ctrl/ctrl_ref.h"
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_takeoff_node3");
    ros::NodeHandle node;
    ros::ServiceClient takeoff_land_srv[5];
    takeoff_land_srv[0] = node.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node1/controller/takeoff_land");
    takeoff_land_srv[1] = node.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node2/controller/takeoff_land");
    takeoff_land_srv[2] = node.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node3/controller/takeoff_land");
    takeoff_land_srv[3] = node.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node4/controller/takeoff_land");
    takeoff_land_srv[4] = node.serviceClient<quad_pos_ctrl::SetTakeoffLand>("/quad_pos_ctrl_node5/controller/takeoff_land");
    //ros::Publisher ctrl_ref_pub = node.advertise<quad_pos_ctrl::ctrl_ref>("/quad_pos_ctrl_node3/controller/ctrl_ref",10);

    quad_pos_ctrl::SetTakeoffLand set_takeoff_land;
    set_takeoff_land.request.takeoff = true;
    set_takeoff_land.request.takeoff_altitude = 0.8f;
    ROS_INFO("takeoff test");
    takeoff_land_srv[0].call(set_takeoff_land);
    takeoff_land_srv[1].call(set_takeoff_land);
    takeoff_land_srv[2].call(set_takeoff_land);
    takeoff_land_srv[3].call(set_takeoff_land);
    takeoff_land_srv[4].call(set_takeoff_land);
    sleep(10);
    ROS_INFO("land test");
    set_takeoff_land.request.takeoff = false;
    takeoff_land_srv[0].call(set_takeoff_land);
    takeoff_land_srv[1].call(set_takeoff_land);
    takeoff_land_srv[2].call(set_takeoff_land);
    takeoff_land_srv[3].call(set_takeoff_land);
    takeoff_land_srv[4].call(set_takeoff_land);

    /*ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 0.0f;
    ctrl_ref_msg.pos_ref[1] = 2.0f;
    ctrl_ref_msg.pos_ref[2] = -1.0f;
    ctrl_ref_msg.yaw_ref = -1.57f;
    ctrl_ref_msg.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover1 test");*/

    //sleep(5);


    return 0;
}