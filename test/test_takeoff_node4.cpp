
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "quad_pos_ctrl/SetTakeoffLand.h"
#include "quad_pos_ctrl/ctrl_ref.h"
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_takeoff_node4");
    ros::NodeHandle node("~");
    std::string uav_name;
    node.param<std::string>("uav_name", uav_name, "juliett");
    std::string uav_srv_name = "/" + uav_name + "_pos_ctrl_node/controller/takeoff_land";
    std::string uav_ref_topic_name = "/" + uav_name + "_pos_ctrl_node/controller/ctrl_ref";
    std::cout << uav_srv_name << std::endl;
    std::cout << uav_ref_topic_name << std::endl;
    ros::ServiceClient takeoff_land_srv = node.serviceClient<quad_pos_ctrl::SetTakeoffLand>(uav_srv_name);
    ros::Publisher ctrl_ref_pub = node.advertise<quad_pos_ctrl::ctrl_ref>(uav_ref_topic_name,10);

    quad_pos_ctrl::SetTakeoffLand set_takeoff_land;
    set_takeoff_land.request.takeoff = true;
    set_takeoff_land.request.takeoff_altitude = 1.3f;
    takeoff_land_srv.call(set_takeoff_land);
    ROS_INFO("takeoff test");
    sleep(15);

    quad_pos_ctrl::ctrl_ref ctrl_ref_msg;
    ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 0.0f;
    ctrl_ref_msg.pos_ref[1] = 1.0f;
    ctrl_ref_msg.pos_ref[2] = -1.3f;
    ctrl_ref_msg.yaw_ref = 0.0f;
    ctrl_ref_msg.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover1 test");

    sleep(35);

    ctrl_ref_msg.header.stamp = ros::Time::now();
    ctrl_ref_msg.pos_ref[0] = 0.0f;
    ctrl_ref_msg.pos_ref[1] = 0.0f;
    ctrl_ref_msg.pos_ref[2] = -1.3f;
    ctrl_ref_msg.yaw_ref = 0.0f;
    ctrl_ref_msg.ref_mask = quad_pos_ctrl::ctrl_ref::POS_CTRL_VALIED;
    ctrl_ref_pub.publish(ctrl_ref_msg);
    ROS_INFO("hover2 test");

    sleep(10);
    //sleep(5);

    // sleep(5);

    set_takeoff_land.request.takeoff = false;
    takeoff_land_srv.call(set_takeoff_land);
    ROS_INFO("land test");

    return 0;
}
