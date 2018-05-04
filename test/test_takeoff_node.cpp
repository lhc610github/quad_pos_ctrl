
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "quad_pos_ctrl/SetArm.h"
#include "quad_pos_ctrl/SetHover.h"
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_takeoff_node");
    ros::NodeHandle node;
    ros::ServiceClient arm_srv = node.serviceClient<quad_pos_ctrl::SetArm>("/quad_pos_ctrl_node/controller/arm_disarm");
    ros::ServiceClient hover_srv = node.serviceClient<quad_pos_ctrl::SetHover>("/quad_pos_ctrl_node/controller/hover_pos");

    quad_pos_ctrl::SetHover set_hover;
    set_hover.request.x_ned = -0.4f;
    set_hover.request.y_ned = 1.8f;
    set_hover.request.z_ned = -0.5f;
    set_hover.request.yaw = -1.57f;
    hover_srv.call(set_hover);
    sleep(3);

    quad_pos_ctrl::SetArm set_arm;
    set_arm.request.armed = true;
    arm_srv.call(set_arm);
    sleep(20);

    set_hover.request.x_ned = -0.4f;
    set_hover.request.y_ned = 2.5f;
    set_hover.request.z_ned = -1.0f;
    set_hover.request.yaw = -1.57f;
    hover_srv.call(set_hover);
    sleep(10);

    set_hover.request.x_ned = -0.4f;
    set_hover.request.y_ned = 2.5f;
    set_hover.request.z_ned = 0.0f;
    set_hover.request.yaw = -1.57f;
    hover_srv.call(set_hover);
    sleep(3);

    set_arm.request.armed = false;
    arm_srv.call(set_arm);
    sleep(1);

    return 0;
}