#ifndef DJI_INTERFACE_H_
#define DJI_INTERFACE_H_

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

#include "geometry_math_type.h"

/* DJI Header file */
#include <sensor_msgs/Joy.h>
#include <dji_sdk/DroneArmControl.h>
#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class DJI_Interface {
public:
    DJI_Interface(const int &id):
    _nh("~cmd") {
        att_target_pub = _nh.advertise<sensor_msgs::Joy>(
                "/lu_test/dji_osdk_ros/flight_control_setpoint_att_thrust", 10);

        flight_status_sub = _nh.subscribe("/lu_test/flight_status", 10,
                &DJI_Interface::flight_status_cb, this);

        arm_status_sub = _nh.subscribe("/lu_test/display_mode", 10,
                &DJI_Interface::display_mode_cb, this);
        /* no need to set flight mode */

        arm_disarm_client = _nh.serviceClient<dji_sdk::DroneArmControl>(
                "dji_sdk/drone_arm_control");
    }

    ~DJI_Interface() {}

    typedef struct mavros_state_t {
        ros::Time header;
        bool has_armed;
        bool offboard_enabled;
        void reset () {
            has_armed = false;
            offboard_enabled = false;
        }
        mavros_state_t() {
            reset();
        }
    }state_s;

    void flight_status_cb(const std_msgs::Bool & fs_data) {
        std_msgs::Bool temp_data = fs_data;
        _state.header = ros::Time::now();
        _state.has_armed = fs_data.data;
    }

    void display_mode_cb(const std_msgs::Bool & dm_data) {
        std_msgs::Bool temp_data = dm_data;
        _state.header = ros::Time::now();
        _state.offboard_enabled = dm_data.data;
    }

    void get_status(bool & arm_state, bool & offboard_enabled, ros::Time & timestamp) {
        arm_state = _state.has_armed;
        offboard_enabled = _state.offboard_enabled;
        timestamp = _state.header;
    }

    bool set_arm_and_offboard() {
        ros::Rate _ofb_check_rate(0.3);
        int try_arm_ofb_times = 0;
        while (!_state.offboard_enabled || !_state.has_armed ) {
            if(_state.offboard_enabled) {
                ros::Rate _arm_check_rate(0.3);
                while (!_state.has_armed) {
                    dji_sdk::DroneArmControl arm_srv;
                    arm_srv.request.arm = 1;//dji_sdk::DroneArmControl::ARM_COMMAND;
                    if (arm_disarm_client.call(arm_srv)) {
                        ROS_INFO("vehicle ARMED");
                    }
                    try_arm_ofb_times = try_arm_ofb_times + 1;
                    if (try_arm_ofb_times >= 3) {
                        ROS_INFO("try 3 times, cannot armed uav, give up!");
                        return false;
                    }
                    _arm_check_rate.sleep();
                }
            } else {
                ROS_INFO("not in OFFBOARD mode");
                ROS_INFO("PLEASE switch to OFFBOARD mode");
                _ofb_check_rate.sleep();
            }
        }
        return true;
    }

    bool set_disarm() {
        ros::Rate _arm_check_rate(0.3);
        while (_state.has_armed) {
            dji_sdk::DroneArmControl arm_srv;
            arm_srv.request.arm = 0;//dji_sdk::DroneArmControl::DISARM_COMMAND;
            if (!arm_disarm_client.call(arm_srv)) {
                return false;
            }
            ROS_INFO("vehicle DISARMED");
            _arm_check_rate.sleep();
        }
        return true;
    }

    void pub_att_thrust_cmd(const Eigen::Quaterniond &q_d, const double & thrust_d) {
        sensor_msgs::Joy at_cmd;
        at_cmd.header.stamp = ros::Time::now();
        at_cmd.axes.resize(4); // roll pitch yaw thrust
        Eigen::Vector3d att_euler;
        get_euler_from_q(att_euler, q_d);
        at_cmd.axes[0] = att_euler(0);
        at_cmd.axes[1] = -att_euler(1);
        at_cmd.axes[2] = -att_euler(2);
        at_cmd.axes[3] = 100.0f*thrust_d;
        att_target_pub.publish(at_cmd);
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher att_target_pub;
    ros::Subscriber flight_status_sub;
    ros::Subscriber arm_status_sub;
    ros::ServiceClient arm_disarm_client;
    state_s _state;
};

#endif
