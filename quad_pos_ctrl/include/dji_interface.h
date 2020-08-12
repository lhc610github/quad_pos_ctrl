#ifndef DJI_INTERFACE_H_
#define DJI_INTERFACE_H_

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

/* DJI Header file */

#include <Eigen/Core>
#include <Eigen/Geometry>

class DJI_Interface {
public:
    DJI_Interface(const int &id):
    _nh("~cmd") {

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

private:
    ros::NodeHandle _nh;
    state_s _state;
};

#endif