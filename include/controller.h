#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define USE_LOGGER 1

#include "state_estimate_vio.h"
#include "mavros_interface.h"
//#include <boost/thread.hpp>
#include "PID_ctrl.h"
#include "geometry_math_type.h"

// srv
#include "quad_pos_ctrl/SetArm.h"
#include "quad_pos_ctrl/SetHover.h"
#include "quad_pos_ctrl/SetTakeoffLand.h"
// msg
#include "quad_pos_ctrl/ctrl_ref.h"
#include "geometry_msgs/PointStamped.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_LOGGER
//#include "logger.h"
#include <string>
#include <iostream>
#include <fstream>
#include <time.h>
#endif

#define ONE_G 9.78

void * start_controller_loop_thread(void *args);

class Controller : public State_Estimate_Vio {
    public:

        Controller (int rigid_id):
        State_Estimate_Vio(rigid_id),
        mavros_interface(rigid_id),
        ctrl_core(rigid_id),
        uav_id(rigid_id),
        nh_("~controller") {
            already_running = false;
            pthread_mutex_init(&ctrl_mutex, NULL);
            pthread_mutex_init(&lidar_data_mutex, NULL);
            arm_status.reset();

            arm_srv = nh_.advertiseService("arm_disarm", &Controller::arm_disarm_srv_handle, this);
            hoverpos_srv = nh_.advertiseService("hover_pos", &Controller::hover_pos_srv_handle, this);
            takeoff_land_srv = nh_.advertiseService("takeoff_land", &Controller::takeoff_land_srv_handle, this);

            ctrl_ref_sub = nh_.subscribe("ctrl_ref",10, &Controller::ctrl_ref_cb, this);

            down_ward_lidar_sub = nh_.subscribe("/lidar_pose",10, &Controller::down_ward_lidar_cb, this);

            nh_.param<std::string>("/controller_logger_file_name",  logger_file_name, "/home/lhc/gazebo_simulate_logger/");

            int result = pthread_create( &ctrl_tid, NULL, &start_controller_loop_thread, this);
            if ( result ) throw result;
        }



        ~Controller() {
            pthread_join(ctrl_tid, NULL);
        }

        typedef struct U_res {
            ros::Time header;
            Eigen::Quaterniond q_d;
            double U1;

            void reset() {
                q_d = Eigen::Quaterniond::Identity();
                U1 = 0.0f;
            }

            U_res() {
                header = ros::Time::now();
                reset();
            }
        }U_s;

        typedef struct ctrl_cmd {
            ros::Time header;
            Eigen::Vector3d pos_d;
            Eigen::Vector3d vel_d;
            Eigen::Vector3d acc_d;
            float yaw_d;
            uint8_t cmd_mask; /* |1: position ctrl valied |2: velocity ctrl valied |3: acc ctrl valied |..|8: */
            ctrl_cmd() {
                header = ros::Time::now();
                pos_d = Eigen::Vector3d::Zero();
                vel_d = Eigen::Vector3d::Zero();
                acc_d = Eigen::Vector3d::Zero();
                yaw_d = 0.0f;
                cmd_mask = 0;
            }
        }cmd_s;

        typedef struct arm_status_t {
            ros::Time header;
            bool armed;
            void reset() {
                armed = false;
            }
            arm_status_t() {
                reset();
            }
        }arm_s;

#ifdef USE_LOGGER
        void start_logger(const ros::Time & t, const int &id);
        std::string getTime_string();
#endif
        void arm_disarm_vehicle(const bool & arm);
        void set_hover_pos(const Eigen::Vector3d & pos, const float & yaw);

        void start_controller_loop() {
            if( already_running ) {
                fprintf(stderr, "controller loop already running!\n");
            } else {
                controller_loop();
            }
        }

        bool arm_disarm_srv_handle(quad_pos_ctrl::SetArm::Request& req,
                                    quad_pos_ctrl::SetArm::Response& res);

        bool hover_pos_srv_handle(quad_pos_ctrl::SetHover::Request& req,
                                    quad_pos_ctrl::SetHover::Response& res);

        bool takeoff_land_srv_handle(quad_pos_ctrl::SetTakeoffLand::Request& req,
                                        quad_pos_ctrl::SetTakeoffLand::Response& res);

        void ctrl_ref_cb(const quad_pos_ctrl::ctrl_ref& msg);
        

        void down_ward_lidar_cb(const geometry_msgs::PointStamped& msg);

    private:
        ros::NodeHandle nh_;
        void controller_loop();
        void one_step();
        U_s cal_Rd_thrust(const PID_ctrl<cmd_s,State_s>::res_s &ctrl_res);
        pthread_t ctrl_tid;
        cmd_s status_ref;
        bool already_running;
        arm_s arm_status;
        geometry_msgs::PointStamped downward_lidar_data;
        /* service list */
        ros::ServiceServer arm_srv;
        ros::ServiceServer hoverpos_srv;
        ros::ServiceServer takeoff_land_srv;

        ros::Subscriber ctrl_ref_sub;

        ros::Subscriber down_ward_lidar_sub;

        ros::Time last_ctrol_timestamp;

        Mavros_Interface mavros_interface;
        
        pthread_mutex_t ctrl_mutex;

        pthread_mutex_t lidar_data_mutex;

        int uav_id;

#ifdef USE_LOGGER
        //Logger ctrl_logger;
        std::string logger_file_name;
        std::ofstream ctrl_logger;
#endif
        /* choose the ctrl method */
        PID_ctrl<cmd_s,State_s> ctrl_core;
};

void *
start_controller_loop_thread(void *args) {
    Controller *controller = (Controller *)args;
    controller->start_controller_loop();
    return NULL;
}

#endif
