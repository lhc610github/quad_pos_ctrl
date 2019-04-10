#include "controller.h"


void Controller::controller_loop() {
    already_running = true;
    ros::Rate ctrl_rate(100);
    ROS_INFO("controller run at 100Hz");
    status_ref.header = ros::Time::now();
    status_ref.pos_d << 10.0f, 0.0f, -0.5f;
    status_ref.cmd_mask = P_C_V;
    while (ros::ok()) {
        if (ros::Time::now() - get_state().header < ros::Duration(0.5f)) {
            one_step(); 
        } else {
            ROS_INFO_THROTTLE(1.0,"State Timeout");
            ctrl_core.reset();
        }
        ctrl_rate.sleep();
        //usleep(10000);
    }
}

void Controller::one_step() {
        /*check arm status*/
        bool arm_state = false;
        bool ofb_enable = false;
        ros::Time arm_state_timestamp;
        mavros_interface.get_status(arm_state,ofb_enable,arm_state_timestamp);

        pthread_mutex_lock(&ctrl_mutex);
        arm_status.header = ros::Time::now();
        arm_status.armed = (arm_state && ofb_enable);
        pthread_mutex_unlock(&ctrl_mutex);

        pthread_mutex_lock(&ctrl_mutex);
        ctrl_core.set_ref(status_ref);
        bool has_armed = arm_status.armed;
        pthread_mutex_unlock(&ctrl_mutex);
        if (has_armed) {
            PID_ctrl<cmd_s,State_s>::res_s ctrl_res;
            ctrl_core.run(get_state(), ctrl_res);
            //std::cout << ctrl_res.res.transpose() << std::endl;
            U_s U = cal_Rd_thrust(ctrl_res);
            if ((ros::Time::now() - last_ctrol_timestamp) > ros::Duration(0.019)) {
                mavros_interface.pub_att_thrust_cmd(U.q_d,U.U1);
                last_ctrol_timestamp = ros::Time::now();
            }

#ifdef USE_LOGGER
            if (ctrl_logger.is_open()) {
                ctrl_logger << ctrl_res.header.toNSec() << ',';
                ctrl_logger << ctrl_res.res(0) << ',';
                ctrl_logger << ctrl_res.res(1) << ',';
                ctrl_logger << ctrl_res.res(2) << ',';
                ctrl_logger << U.U1 << ',';
                ctrl_logger << U.q_d.w() << ',';
                ctrl_logger << U.q_d.x() << ',';
                ctrl_logger << U.q_d.y() << ',';
                ctrl_logger << U.q_d.z() << std::endl;
            }
#endif
        } else {
            ctrl_core.reset();
            mavros_interface.pub_att_thrust_cmd(Eigen::Quaterniond::Identity(),0.0f);
        }
}

Controller::U_s Controller::cal_Rd_thrust(const PID_ctrl<cmd_s,State_s>::res_s &ctrl_res) {
    U_s res;  
    res.header = ctrl_res.header;
    float _yaw_d = status_ref.yaw_d;
    if (ros::Time::now() - ctrl_res.header < ros::Duration(0.5f)) {
        /* get R */
        Eigen::Matrix3d _R;
        get_dcm_from_q(_R, get_state().att_q); 
        /* get U1 */
        double real_U1 = - ctrl_res.res.transpose() * _R.col(2);
        res.U1 = real_U1 / ONE_G * ctrl_core.get_hover_thrust();
        
        /* get body_z */
        Eigen::Vector3d _body_z;
        if (ctrl_res.res.norm() > 0.001f) {
            _body_z = - ctrl_res.res.normalized();
        } else {
            _body_z << 0.0f , 0.0f , 1.0f;
        }

        /* get y_C */
        Eigen::Vector3d _y_C(-sin(_yaw_d), cos(_yaw_d), 0.0f);

        /* get body_x */
        Eigen::Vector3d _body_x;
        if ( fabsf(_body_z(2)) > 0.0001f ) {
            _body_x = _y_C.cross(_body_z);
            if (_body_z(2) < 0) {
                _body_x = - _body_x;
            }
            _body_x = _body_x.normalized();
        } else {
            _body_x << 0.0f , 0.0f , 1.0f;
        }

        /* get body_y */
        Eigen::Vector3d _body_y = _body_z.cross(_body_x);
        _body_y = _body_y.normalized();

        /* get R_d */
        Eigen::Matrix3d _R_d;
        _R_d.col(0) = _body_x;
        _R_d.col(1) = _body_y;
        _R_d.col(2) = _body_z;

        /* get q_d */
        get_q_from_dcm(res.q_d, _R_d);

    } else {
        res.reset();
    }
    return res; 
}

void Controller::arm_disarm_vehicle(const bool & arm) {
    if (arm) {
        ROS_INFO("vehicle will be armed!");
#ifdef USE_LOGGER
            start_logger(ros::Time::now(),uav_id);
#endif
        if (mavros_interface.set_arm_and_offboard()) {
            ROS_INFO("done!");
        }
    } else {
        ROS_INFO("vehicle will be disarmed!");
        if (mavros_interface.set_disarm()) {
            ROS_INFO("done!");
        }
#ifdef USE_LOGGER
        if (ctrl_logger.is_open()) {
            ctrl_logger.close();
        }
#endif
    }
}

void Controller::set_hover_pos(const Eigen::Vector3d & pos, const float & yaw) {
    pthread_mutex_lock(&ctrl_mutex);
    status_ref.header = ros::Time::now();
    status_ref.pos_d = pos;
    status_ref.yaw_d = yaw;
    status_ref.cmd_mask = P_C_V;
    pthread_mutex_unlock(&ctrl_mutex);
}

bool Controller::arm_disarm_srv_handle(quad_pos_ctrl::SetArm::Request& req,
                                    quad_pos_ctrl::SetArm::Response& res){
    bool arm_req = req.armed;
    arm_disarm_vehicle(arm_req);
    res.res = true;
    return true;
}

bool Controller::hover_pos_srv_handle(quad_pos_ctrl::SetHover::Request& req,
                                    quad_pos_ctrl::SetHover::Response& res){
    Eigen::Vector3d pos_d;
    pos_d << req.x_ned, req.y_ned, req.z_ned;
    float yaw_d = req.yaw;
    set_hover_pos(pos_d,yaw_d);
    res.res = true;
    return true;
}
bool Controller::takeoff_land_srv_handle(quad_pos_ctrl::SetTakeoffLand::Request& req,
                                        quad_pos_ctrl::SetTakeoffLand::Response& res) {
    Eigen::Vector3d pos_d; 
    State_s state_now = get_state();
    Eigen::Vector3d euler;
    get_euler_from_q(euler, state_now.att_q);
    float yaw_d = euler(2);
    if (req.takeoff) {
        pos_d << state_now.Pos(0), state_now.Pos(1), state_now.Pos(2) + 0.2f;
        set_hover_pos(pos_d,yaw_d);
        std::cout << "takeoff process start" << std::endl;
        arm_disarm_vehicle(true); // Arm uav
        sleep(1);
        // float Pos_d_z = -req.takeoff_altitude;
        float Pos_d_z = state_now.Pos(2);
        float takeoff_vel = 0.8f;
        float takeoff_ddz = takeoff_vel / 20.0f;
        ros::Rate takeoff_loop(20);
        std::cout << "takeoff altitude: " << req.takeoff_altitude << " m" << std::endl;
        std::cout << "takeoff velocity: " << takeoff_vel << " m/s"<< std::endl;
        ros::Time start_takeoff_task_time = ros::Time::now();
        while(ros::ok() && ros::Time::now() - start_takeoff_task_time < ros::Duration(8.0)) {
            pos_d(2) = Pos_d_z;
            set_hover_pos(pos_d,yaw_d);
            Pos_d_z -= takeoff_ddz;
            if (Pos_d_z < -req.takeoff_altitude) {
                std::cout << "takeoff process done" << std::endl;
                break;
            }
            takeoff_loop.sleep();
        }
    } else {
        //pos_d << state_now.Pos(0), state_now.Pos(1), 0.0f;
        pos_d << state_now.Pos(0), state_now.Pos(1), state_now.Pos(2);// + 0.5f;
        ros::Time start_land_task_time = ros::Time::now();
        ros::Rate land_loop(20.0);
        float temp_Pos_d_z = state_now.Pos(2);
        while(ros::ok() && ros::Time::now()-start_land_task_time < ros::Duration(8.0)) {
            temp_Pos_d_z += 0.3f/20.0f;
            pos_d(2) = temp_Pos_d_z;
            set_hover_pos(pos_d,yaw_d);
            state_now = get_state();
            pthread_mutex_lock(&lidar_data_mutex);
            float temp_lidar_z = downward_lidar_data.point.z;
            pthread_mutex_unlock(&lidar_data_mutex);
            if (temp_lidar_z < 0.25f && fabs(state_now.Vel(2)) < 1.0f) {
                ROS_INFO("detect land: disarm");
                // sleep(1);
                arm_disarm_vehicle(false); // Arm uav
                break;
            }
            land_loop.sleep();
        }
    }
    res.res = true;
    return true;
}

void Controller::ctrl_ref_cb(const quad_pos_ctrl::ctrl_ref& msg) {
    pthread_mutex_lock(&ctrl_mutex);
    status_ref.header = msg.header.stamp;
    status_ref.pos_d << msg.pos_ref[0], msg.pos_ref[1], msg.pos_ref[2];
    status_ref.vel_d << msg.vel_ref[0], msg.vel_ref[1], msg.vel_ref[2];
    status_ref.acc_d << msg.acc_ref[0], msg.acc_ref[1], msg.acc_ref[2];
    status_ref.yaw_d = msg.yaw_ref;
    status_ref.cmd_mask = msg.ref_mask;
    pthread_mutex_unlock(&ctrl_mutex);
}

void Controller::down_ward_lidar_cb(const geometry_msgs::PointStamped& msg) {
    pthread_mutex_lock(&lidar_data_mutex);
    downward_lidar_data = msg;
    pthread_mutex_unlock(&lidar_data_mutex);
}

#ifdef USE_LOGGER
void Controller::start_logger(const ros::Time & t, const int &id) {
    // std::string logger_file_name("/home/lhc/gazebo_simulate_logger/");
    std::string temp_file_name = logger_file_name + "UAV_";
    temp_file_name += std::to_string(id);
    temp_file_name += "/ctrl_logger";
    /*char data[20];
    sprintf(data, "%lu", t.toNSec());
    logger_file_name += data;*/
    temp_file_name += getTime_string();
    temp_file_name += ".csv";
    std::cout << "controller logger: "<< temp_file_name << std::endl;
    if (ctrl_logger.is_open()) {
        ctrl_logger.close();
    }
    ctrl_logger.open(temp_file_name.c_str(), std::ios::out);
    if (!ctrl_logger.is_open()) {
        std::cout << "cannot open the logger." << std::endl;
    } else {
        ctrl_logger << "timestamp" << ',';
        ctrl_logger << "ctrl_output_x(ned)" << ',';
        ctrl_logger << "ctrl_output_y(ned)" << ',';
        ctrl_logger << "ctrl_output_z(ned)" << ',';
        ctrl_logger << "U1" << ',';
        ctrl_logger << "q_d_w" << ',';
        ctrl_logger << "q_d_x" << ',';
        ctrl_logger << "q_d_y" << ',';
        ctrl_logger << "q_d_z" << std::endl;
    }
}

std::string Controller::getTime_string() {
    time_t timep;
    timep = time(0);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S",localtime(&timep));
    return tmp;
}
#endif

