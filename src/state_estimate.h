#ifndef STATE_ESTIMATE_H_
#define STATE_ESTIMATE_H_

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "diff_intigral_cal.h"

class State_Estimate{
    public:

        typedef struct state_struction {
            ros::Time header;
            Eigen::Vector3d Pos;
            Eigen::Vector3d Vel;
            Eigen::Vector3d Acc;
            Eigen::Quaterniond att_q;
        }State_s;

        void mocap_data_cb(const geometry_msgs::PoseStamped& mocap_data) {
            pthread_mutex_lock(&mocap_mutex);
            rigid_ = mocap_data;
            state_.Pos << rigid_.pose.position.x, rigid_.pose.position.y, rigid_.pose.position.z; 
            
            Pos_differ_.update(state_.Pos, rigid_.header.stamp);
            Eigen::Vector3d temp;
            if (Pos_differ_.get_diff(temp)) {
                state_.Vel = temp;
                Vel_differ_.update(state_.Vel, rigid_.header.stamp);
                if (Vel_differ_.get_diff(temp)) {
                    state_.Acc = temp;
                } else {
                    state_.Acc = Eigen::Vector3d::Zero();
                }
            } else {
                state_.Vel = Eigen::Vector3d::Zero();
                state_.Acc = Eigen::Vector3d::Zero();
            }

            state_.att_q.w() = rigid_.pose.orientation.w;
            state_.att_q.x() = rigid_.pose.orientation.x;
            state_.att_q.y() = rigid_.pose.orientation.y;
            state_.att_q.z() = rigid_.pose.orientation.z;
            state_.header = rigid_.header.stamp;

            /* publish the vel acc state */
            geometry_msgs::PoseStamped pose_msg;
            geometry_msgs::Vector3Stamped vel_msg;
            geometry_msgs::Vector3Stamped acc_msg;

            pose_msg = rigid_;

            vel_msg.header = rigid_.header;
            vel_msg.vector.x = state_.Vel(0);
            vel_msg.vector.y = state_.Vel(1);
            vel_msg.vector.z = state_.Vel(2);

            acc_msg.header = rigid_.header;
            acc_msg.vector.x = state_.Acc(0);
            acc_msg.vector.y = state_.Acc(1);
            acc_msg.vector.z = state_.Acc(2);

            pthread_mutex_unlock(&mocap_mutex);
            //std::cout << "POS: " << state_.Pos.transpose() << std::endl; 
            //std::cout << "VEL: " << state_.Vel.transpose() << std::endl; 
            //std::cout << "ACC: " << state_.Acc.transpose() << std::endl; 
            pose_pub_.publish(pose_msg);
            vel_pub_.publish(vel_msg);
            acc_pub_.publish(acc_msg);
        }
        
        State_Estimate(int id) : 
        nh_("~state_estimate"),
        rigidbody_id_(id),
        Pos_differ_(120.0f,60.0f),
        Vel_differ_(60.0f,30.0f) {
            char * base_channel;
            base_channel = new char[sizeof("/mocap_data_rigid")];
            strcpy(base_channel,"/mocap_data_rigid");
            char *child_channel = new char[2];
            sprintf(child_channel,"%d",rigidbody_id_);
            char * topic_channel = strcat(base_channel,child_channel);
            mocap_sub_ = nh_.subscribe(topic_channel,10,&State_Estimate::mocap_data_cb,this);
            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose",10);
            vel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("vel",10);
            acc_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("acc",10);
            pthread_mutex_init(&mocap_mutex, NULL);
            //mocap_sub = nh_->subscribe("/mocap_data_rigid1",10,&State_Estimate::mocap_data_cb,this);
            //Pos_differ_(120.0f);
            //Vel_differ_(120.0f);
        }
        ~State_Estimate() {
            pthread_mutex_destroy(&mocap_mutex);
        }

        State_s get_state() {
            State_s temp_state_;
            pthread_mutex_lock(&mocap_mutex);
            temp_state_ = state_;
            pthread_mutex_unlock(&mocap_mutex);
            return temp_state_; 
        }

        int get_rigidbody_id() { return rigidbody_id_; }

    private:
        ros::NodeHandle nh_;
        int rigidbody_id_;
        geometry_msgs::PoseStamped rigid_;
        Diff_State<Eigen::Vector3d> Pos_differ_;
        Diff_State<Eigen::Vector3d> Vel_differ_;
        State_s state_;
        ros::Subscriber mocap_sub_;
        pthread_mutex_t mocap_mutex;
        ros::Publisher pose_pub_;
        ros::Publisher vel_pub_;
        ros::Publisher acc_pub_;
};

#endif