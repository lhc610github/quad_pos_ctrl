#ifndef STATE_ESTIMATE_VIO_H_
#define STATE_ESTIMATE_VIO_H_

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "diff_intigral_cal.h"
//#include "sliding_differentiation.h"

class State_Estimate_Vio{
    public:

        typedef struct state_struction {
            ros::Time header;
            Eigen::Vector3d Pos;
            Eigen::Vector3d Vel;
            Eigen::Vector3d Acc;
            Eigen::Quaterniond att_q;
        }State_s;

        void vio_pos_data_cb(const geometry_msgs::PoseStamped& _data) {
            pthread_mutex_lock(&state_mutex);
            geometry_msgs::PoseStamped temp_rigid_ = _data;
            state_.Pos << temp_rigid_.pose.position.x, temp_rigid_.pose.position.y, temp_rigid_.pose.position.z; 
            state_.header = temp_rigid_.header.stamp;
            pthread_mutex_unlock(&state_mutex);
            //std::cout << "POS: " << state_.Pos.transpose() << std::endl; 
            //std::cout << "VEL: " << state_.Vel.transpose() << std::endl; 
            //std::cout << "ACC: " << state_.Acc.transpose() << std::endl; 
            pose_pub_.publish(temp_rigid_);
            //vel_pub_.publish(vel_msg);
            //acc_pub_.publish(acc_msg);
        }

        void vio_vel_data_cb(const geometry_msgs::Vector3Stamped& _data) {
            pthread_mutex_lock(&state_mutex);
            geometry_msgs::Vector3Stamped temp_vel_ = _data;
            state_.Vel << temp_vel_.vector.x, temp_vel_.vector.y, temp_vel_.vector.z; 
            state_.header = temp_vel_.header.stamp;
            pthread_mutex_unlock(&state_mutex);
            //std::cout << "POS: " << state_.Pos.transpose() << std::endl; 
            //std::cout << "VEL: " << state_.Vel.transpose() << std::endl; 
            //std::cout << "ACC: " << state_.Acc.transpose() << std::endl; 
            vel_pub_.publish(temp_vel_);
            //vel_pub_.publish(vel_msg);
            //acc_pub_.publish(acc_msg);
        }
        
        void vio_acc_data_cb(const geometry_msgs::Vector3Stamped& _data) {
            pthread_mutex_lock(&state_mutex);
            geometry_msgs::Vector3Stamped temp_acc_ = _data;
            state_.Acc << temp_acc_.vector.x, temp_acc_.vector.y, temp_acc_.vector.z;
            state_.header = temp_acc_.header.stamp;
            pthread_mutex_unlock(&state_mutex);
            //std::cout << "POS: " << state_.Pos.transpose() << std::endl; 
            //std::cout << "VEL: " << state_.Vel.transpose() << std::endl; 
            //std::cout << "ACC: " << state_.Acc.transpose() << std::endl; 
            acc_pub_.publish(temp_acc_);
            //vel_pub_.publish(vel_msg);
            //acc_pub_.publish(acc_msg);
        }
        
        void vio_att_data_cb(const geometry_msgs::PoseStamped& _data) {
            pthread_mutex_lock(&state_mutex);
            geometry_msgs::PoseStamped temp_att_ = _data;
            state_.att_q.w() = temp_att_.pose.orientation.w;
            state_.att_q.x() = temp_att_.pose.orientation.x;
            state_.att_q.y() = temp_att_.pose.orientation.y;
            state_.att_q.z() = temp_att_.pose.orientation.z;
            state_.header = temp_att_.header.stamp;
            pthread_mutex_unlock(&state_mutex);
        }

        State_Estimate_Vio(int id) : 
        nh_("~state_estimate"),
        rigidbody_id_(id){

            std::string parent_topic_name = "/vio_data_rigid";

            std::string pos_topic_channel = parent_topic_name + std::to_string(id);
            pos_topic_channel += "/pos";

            std::string vel_topic_channel = parent_topic_name + std::to_string(id);
            vel_topic_channel += "/vel";

            std::string acc_topic_channel = parent_topic_name + std::to_string(id);
            acc_topic_channel += "/acc";

            std::string att_topic_channel = parent_topic_name + std::to_string(id);
            att_topic_channel += "/att";

            std::cout << "pos topic name:" << pos_topic_channel << std::endl;
            std::cout << "vel topic name:" << vel_topic_channel << std::endl;
            std::cout << "acc topic name:" << acc_topic_channel << std::endl;
            std::cout << "att topic name:" << att_topic_channel << std::endl;

            pos_sub_ = nh_.subscribe(pos_topic_channel,10,&State_Estimate_Vio::vio_pos_data_cb,this);
            vel_sub_ = nh_.subscribe(vel_topic_channel,10,&State_Estimate_Vio::vio_vel_data_cb,this);
            acc_sub_ = nh_.subscribe(acc_topic_channel,10,&State_Estimate_Vio::vio_acc_data_cb,this);
            att_sub_ = nh_.subscribe(att_topic_channel,10,&State_Estimate_Vio::vio_att_data_cb,this);

            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose",10);
            vel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("vel",10);
            acc_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("acc",10);
            pthread_mutex_init(&state_mutex, NULL);
        }

        ~State_Estimate_Vio() {
            pthread_mutex_destroy(&state_mutex);
        }

        State_s get_state() {
            State_s temp_state_;
            pthread_mutex_lock(&state_mutex);
            temp_state_ = state_;
            pthread_mutex_unlock(&state_mutex);
            return temp_state_; 
        }

        int get_rigidbody_id() { return rigidbody_id_; }

    private:
        ros::NodeHandle nh_;
        int rigidbody_id_;
        //geometry_msgs::PoseStamped rigid_;
        //Diff_State<Eigen::Vector3d> Pos_differ_;
        //Diff_State<Eigen::Vector3d> Vel_differ_;
        //Sliding_diff<Eigen::Vector3d> sliding_differ_;
        State_s state_;
        ros::Subscriber pos_sub_;
        ros::Subscriber vel_sub_;
        ros::Subscriber acc_sub_;
        ros::Subscriber att_sub_;
        pthread_mutex_t state_mutex;
        ros::Publisher pose_pub_;
        ros::Publisher vel_pub_;
        ros::Publisher acc_pub_;
};

#endif