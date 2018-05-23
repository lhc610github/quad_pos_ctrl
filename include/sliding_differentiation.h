#ifndef SLIDING_DIFFERENTIATION_H_
#define SLIDING_DIFFERENTIATION_H_

#include "ros/ros.h"
#include <Eigen/Core>
#include "math.h"

template<class T>
class Sliding_diff{

    public:

        Sliding_diff(uint8_t order_k): order_diff(order_k) {
            state.resize(order_k);
            state_dot.resize(order_k);
            param.lamada.resize(order_k);
            param.lamada[0] = 6.0f;
            param.lamada[1] = 15.0f;
            param.lamada[2] = 15.0f;
            param.lamada[3] = 10.0f;
            param.eps_temp = 0.00001f;
            param.L = 6.0f;
            param.max_err_tolerate = 0.2f;
            init_flag = false;
        }

        ~Sliding_diff() {}

        void init_diff(T initial_state, ros::Time timeStamp) {
            reset(initial_state, timeStamp);
            init_flag = true;
        }

        bool get_status() {
            return init_flag;
        }

        void reset(T init_data, ros::Time timeStamp) {
            state[0] = init_data;
            for (uint8_t i = 1; i < order_diff; i++) {
                state[i] = T::Zero();
            }

            for (uint8_t i = 0; i < order_diff; i++) {
                state_dot[i] = T::Zero();
            }
            sample_timestamp = timeStamp;
        }

        bool update(T f, ros::Time timeStamp) {
            if (((timeStamp - sample_timestamp) < ros::Duration(0.5)) && init_flag) {
                T e;
                e = state[0] - f;
                T abs_e;
                abs_e = e.array().abs();
                //abs_e = abs_e.array() + param.eps_temp;
                double temp_max_e = abs_e.maxCoeff();
                if (temp_max_e > param.max_err_tolerate) {
                    //std::cout << temp_max_e << std::endl;
                    ROS_INFO("err[%f] too big, canot tolerate! reset ",temp_max_e);
                    reset(f,timeStamp);
                    return false;
                }

                T sign_e;
                sign_e = e.array() / (abs_e.array() + param.eps_temp);
                //std::cout << sign_e << std::endl;
                for (uint8_t i = 0; i < order_diff-1; i++) { 
                        int _k = order_diff - 1;
                        T temp0 = abs_e.array().pow( (double)(_k-i) / (double)(_k + 1) );
                        T temp1 = sign_e.array() * temp0.array();
                        T temp2 = temp1.array() * (powf(param.L, (double)(i+1)/(double)(_k+1) ) * param.lamada[_k - i] * -1.0f);
                        state_dot[i] = temp2.array() + state[i+1].array();
                }
                state_dot[order_diff-1] = sign_e.array() * (param.L * param.lamada[0] * -1.0f);
            } else {
                reset(f,timeStamp);
                return false;
            }
            
            for (uint8_t i = 0; i < order_diff; i++) {
                state[i] = state[i].array() + state_dot[i].array() * (timeStamp-sample_timestamp).toSec();
            }
            sample_timestamp = timeStamp;
            return true;
        }

        bool get_diff(T& diff, uint8_t order) {
            diff = state[order];
            return init_flag;
        }

        typedef struct Sliding_diff_param_s {
            std::vector<double> lamada;
            double L;
            double eps_temp;
            double max_err_tolerate;
        } sd_p_s;

    private:
        bool init_flag;
        uint8_t order_diff;
        //T init_state;
        std::vector<T> state;
        std::vector<T> state_dot;
        ros::Time sample_timestamp;
        sd_p_s param;
};
#endif