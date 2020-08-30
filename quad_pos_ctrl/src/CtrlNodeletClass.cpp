#include <pluginlib/class_list_macros.h>
#include "CtrlNodeletClass.h"

PLUGINLIB_EXPORT_CLASS(ctrl_namespace::CtrlNodeletClass, nodelet::Nodelet)

namespace ctrl_namespace {
    CtrlNodeletClass::CtrlNodeletClass() {

    }
    CtrlNodeletClass::~CtrlNodeletClass() {
        delete controller_ptr;
    }
    void CtrlNodeletClass::onInit() {
        std::string initial_str = "Initializing ctrl nodelet...";
        NODELET_DEBUG_STREAM(initial_str);
        initial_str += this->getName();
        ROS_INFO_STREAM(initial_str);
        this->getNodeHandle().param<int>("uav_id", id, 1);
        std::cout << "uav_id: " << id << std::endl;
        controller_ptr = new Controller(id);
    }
}