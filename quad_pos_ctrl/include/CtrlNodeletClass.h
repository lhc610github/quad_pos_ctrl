#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "state_estimate_vio.h"
#include "controller.h"
#include <iostream>
#include <memory>
namespace ctrl_namespace {
    class CtrlNodeletClass: public nodelet::Nodelet {
    public:
        CtrlNodeletClass();
        ~CtrlNodeletClass();
        virtual void onInit();
    private:
        int id;
        Controller* controller_ptr;
//        std::auto_ptr<Controller> controller_ptr;
    };
}
