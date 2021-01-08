//
// Created by mike on 07/01/2021.
//

#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "MyNodeletClass.h"

using namespace std;

namespace hello_nodelet {

    MyNodeletClass::~MyNodeletClass() {
        ros::NodeHandlePtr
        cout << "He's dead, Jim" << endl;
    }
    MyNodeletClass::MyNodeletClass() {
        cout << "Imma nodelet" << endl;
    }
    void MyNodeletClass::onInit() {
        NODELET_WARN("initializing nodelet");
    }
    void foo() {

    }
} // namespace hello_nodelet

PLUGINLIB_EXPORT_CLASS(hello_nodelet::MyNodeletClass, nodelet::Nodelet);