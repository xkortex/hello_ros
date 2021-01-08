//
// Created by mike on 07/01/2021.
//
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include "MyNodeletClass.h"

using namespace std;
int main(int argc, char **argv) {
    ros::init(argc, argv, "hello_nodelet_node");
    cout << "Hello Nodelet!" << endl;
    nodelet::Loader manager(true);
    nodelet::M_string remappings;
    nodelet::V_string my_argv;
    manager.load(ros::this_node::getName(), "hello_nodelet/MyNodeletClass", remappings, my_argv);
    ros::spin();
}
