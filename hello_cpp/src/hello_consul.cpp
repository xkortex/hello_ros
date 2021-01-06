/*
   Hello CPP
 */
#include <iostream>
#include<ros/ros.h>


std::string to_string(ros::Time const & t) {
    return std::to_string(t.toSec());
}

void now() {
    ROS_INFO("# Now: %s", to_string(ros::Time::now()).c_str());

}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"hello_cpp"); // Registering a node in ros master
    ros::NodeHandle nh;
    ros::start();
    ROS_INFO("Welcome to ROS!");

    ros::SingleThreadedSpinner spinner{};
    ros::SteadyTimer steady ;

    double x = 1.0;

    if (x > 0) {
        steady = nh.createSteadyTimer(ros::WallDuration(x), [](const ros::SteadyTimerEvent &event) {
            now();
        });
    }
    ros::spin(spinner);

    return 0;
}
