/*
   Hello CPP
 */
#include <iostream>
#include<ros/ros.h>
#include <sw/redis++/redis++.h>
#include <std_msgs/Header.h>
#include <nlohmann/json.hpp>



using namespace std;
using namespace sw;
namespace json = nlohmann;
using OptionalHeader = redis::Optional<std_msgs::Header>;



int main(int argc, char **argv) {
    //   what():  Cannot use ros::Time::now() before the first NodeHandle has been created or ros::start() has been called.  If this is a standalone app or test that just uses ros::Time and does not communicate over ROS, you may also call ros::Time::init()
//    ros::init(argc, argv, "hello_redis"); // Registering a node in ros master
//  ros::Time::init();
//    ros::NodeHandle nh;
//    ros::NodeHandlePtr nhp = ros::NodeHandlePtr(new ros::NodeHandle);
//    ros::start();
//    cout << ros::this_node::getName() << endl;
//    ROS_INFO("Welcome to ROS!");
    auto rc = redis::Redis("tcp://127.0.0.1:6379");
    redis::OptionalString val = rc.get("foo2");    // val is of type OptionalString. See 'API Reference' section for details.
    if (bool(val)) {
        // Dereference val to get the returned value of std::string type.
        std::cout << typeid(val).name() << ": " << *val << std::endl;
    }   // else key doesn't exist.
    auto valu = val.value();
    cout << valu << endl;
    std_msgs::Header head{};
    head.frame_id = "using_redis";
    auto opt_head = OptionalHeader{head};
    cout << opt_head.value() << val.operator bool();
    json::json j;
    j["pi"] = 3.141;
    j["answer"]["everything"] = 42;
    j["answer"]["nothing"] = "pain";
    auto answer = j["answer"];
    cout << j.dump() << endl;
    cout << answer.dump() << endl;
    cout << j.flatten().dump() << endl;

    redis::ReplyUPtr sn_out = rc.command("CLIENT", "SETNAME", "george");
    redis::ReplyUPtr clients = rc.command("CLIENT", "LIST");
    std::string msg;

    cout << std::string(sn_out->str) << endl << clients->str  << endl;
    return 0;
}
