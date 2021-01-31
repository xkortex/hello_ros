#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <boost/filesystem.hpp>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>


using namespace std;
typedef map<int, sensor_msgs::ImagePtr> imgmap;
typedef map<int, std_msgs::Header> headmap;
typedef map<ros::Time, std_msgs::Header> t_headmap;
typedef vector<sensor_msgs::Image> imglist;

int abs(int x) {
    return x < 0 ? -x : x;
}

std::string to_string(ros::Time const &t) {
    return std::to_string(t.toSec());
}

class AltTimer : public ros::Timer {
    AltTimer() {}

    AltTimer(const AltTimer &rhs);

    ~AltTimer();

    AltTimer createTimer();

    friend ros::Timer; // does nothing

    std::string to_string(ros::Timer &t) {
        std::stringstream ss;
        ss << t;
        return ss.str();
    }
};


std::string to_string(ros::Timer &t) {
    std::stringstream ss;

    ss << "Timer@" << &t << "(s=" << t.hasStarted() << ", p=" << t.hasPending() << ")";
    return ss.str();
}

std::string to_string(ros::SteadyTimer &t) {
    std::stringstream ss;
    ss << "SteadyTimer@" << &t << "(s=" << "X" << ", p=" << t.hasPending() << ")";
    return ss.str();
}

std::string to_string(ros::WallTimer &t) {
    std::stringstream ss;
    ss << "WallTimer@" << &t << "(s=" << "X" << ", p=" << t.hasPending() << ")";
    return ss.str();
}

std::string to_string(AltTimer &t) {
    std::stringstream ss;
    ss << "WallTimer@" << &t << "(s=" << "X" << ", p=" << t.hasPending() << ")";
    return ss.str();
}

const ros::Duration ZERO_DURATION(0);

ros::Duration abs(ros::Duration dur) {
    if (dur < ZERO_DURATION) return -dur;
    return dur;
}

headmap::iterator find_near(headmap im, int x, int tolerance = 1) {
    for (auto it = im.begin(); it != im.end(); ++it) {
        if (abs(x - (it->first)) <= tolerance) {
            return it;
        }
    }
    return im.end();
}


template<typename T>
using RosTimeMap = std::map<ros::Time, T>;

template<typename T>
typename RosTimeMap<T>::const_iterator find_near(const std::map<ros::Time, T> &iterable,
                                                 ros::Time t, ros::Duration tol = ros::Duration(0.1)) {
    for (auto it = iterable.begin(); it != iterable.end(); ++it) {
        auto dt = abs(t - (it->first));
        if (dt <= tol) {
            return it;
        }
    }
    return iterable.end();
}

template<typename T>
void print(const std::map<ros::Time, T> &iterable) {
    for (auto it = iterable.begin(); it != iterable.end(); ++it) {
        cout << "> " << it->first << ": " << it->second << endl;
    }
}


//}

//template<typename T>
//class MessageWithStamp {
//public:
//    MessageWithStamp(ros::Time time);
//};

//
//template<typename T>
//typename std::map<ros::Time, T>::iterator find_near(const std::map<ros::Time, T> &iterable, ros::Time t);



//template<class Container>
//Container:iterator find_near(Container iterable, ros::Time t, ros::Duration tol = ros::Duration(0.1));
//template<class Container>
//Container::iterator find_near(Container iterable, ros::Time t, ros::Duration tol = ros::Duration(0.1));
//
//template<typename T>
//class Finder {
//    std::map<ros::Time, T> find_near(std::map<ros::Time, T> iterable, ros::Time t, ros::Duration tol = ros::Duration(0.1)){
//
//    }
//};

void chatterCallback(const std_msgs::Header::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->frame_id.c_str());
}

void betterCallback(const ros::TimerEvent &event, std::shared_ptr<ros::Timer> const timer) {
    ROS_ERROR("bC inside %s", to_string(*timer).c_str());

}

void make_yolo(ros::NodeHandlePtr const &nhp, double dur) {
    auto yolo2 = std::make_shared<ros::Timer>();
    ROS_INFO("y3 prior  %s", to_string(*yolo2).c_str());
    auto func = boost::bind(betterCallback, _1, yolo2);
    *yolo2 = nhp->createTimer(ros::Duration(dur), func, true, true);

//                              *yolo2 = nhp->createTimer(ros::Duration(dur),
//                              [yolo2](const ros::TimerEvent &event) {
//                                  ROS_ERROR("y3 inside %s", to_string(*yolo2).c_str());
//                                  ROS_ERROR("this should trigger");
//                              },
//                              true, true);
    ROS_WARN("y3 outside %s", to_string(*yolo2).c_str());
}


/// Create an anonymous timer callback. A reference to ros::Timer must exist in order for the callback
/// to trip. If the Timer is destructed, the callback is popped from the event queue
/// NodeHandle is ROS's god class. It handles the event queue, firing callbacks, message bus, etc.
void anonymous_timer(ros::NodeHandlePtr const &nhp, double duration, long int tail) {
    if (!ros::ok()) return;
    auto yolo = std::make_shared<ros::Timer>();
    auto func = [&, yolo, tail](const ros::TimerEvent &event) {
        ROS_INFO("inside callback %s %ld", to_string(*yolo).c_str(), tail);
        string yuge;
        yuge.resize(10000000);
        const string yuck = std::string(yuge);
        if (tail ) {
            anonymous_timer(nhp, 0.01, tail-1);
        }

        /// (yolo) falls out of scope here which should fully destruct the shared pointer
    };
    *yolo = nhp->createTimer(ros::Duration(duration), func, true, true);
    /// (*yolo) falls out of scope, but since we retained a shared_ptr (yolo) to it in the lambda, it should survive
}

void anonymous_timer(ros::NodeHandlePtr const &nhp, double duration) {
    return anonymous_timer(nhp, duration, 0);
}

uint64_t COUNT = 0;
uint64_t *P_COUNT = &COUNT;
uint64_t COUNTS[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

auto ROS_INSTANT = ros::Duration{0,0};
void anonymous_infinite(ros::NodeHandlePtr const &nhp, int i) {
    if (!ros::ok()) return;
    auto yolo = std::make_shared<ros::Timer>();
    auto func = [&, yolo, i](const ros::TimerEvent &event) {
//        ROS_INFO("inside callback %s %ld %p", to_string(*yolo).c_str(), *P_COUNT, P_COUNT);
        cout << yolo << ", ";
        if (COUNT % 5 == 0) {
            cout << endl;
        }
        (COUNTS[i])++;
        (*P_COUNT)++;
        anonymous_infinite(nhp, i);


        /// (yolo) falls out of scope here which should fully destruct the shared pointer
    };
    *yolo = nhp->createTimer(ROS_INSTANT, func, true, true);
    /// (*yolo) falls out of scope, but since we retained a shared_ptr (yolo) to it in the lambda, it should survive
}


/// This is just to contrast the above approach, to show why I can't just capture the timer itself
void anonymous_timer_done_wrong(ros::NodeHandlePtr const &nhp, double duration) {
    ros::Timer yolo;
    ros::Timer *yolo_p;
    auto func = [&](const ros::TimerEvent &event) {
        ROS_INFO("inside callback %s", to_string(yolo).c_str());
    };
    yolo = nhp->createTimer(ros::Duration(duration), func, true, true);
    /// (yolo) falls out of scope, ~ros::Timer(), the callback dies before triggering. This fails.
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hello_cpp"); // Registering a node in ros master
    //   what():  Cannot use ros::Time::now() before the first NodeHandle has been created or ros::start() has been called.  If this is a standalone app or test that just uses ros::Time and does not communicate over ROS, you may also call ros::Time::init()
//  ros::Time::init();
    ros::NodeHandle nh;
//    ros::NodeHandlePtr nhp = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::NodeHandlePtr nhp = boost::make_shared<ros::NodeHandle>(nh);

//    auto sub = nh.subscribe("/topic", 1, );
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    ros::TransportHints();

    headmap im;
    imglist lst;
//    sensor_msgs::Image *p_img;
    string msg = "abc";
    im[3].frame_id = "third";
    im[5].frame_id = "fifth";
    im[8].frame_id = "eighth";
    auto out = im.find(3);
    cout << out->first << ": " << out->second << endl;
//    cout << (im.find(4) == im.end());
    {
        auto it = find_near(im, 6);
//        RosTimeMap<std_msgs::Header>::iterator it = find_near(im, ros::);


        if (it != im.end()) {
            cout << it->second << endl;
        }
    }

    t_headmap mp;
    auto maybe_time = ros::Time(34.55) - ros::Duration(1.0);
    mp[ros::Time(11.23)].frame_id = "11.23";
    mp[ros::Time(33.55)].frame_id = "33.55";
    mp[ros::Time(39.49)].frame_id = "39.49";
//    mp.emplace(ros::Time(99.49), std_msgs::Header{});
    print(mp);
    cout << "---" << endl;
    {
        auto it = find_near(mp, maybe_time, ros::Duration(1.0));
        if (it != mp.end()) {
            cout << "near hit: " << it->first << ": " << it->second << endl;
            auto maybe_evt_pair = mp.find(it->first);
            if (maybe_evt_pair != mp.end()) {

                cout << "contains: " << maybe_evt_pair->first << endl;
            }
//            mp.erase(std::remove(mp.begin(), mp.end(), it), mp.end());
            mp.erase(it->first);
        } else {
            cout << "miss" << endl;
        }
    }
    cout << "---" << endl;
    print(mp);
    cout << "---" << endl;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(1, 6);
    std::uniform_int_distribution<uint32_t> random_uint32(0, 0xffffffff);

    std::cout << distribution(generator) << distribution(generator) << " " << random_uint32(generator) << std::endl;

    std::map<uint32_t, ros::Timer> timer_map;

//    cout << boost::filesystem::exists(boost::filesystem::path("/home/mike"));
    ros::AsyncSpinner spinner{0};

    int count = 0;
    ros::Timer tim = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &event) {
//        cout << "is tim valid2: " << tim.hasStarted() << endl;
        ROS_INFO("tim2 %s %s", to_string(event.current_expected).c_str(), to_string(event.last_expected).c_str());
        cout << "e> is tim started: " << tim.hasStarted() << " valid: " << tim.isValid() << " pending: "
             << tim.hasPending() << endl;

    }, true, true);
    ros::Timer tim2 = nh.createTimer(ros::Duration(0.2), [&](const ros::TimerEvent &event) {
        if (count > 120) {
            ros::shutdown();
        }
//        cout << "is tim valid2: " << tim.hasStarted() << endl;
//        ROS_INFO("tim2 %s %s", to_string(event.current_expected).c_str(), to_string(event.last_expected).c_str() );
        cout << count << "> is tim started: " << tim.hasStarted() << " valid: " << tim.isValid() << " pending: "
             << tim.hasPending() << endl;
        count++;
    }, true, true);
    cout << "o> is tim started: " << tim.hasStarted() << " valid: " << tim.isValid() << " pending: " << tim.hasPending()
         << endl;

    {
        uint32_t yolo_id = random_uint32(generator);
        ROS_INFO("yolo_id %u", yolo_id);
        ros::Timer yolo = nh.createTimer(ros::Duration(0.1), [&, yolo_id](const ros::TimerEvent &event) {
//        cout << "is tim valid2: " << tim.hasStarted() << endl;
            ROS_ERROR("inside %s: %u", to_string(yolo).c_str(), yolo_id);
//        ROS_INFO("tim2 %s %s", to_string(event.current_expected).c_str(), to_string(event.last_expected).c_str());
            cout << "e> is tim started: " << tim.hasStarted() << " valid: " << tim.isValid() << " pending: "
                 << tim.hasPending() << endl;
            timer_map.erase(yolo_id);
            ROS_INFO("erased %u", yolo_id);
        }, true, true);
        timer_map.emplace(yolo_id, yolo);
        ROS_WARN("outside %s", to_string(yolo).c_str());
    }
    {
        auto yolo2 = std::make_shared<ros::Timer>();
        ROS_INFO("y2 prior  %s", to_string(*yolo2).c_str());
        *yolo2 = nh.createTimer(ros::Duration(0.5), [&, yolo2](const ros::TimerEvent &event) {
            ROS_ERROR("y2 inside %s", to_string(*yolo2).c_str());

        }, true, true);
        ROS_WARN("y2 outside %s", to_string(*yolo2).c_str());

    }

    make_yolo(nhp, 0.6);
    anonymous_timer(nhp, 0.7);
    for (int i = 0; i < 16; i++) {
        anonymous_infinite(nhp, i);
    }

    spinner.start();

    ros::Timer shutdown_timer = nh.createTimer(ros::Duration(10), [&](const ros::TimerEvent &event) {
        ROS_INFO("shutting down....");
        ROS_INFO("COUNT %ld %p", *P_COUNT, P_COUNT);
        for( int j = 0; j < 16; j++) {
            cout << COUNTS[j] << "," ;
        }
        cout << "}" << endl;
        ros::shutdown();
    }, true, true);
    ros::waitForShutdown();
    ROS_INFO("fin");

}