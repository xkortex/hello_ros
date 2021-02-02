#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>


std::string isonow() {
    boost::posix_time::ptime pnow = boost::posix_time::microsec_clock::universal_time();
    return boost::posix_time::to_iso_extended_string(pnow);
}

using namespace std;

int abs(int x) {
    return x < 0 ? -x : x;
}

std::string to_string(ros::Time const &t) {
    return std::to_string(t.toSec());
}

std::string substr(std::string s, int start, int end) {
//    cout << substr("abcdef", 0, -1) << endl;
//    cout << substr("abcdef", 0, 5) << endl;
//    cout << substr("abcdef", 5, 5) << endl;
//    cout << substr("abcdef", -5, -2) << endl;
    size_t startu, lenu;
    int start1, end1, len1;
    if (start < 0) {
        start1 = (int) s.size() + start;
    } else {
        start1 = start;
    }
    if (start1 < 0) {
        startu = 0;
    } else if (start1 > s.size()) {
        startu = s.size();
    } else {
        startu = unsigned(start1);
    }

    if (end < 0) {
        end1 = (int) s.size() + end;
    } else if (end > s.size()) {
        end1 = (int) s.size();
    } else {
        end1 = end;
    }

    len1 = end1 - (int) startu;
    if (len1 < 0) {
        lenu = 0;
    } else {
        lenu = unsigned(len1);
    }
    return s.substr(startu, lenu);
}

std::string abrTime(ros::Time const &t) {
    std::string sec = std::to_string(t.sec);
    std::string nsec = std::to_string(t.nsec);

    return substr(sec, 6, 12) + "." + substr(nsec, 0, 6);

}

std::string to_string(ros::TimerEvent const &t) {
    stringstream ss;
    ss << "Event(curR " << abrTime(t.current_real)<< ", curE " << abrTime(t.current_expected);
    if (!t.last_real.isZero()) {
        ss << ", lastR" << t.last_real.toSec() << ", lastE" << t.last_expected.toSec();
    }
    ss << ")";
    return ss.str();
}

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


const ros::Duration ZERO_DURATION(0);

ros::Duration abs(ros::Duration dur) {
    if (dur < ZERO_DURATION) return -dur;
    return dur;
}

void chatterCallback(const std_msgs::Header::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->frame_id.c_str());
}

void betterCallback(const ros::TimerEvent &event, std::shared_ptr<ros::Timer> const timer) {
    ROS_ERROR("bC inside %s", to_string(*timer).c_str());

}

/// Create an anonymous timer callback. A reference to ros::Timer must exist in order for the callback
/// to trip. If the Timer is destructed, the callback is popped from the event queue
/// NodeHandle is ROS's god class. It handles the event queue, firing callbacks, message bus, etc.
void anonymous_timer(ros::NodeHandlePtr const &nhp, double duration, long int tail) {
    if (!ros::ok()) return;
    auto yolo = std::make_shared<ros::Timer>();
    auto func = [nhp, yolo, duration, tail](const ros::TimerEvent &event) {
        ROS_INFO("inside callback %s %ld", to_string(*yolo).c_str(), tail);
        if (tail) {
            anonymous_timer(nhp, duration, tail - 1);
        }

        /// (yolo) falls out of scope here which should fully destruct the shared pointer
    };
    *yolo = nhp->createTimer(ros::Duration(duration), func, true, true);
    /// (*yolo) falls out of scope, but since we retained a shared_ptr (yolo) to it in the lambda, it should survive
}

void anonymous_timer(ros::NodeHandlePtr const &nhp, double duration) {
    return anonymous_timer(nhp, duration, 0);
}

void cb_rate(std_msgs::Float64 msg) {
    ROS_INFO("& %s Got double: %lf", isonow().c_str(), msg.data);
}


void print_now() {
    ROS_INFO("# %s", isonow().c_str());
}

void cb_now(const ros::SteadyTimerEvent &e) {
    ROS_INFO("! %s", isonow().c_str());
}


void cb_now_w(const ros::WallTimerEvent &e) {
    ROS_INFO("! %s", isonow().c_str());
}

class TimerTimer {
public:
    TimerTimer() : last{ros::WallTime::now()} {}

    ros::WallDuration dt;
    ros::WallTime last;
    ros::WallTime last_st;
    ros::WallTime last_wt;

    void cb_st(const ros::SteadyTimerEvent &e) {
        auto now = ros::WallTime::now();
        ROS_INFO("! Stdy %s %lf", isonow().c_str(), (now - last_st).toSec());
        last_st = now;
    }

    void cb_wt(const ros::WallTimerEvent &e) {
        auto now = ros::WallTime::now();
        ROS_INFO("! Wall %s %lf", isonow().c_str(), (now - last_wt).toSec());
        last_wt = now;
    }
};

template<typename TimerType, typename DurType>
class PeriodSet {
public:
    PeriodSet(TimerType *timer) : timer{timer} {}

    void cb_set_period(std_msgs::Float64 msg) {
        ROS_INFO("& %s Got double: %lf, set period", isonow().c_str(), msg.data);
        timer->setPeriod(DurType(msg.data));
//        wt->setPeriod(ros::WallDuration(msg.data));
    }

//    ros::SteadyTimer *st;
//    ros::WallTimer *wt;
    TimerType *timer;
};

class OneShotManager {
public:
    OneShotManager() {}

    void erase(boost::uuids::uuid i) {
        timer_map.erase(i);
        std::cout << "erasing: " << i << " sz: " << timer_map.size() <<std::endl;
    }

    boost::uuids::uuid addOneShot(ros::NodeHandlePtr nhp, const ros::Duration &period, const ros::TimerCallback& callback) {
        boost::uuids::uuid i = boost::uuids::random_generator()();

        ros::TimerCallback cb2 = [i, this, callback](const ros::TimerEvent &e) {
            callback(e);
            erase(i);
        };
        ros::Timer tmp = nhp->createTimer(period, cb2, true, false);
        timer_map.emplace(i, tmp);
        tmp.start(); // safety here, need to ensure it's in the map before it pops
        return i;
    }


private:
    std::map<boost::uuids::uuid, ros::Timer> timer_map;

};

using RosPeriodSet = PeriodSet<ros::Timer, ros::Duration>;

void cb_print(const ros::TimerEvent &e) {
    ROS_INFO("fired callback %s", isonow().c_str());

}

class AdjustableTimer {
public:
    AdjustableTimer(ros::NodeHandlePtr nhp, ros::Duration period) : nhp{nhp}, period_{period}, callback{cb_print} {
        p_timer = std::make_shared<ros::Timer>();
    }

    void start() {
        callTick(ros::TimerEvent{});
//        anonymous_timer(nhp, 1, 5);
    }


    void setRate(double rate) {
        period_ = ros::Rate(rate).cycleTime();
    }
    void setPeriod(const ros::Duration &period) {
        period_ = period;
    }

    void cb_setPeriod(std_msgs::Float64 msg) {
        ROS_INFO("& %s Got double: %lf, set period", isonow().c_str(), msg.data);
        setPeriod(ros::Duration(msg.data));
    }

    void call(const ros::TimerEvent &e) {
        auto now = ros::Time::now();
        ROS_INFO("? AdjT %s RealDT( %lf ) %u", isonow().c_str(), (now - last).toSec(), 0);
        callback(e);
        last = now;
    }



    ros::Time calcNext(ros::Time const &last_event, ros::Time const &current_real) {
        ros::Duration tolerance{0.01};
        auto remaining = last_event + period_;
        return last_event;
    }

    void setCallback(const ros::TimerCallback& callback_) {
        callback = callback_;
    }



    void callTick(const ros::TimerEvent &e) {
        std::lock_guard<std::mutex> guard(mutex);
        // "now" is actually event.current_real
        /// we need an anonymous oneshot timer because otherwise it gets reset
        auto yolo = std::make_shared<ros::Timer>();

//        ROS_INFO("%d AdjT %s dt: %lf period: %3.2lf Evt: %s",i++, isonow().c_str(), (e.last_real - last).toSec(), period_.toSec(), to_string(e).c_str());
        last_real = e.current_real;
        nextExpected = e.current_real + period_;
        auto dur = nextExpected - ros::Time::now();
//        ROS_INFO("now: %lf last: %lf next: %lf dur: %lf", e.last_real.toSec(), last.toSec(), nextExpected.toSec(), dur.toSec());
        call(e);
//        *p_timer = nhp->createTimer(dur, &AdjustableTimer::callTick, this);
        *yolo = nhp->createTimer(ros::Duration(dur), [yolo, this, dur](const ros::TimerEvent &event) {
//            ROS_INFO("Enqueued next, dur: %lf", dur.toSec());
            callTick(event);
        }, true, true);
    }
    void callTock(const ros::TimerEvent &e) {
        auto now = ros::Time::now();
    }



private:
    int i{0};
    std::mutex mutex;
    ros::NodeHandlePtr nhp;
    std::shared_ptr<ros::Timer> p_timer;
    ros::Duration period_{1.0};
    ros::Time last{ros::Time::now()};
    ros::Time last_real{ros::Time::now()};
    ros::Time nextExpected{ros::Time::now()};
    ros::TimerCallback callback;
};


class AsyncTriggerTimer {
public:
    AsyncTriggerTimer(ros::NodeHandlePtr nhp, ros::Duration period)
    : nhp{nhp}, period_{period}, callback{cb_print} {}

    void start() {
        callTick(ros::TimerEvent{});
    }

    void setRate(double rate) {
        period_ = ros::Duration(ros::Rate(rate));
    }

    void setPeriod(const ros::Duration &period) {
        period_ = period;
    }

    void cb_setPeriod(std_msgs::Float64 msg) {
        ROS_INFO("& %s Got double: %lf, set period", isonow().c_str(), msg.data);
        setPeriod(ros::Duration(msg.data));
    }
    void cb_setRate(std_msgs::Float64 msg) {
        ROS_INFO("& %s Got double: %lf, set rate", isonow().c_str(), msg.data);
        setRate(msg.data);
    }

    void call(const ros::TimerEvent &e) {
        auto now = ros::Time::now();
        ROS_INFO("& AdjT %s RealDT( %lf ) %u", isonow().c_str(), (now - last).toSec(),0);
        callback(e);
        last = now;
    }


    void setCallback(const ros::TimerCallback& callback_) {
        callback = callback_;
    }



    void callTick(const ros::TimerEvent &e) {
        std::lock_guard<std::mutex> guard(mutex);
        ROS_INFO("callTick()");
        // "now" is actually event.current_real

        last_real = e.current_real;
        nextExpected = e.current_real + period_;
        auto dur = nextExpected - ros::Time::now();
        ROS_INFO("binding ATT");
//        ros::TimerCallback cb = boost::bind(&AsyncTriggerTimer::callTick, this, _1);
        ros::TimerCallback cb = [this](const ros::TimerEvent &e) {
            callTick(e);
        };
        osm.addOneShot(nhp, period_, cb);
        call(e);
    }
    OneShotManager osm;


private:
    std::mutex mutex;
    ros::NodeHandlePtr nhp;
    ros::Duration period_{1.0};
    ros::Time last{ros::Time::now()};
    ros::Time last_real{ros::Time::now()};
    ros::Time nextExpected{ros::Time::now()};
    ros::TimerCallback callback;
};

void cb_request_shutdown(std_msgs::Int8 const &msg) {
    ROS_INFO("Requesting shutdown %d", msg.data);
    ros::requestShutdown();
}

void oops() {
    ROS_WARN("oops!");
}


class EventWatchdog {
public:
    EventWatchdog() = default;
    void addEvent(ros::NodeHandlePtr const &nhp, ros::Time const &t) {
        ros::TimerCallback cb = [t](ros::TimerEvent const &inner_event) {
            oops();
        };
        auto uuid = osm.addOneShot(nhp, ros::Duration(1.0), cb);
        event2id.emplace(t, uuid);
    }

    void clearEvent(ros::Time const &t) {
        ROS_INFO("Cleared %s", to_string(t).c_str());
        auto uuid = event2id[t];
        osm.erase(uuid);
    }

    std::map<ros::Time, boost::uuids::uuid> event2id;
    OneShotManager osm;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "hello_cpp"); // Registering a node in ros master
    //   what():  Cannot use ros::Time::now() before the first NodeHandle has been created or ros::start() has been called.  If this is a standalone app or test that just uses ros::Time and does not communicate over ROS, you may also call ros::Time::init()
//  ros::Time::init();
    ros::NodeHandle nh;
    ros::NodeHandlePtr nhp = boost::make_shared<ros::NodeHandle>(nh);
    ros::AsyncSpinner spinner{0};
    spinner.start();

//    auto sub = nh.subscribe("/topic", 1, );
    print_now();
    TimerTimer tt;
    AsyncTriggerTimer adt{nhp, ros::Duration(1.0)};

//    auto steady_timer = nhp->createSteadyTimer(ros::WallDuration(1.0), &TimerTimer::cb_st, &tt);
//    auto wall_timer = nhp->createWallTimer(ros::WallDuration(1.0), &TimerTimer::cb_wt, &tt);
//    auto ros_timer = nhp->createTimer(ros::Duration(1.0), &AdjustableTimer::call, &adt);
//    auto set_period = boost::bind(PeriodSet::cb_set_period, _1, &steady_timer, &wall_timer);
//    ros::Subscriber sub = nh.subscribe("/rate", 1000, cb_rate);


//    RosPeriodSet ps{&ros_timer};
//    ros::Subscriber sub = nh.subscribe("/period", 1000, &PeriodSet<ros::SteadyTimer>::cb_set_period, &ps);

    adt.setCallback([](const ros::TimerEvent &e) {
//        ROS_INFO("new callback %s", to_string(e).c_str());
    });
    adt.start();
    ros::Subscriber sub_rost = nh.subscribe("/period", 1000, &AsyncTriggerTimer::cb_setPeriod, &adt);
    ros::Subscriber sub_rosr = nh.subscribe("/rate", 1000, &AsyncTriggerTimer::cb_setRate, &adt);
    ros::Subscriber sub_shut = nh.subscribe("/shutdown", 10, cb_request_shutdown);
//    anonymous_timer(nhp, 0.5, 5);

    auto whatcb = [](const ros::TimerEvent &event){};
    cerr << typeid(whatcb).name() << endl;
    cerr << ros::Duration(ros::Rate(1.5)) << " | " << to_string(ros::TimerEvent{}) << endl;
    ros::TimerEvent event;



    sensor_msgs::Image myImg;
    myImg.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::ImagePtr p_img = boost::make_shared<sensor_msgs::Image>(myImg);
    sensor_msgs::ImageConstPtr pc_myImg = p_img;
    auto out = cv_bridge::toCvShare(pc_myImg, sensor_msgs::image_encodings::BGR8);
    // cv_bridge::CvImageConstPtr
    std::cout << out << std::endl;





    boost::filesystem::path ouch("/mount/test/ouch");
    try {

        boost::filesystem::create_directories(ouch);
    } catch (boost::filesystem::filesystem_error &e) {
        ROS_ERROR("[%d] %s", e.code().value(), e.what());
    }


    ros::Timer shutdown_timer = nh.createTimer(ros::Duration(6), [&](const ros::TimerEvent &event) {
        ROS_INFO("shutting down....");
        ros::shutdown();
    }, true, true);
    ros::waitForShutdown();
    ROS_INFO("fin");

}