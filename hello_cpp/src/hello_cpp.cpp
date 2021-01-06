/*
   Hello CPP
 */
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include<ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>


using namespace std;

bool is_simple_frac(double d);

ros::Duration calculate_edge(ros::Duration const &granularity, ros::Duration const &thresh);

ros::Duration calculate_edge(ros::Duration const &granularity);

std::string to_string(ros::Time const &);

std::string to_string(ros::Duration const &);

std::string to_string(ros::Rate const &);

int64_t to_nano64(ros::Duration const &);

typedef enum {
    eTypeUnknown = 0,
    eTypeString,
    eTypeFloat32,
    eTypeUint32,
    eTypeInt64,
} tPvDatatype;

static char *typeNames[] = {"Unknown", "String", "Float32", "Uint32", "Int64"};

class ImageHandler {
public:
    void callback(const sensor_msgs::ImageConstPtr& msg);
};

void ImageHandler::callback(const sensor_msgs::ImageConstPtr &msg) {
    cout << "---\n" << *msg << "---" << endl;
}

class CamAttr {
public:
    CamAttr(tPvDatatype);

    std::string name = "Dummy";
    std::string dtypeName = "Dummy";

    bool set(string const &);

    bool get(int64_t &);

    bool get(string &);

    bool getStr(string &);

protected:

    tPvDatatype dtype;
private:
    bool (CamAttr::*setAttr)(string const &);

    bool (CamAttr::*getAttrVal)(string &);

    bool Nop_setAttr(string const &);

    bool Uint32_setAttr(string const &);

    bool Float32_setAttr(string const &);

    bool Int64_setAttr(string const &);

    bool String_setAttr(string const &);

    bool String_getAttr(string &);

};

bool CamAttr::set(string const &s) {
    return (this->*setAttr)(s);
}

bool CamAttr::get(string &s) {
    return (this->*getAttrVal)(s);
}

bool CamAttr::Nop_setAttr(string const &value) {
    cout << "D'oh! " << value << endl;
    return false;
}

bool CamAttr::Uint32_setAttr(string const &value) {
    cout << "called Uint32 setAttr: " << value << endl;
    return true;
}

bool CamAttr::Float32_setAttr(string const &value) {
    cout << "called Float32 setAttr: " << value << endl;
    return true;
}

bool CamAttr::Int64_setAttr(string const &value) {
    cout << "called Int64 setAttr: " << value << endl;
    return true;
}

bool CamAttr::String_setAttr(string const &value) {
    cout << "called String setAttr: " << value << endl;
    return true;
}

CamAttr::CamAttr(tPvDatatype dataType) : dtype{dataType} {
    dtypeName = string(typeNames[dtype]);
    cout << "new attr: " << name << " type: " << dtypeName << endl;
    switch (dtype) {
        case eTypeString:
            setAttr = &CamAttr::String_setAttr;
            break;
        case eTypeFloat32:
            setAttr = &CamAttr::Float32_setAttr;
            break;
        case eTypeUint32:
            setAttr = &CamAttr::Uint32_setAttr;
            break;
        case eTypeInt64:
            setAttr = &CamAttr::Int64_setAttr;
            break;
        default:
            setAttr = &CamAttr::Nop_setAttr;
    }
}

template<typename T>
std::ostream &operator<<(std::ostream &os, const CamAttr &obj) {
    os << obj.name;
    return os;
}

class ScheduledEvent {
public:
    ScheduledEvent() {}
    ros::SteadyTimer timer;
};

class EventCache {
public:
    EventCache() : tol(0.0555), delay(0.55) {}

    EventCache(ros::Duration);

    EventCache(ros::Duration, ros::Duration);

    ros::Duration tol;
    ros::Duration delay;
};

EventCache::EventCache(ros::Duration delay) : tol(0.001), delay{delay} {}

EventCache::EventCache(ros::Duration delay, ros::Duration tol) : tol{tol}, delay{delay} {}

bool is_simple_frac(double d) {
    const double ratio = 1.0 / 120; // ratio considered for "simple fractions"
    auto m = fmod(d, ratio);
    return m < 1e-6;
}

std::string to_string(ros::Time const &t) {
    return std::to_string(t.toSec());
}

std::string to_string(ros::Duration const &t) {
    return std::to_string(t.toSec());
}

std::string to_string(ros::Rate const &t) {
    return std::to_string(ros::Duration(t).toSec());
}

ros::Duration calculate_edge(ros::Duration const &granularity) {
    return calculate_edge(granularity, ros::Duration{0, 0});
}

ros::Duration calculate_edge(ros::Duration const &granularity, ros::Duration const &thresh) {
    auto now = ros::Time::now();
    auto nowns = now.toNSec();
    auto dt = granularity.toNSec();
    auto m = nowns % dt;
    auto delay_ns = dt - m;
//  auto next =
    ros::Duration out = ros::Duration().fromNSec(delay_ns);
    ROS_INFO("Now: %s Next: %s", to_string(now).c_str(), std::to_string(nowns + delay_ns).c_str());
    ROS_INFO("dt: %s", to_string(granularity).c_str());
    ROS_INFO("delay_ns: %ld m: %ld dt %ld", delay_ns, m, dt);
//    ROS_INFO("delaydur: %s", to_string(out).c_str());
    if (out < thresh) return ros::Duration{0, 0};
    return out;

}

ros::Duration since_edge(ros::Duration const &granularity) {
    auto now = ros::Time::now();
    auto nowns = now.toNSec();
    auto dt = granularity.toNSec();
    auto div = nowns / dt;
    auto last_edge_ns = div * dt;
    auto since = ros::Duration().fromNSec(nowns - last_edge_ns);
//    ROS_INFO("Now: %s", to_string(now).c_str());
//    ROS_INFO("Last edge: %ld", last_edge_ns);
    ROS_INFO("Since: %s", to_string(since).c_str());
    return since;
}

void now() {
    ROS_INFO("# Now: %s", to_string(ros::Time::now()).c_str());

}

void callback(const ros::SteadyTimerEvent &event) {
    ROS_WARN("steady callback");
    now();
}

void te_callback(const ros::TimerEvent &event) {
    ROS_WARN("plain timer callback");
    now();
}


std::string hexStr(char *data, int len) {
    std::stringstream ss;
    ss << std::hex;
    for (int i(0); i < len; i++) {
        ss << std::setw(2) << std::setfill('0') << (int) data[i] << ' ';
    }
    return ss.str();
}

void printi(int &foo) {
    printf("& %p->%d\n", &foo, foo);
    foo++;
}

void printii(int &&foo) {
    printf("& %p->%d\n", &foo, foo);
    foo++;
}

void printi(int *foo) {
    printf("* %p->%d\n", foo, *foo);
    (*foo)++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hello_cpp"); // Registering a node in ros master
    //   what():  Cannot use ros::Time::now() before the first NodeHandle has been created or ros::start() has been called.  If this is a standalone app or test that just uses ros::Time and does not communicate over ROS, you may also call ros::Time::init()
//  ros::Time::init();
    ros::NodeHandle nh;
    ros::start();
    cout << ros::this_node::getName() << endl;
    ROS_INFO("Welcome to ROS!");
    double val = 1. / 4;
    auto d = ros::Duration(val);
//  auto d = ros::Duration{0, 987654321};
    auto delay = calculate_edge(d);
    ROS_INFO("delay: %s %s", to_string(delay).c_str(), std::to_string(delay.toNSec()).c_str());
    ROS_INFO("is zero: %s", std::to_string(delay == ros::Duration{0, 0}).c_str());
    now();
    for (int i = 0; i < 1 && ros::ok(); i++) {
        auto delay1 = calculate_edge(d);

        delay1.sleep();
//      ros::Duration(0.5).sleep();
        now();
        since_edge(d);
    }
    ROS_INFO("x: %lf %d", fmod(val, 1.0 / 120), is_simple_frac(val));
    ROS_WARN("part 2");
//    auto timer = ros::Timer();
//    auto cb = ros::CallbackQueue();
    ros::SingleThreadedSpinner spinner{};
//    spinner.start();
//    auto future_time = ros::Time::now() + ros::Duration(1.0);
//    nh.createTimer(ros::Duration(0.01), [](const ros::TimerEvent& event) {
//        now();
//    }, false, true);
//    ros::SteadyTimer steady1 = nh.createSteadyTimer(ros::WallDuration(7/11.0), callback, false, true);
    ros::SteadyTimer steady;
    ros::SteadyTimer tick;
    ros::SteadyTimer tock;
    std::string steady_rate_s;
    if (const char *env_p = std::getenv("STEADY_PERIOD")) {
        if (env_p && env_p[0]) {
            ROS_INFO("got env");
            steady_rate_s = std::string(env_p);
        }

    }
    float period = 0;
    try {
        period = std::stof(std::string(steady_rate_s));
    } catch (...) {
        ROS_WARN("not a value: `%s`", steady_rate_s.c_str());
    }

    if (0) {
        std::cout << period << std::endl;
        tick = nh.createSteadyTimer(ros::WallDuration(2.0 / 30),
                                    [&tick, &tock, &nh](const ros::SteadyTimerEvent &event) {
                                        ROS_WARN("tick");
                                        now();
                                        tock = nh.createSteadyTimer(ros::WallDuration(0.05),
                                                                    [&tick](const ros::SteadyTimerEvent &event) {
                                                                        ROS_WARN("tock");
                                                                        now();

                                                                    }, true, true);
                                    }, true, true);
//    ros::Timer timer1 = nh.createTimer(ros::Duration(0.001), te_callback);
        ROS_INFO("out");
    }


    std_msgs::Header head;
    head.seq = 7;
//    auto x = head.seq;
    head.stamp = ros::Time::now();
    head.frame_id = "abcdefABCDEF";
    auto data = ros::serialization::serializeMessage(head);
//    std::cout << hexStr(reinterpret_cast<char *>(data.buf.get()), data.num_bytes) << std::endl;
//    printer::stream(std::cout, " ", head);
    std::vector<std_msgs::Header> array;

    boost::shared_ptr<std_msgs::Header> p_header(&head);
    array.emplace_back(std_msgs::Header(*p_header));
    p_header->seq = 456;
    p_header->frame_id = "new";
//    auto now = ros::Time();
    std::stringstream sbuf{"foo"};
    array.emplace_back(std_msgs::Header(*p_header));
    if (1) {
        for (auto it = array.rbegin(); it != array.rend(); ++it) {
            ros::message_operations::Printer<std_msgs::Header>::stream(std::cout, " ", *it);
            ros::message_operations::Printer<ros::Time>::stream(std::cout, " ", it->stamp);
//        std::cout << it->stamp << std::endl;
//        sprintf(&sbuf[0], "%.3f\n", it->stamp.toSec());
//        printf("%s", sbuf.c_str());
//        std::tm *tmp_tm;
            boost::posix_time::ptime boost_t = it->stamp.toBoost();
            auto time_s = boost::posix_time::to_iso_string(boost_t);
            time_s[8] = '_';
            cout << time_s << std::endl;
            cout << it->stamp;
//        tmp_tm = std::gmtime()
            int fooi, fooj;
            fooi = 1337;
            fooj = 13337;
            int *p = new int;
            *p = 456;
            sbuf << it->stamp;
            std::cout << "sbuf: " << sbuf.str() << " " << true << endl;

//        printii(fooi + 1);
//        printii(fooi + 1);
//        printi(p);
//        printi(p);
            std::cout << "\n---" << std::endl;
        }


        static const char *channels[] = {"rgb", "uv", "ir"};
        static std::map<std::string, double> cam_delays{{"rgb", 0.423},
                                                        {"uv",  0.234},
                                                        {"ir",  0.001}};
        auto nodeName = ros::this_node::getName();
        for (auto i = 0; i < 3; i++) {
            auto found = nodeName.find("/" + std::string(channels[i]) + "/");
            if (found != std::string::npos) {
                auto del = cam_delays[channels[i]];
                cout << "found at " << i << ": " << channels[i] << " del: " << del << endl;
            }
        }
    }


    EventCache ec3(ros::Duration(0.333), ros::Duration(0.343));
    EventCache ec2(ros::Duration(0.222));
    EventCache ec;
    std::cout << to_string(ec3.delay) << "  " << to_string(ec3.tol) << std::endl;

    if (period > 0) {
        steady = nh.createSteadyTimer(ros::WallDuration(period), [](const ros::SteadyTimerEvent &event) {
            now();
        });
    }
    std::map<ros::Duration, double> duration_map;


    CamAttr attr_u32{tPvDatatype::eTypeFloat32};
    attr_u32.set("3232");
    shared_ptr<string> sp;
    ImageHandler handler;
    sensor_msgs::Image image;
    image.data = vector<uint8_t >{0, 1,2,3};
//    image.header = std_msgs::Header;

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/topic", 5);
//    ros::Subscriber sub = nh.subscribe()<std_msgs::Header>("/topic", 5);

    ScheduledEvent sched;
    tock = nh.createSteadyTimer(ros::WallDuration(0.05),
                                [&tick](const ros::SteadyTimerEvent &event) {
                                    ROS_WARN("tock");
                                    now();

                                }, true, true);
//    delete sched.timer;
    cout << &image.data << typeid(image.data).name() << endl;
    sensor_msgs::ImageConstPtr imagePtr{ &image};
    handler.callback(imagePtr);

    tock.~SteadyTimer();
    ros::spin();

    ros::spin(spinner);
//    ros::waitForShutdown();
//    for (int i = 0; i < 999999; i++) {
//        ros::spinOnce();
//    }
    return 0;
}
