/*
   Hello CPP
 */
#include <iostream>
#include<ros/ros.h>

bool  is_simple_frac(double d);
ros::Duration calculate_edge(ros::Duration const & granularity, ros::Duration const & thresh);
ros::Duration calculate_edge(ros::Duration const & granularity);
std::string to_string(ros::Time const &);
std::string to_string(ros::Duration const &);
std::string to_string(ros::Rate const &);
int64_t to_nano64(ros::Duration const &);

bool is_simple_frac(double d) {
    const double ratio = 1.0/120; // ratio considered for "simple fractions"
    auto m = fmod(d, ratio);
    return m < 1e-6;
}

std::string to_string(ros::Time const & t) {
    return std::to_string(t.toSec());
}
std::string to_string(ros::Duration const & t) {
    return std::to_string(t.toSec());
}
std::string to_string(ros::Rate const & t) {
    return std::to_string(ros::Duration(t).toSec());
}

ros::Duration calculate_edge(ros::Duration const & granularity) {
    return calculate_edge(granularity, ros::Duration{0,0});
}
ros::Duration calculate_edge(ros::Duration const & granularity, ros::Duration const & thresh) {
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
    if (out < thresh) return ros::Duration{0,0};
  return out;

}

ros::Duration since_edge(ros::Duration const & granularity) {
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

void callback(const ros::SteadyTimerEvent& event) {
    ROS_WARN("steady callback");
    now();
}
void te_callback(const ros::TimerEvent& event) {
    ROS_WARN("plain timer callback");
    now();
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"hello_cpp"); // Registering a node in ros master
  //   what():  Cannot use ros::Time::now() before the first NodeHandle has been created or ros::start() has been called.  If this is a standalone app or test that just uses ros::Time and does not communicate over ROS, you may also call ros::Time::init()
//  ros::Time::init();
    ros::NodeHandle nh;
  ros::start();
  ROS_INFO("Welcome to ROS!");
  double val = 1./4;
  auto d = ros::Duration(val);
//  auto d = ros::Duration{0, 987654321};
  auto delay = calculate_edge(d);
    ROS_INFO("delay: %s %s", to_string(delay).c_str(), std::to_string(delay.toNSec()).c_str());
    ROS_INFO("is zero: %s", std::to_string(delay == ros::Duration{0,0}).c_str());
    now();
    for (int i = 0; i < 1 && ros::ok(); i++) {
        auto delay1 = calculate_edge(d);

      delay1.sleep();
//      ros::Duration(0.5).sleep();
      now();
      since_edge(d);
  }
    ROS_INFO("x: %lf %d", fmod(val, 1.0/120), is_simple_frac(val));
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
    ros::SteadyTimer steady ;
    ros::SteadyTimer tick ;
    ros::SteadyTimer tock;
    std::string steady_rate_s;
    if (const char *env_p = std::getenv("STEADY_PERIOD")) {
        if (env_p  && env_p[0]) {
            ROS_INFO("got env");
            steady_rate_s = std::string(env_p);
        }

    }
    float x = 0;
    try {
        x = std::stof(std::string(steady_rate_s));
    } catch (...) {
        ROS_WARN("not a value: `%s`", steady_rate_s.c_str());
    }

    std::cout << x << std::endl;
    tick = nh.createSteadyTimer(ros::WallDuration(2.0/3), [&tick, &tock, &nh](const ros::SteadyTimerEvent &event) {
        ROS_WARN("tick");
        now();
        tock = nh.createSteadyTimer(ros::WallDuration(0.5), [&tick](const ros::SteadyTimerEvent &event) {
            ROS_WARN("tock");
            now();

            }, true, true);
        }, true, true);
//    ros::Timer timer1 = nh.createTimer(ros::Duration(0.001), te_callback);
    ROS_INFO("out");

    if (x > 0) {
        steady = nh.createSteadyTimer(ros::WallDuration(x), [](const ros::SteadyTimerEvent &event) {
            now();
        });
    }
    ros::spin(spinner);
//    ros::waitForShutdown();
//    for (int i = 0; i < 999999; i++) {
//        ros::spinOnce();
//    }
  return 0;
}
