#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <fog_msgs/msg/trajectory.hpp>

using namespace std::chrono_literals;

class NavMock : public rclcpp::Node {
public:
  NavMock() : Node("nav_mock"), count_(0) {
    drone_id_ = declare_parameter("drone_id", "Testi_drone_1");
    time_skew_ = declare_parameter("time_skew", 0.0);
    path_jump6_ = declare_parameter("path_jump_on_6", 0.0);
    priority_ = declare_parameter("priority", 123);
    datum_lat_ = declare_parameter("datum_latitude", 23.22122);
    datum_lon_ = declare_parameter("datum_longitude", 64.54332);
    datum_alt_ = declare_parameter("datum_altitude", 123.311);
    publish_interval_s_ = declare_parameter("publish_interval_s", 0.1);
    publisher_ = create_publisher<fog_msgs::msg::Trajectory>(
        "navigation/trajectory", 10);
    sub_ = create_subscription<fog_msgs::msg::Trajectory>(
        "bc_node/trajectories", rclcpp::SystemDefaultsQoS(),
        std::bind(&NavMock::trajectory_callback, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::duration<double>(publish_interval_s_),
        std::bind(&NavMock::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = fog_msgs::msg::Trajectory();
    msg.header.stamp = now() + rclcpp::Duration::from_seconds(time_skew_);
    msg.priority = priority_;
    msg.droneid = drone_id_;
    msg.datum.latitude = datum_lat_;
    msg.datum.longitude = datum_lon_;
    msg.datum.altitude = datum_alt_;
    for (int i = 0; i < 10; ++i) {
      msg.path[i].x = i + 0.2 * i;
      msg.path[i].y = -i + 0.1;
      msg.path[i].z = 0.03 * i;
    }
    msg.path[6].x += path_jump6_;
    msg.path[6].y += path_jump6_;
    msg.path[6].z += path_jump6_;

    RCLCPP_INFO(get_logger(), "Publishing: '%ld'", ++count_);
    RCLCPP_INFO_STREAM(get_logger(),
                       "Published message:\n -droneid:"
                           << msg.droneid << "\n -priority:" << msg.priority
                           << "\n -sec:" << msg.header.stamp.sec
                           << "\n -nanosec:" << msg.header.stamp.nanosec
                           << "\n -datum lat:" << msg.datum.latitude
                           << "\n -datum lon:" << msg.datum.longitude
                           << "\n -datum alt:" << msg.datum.altitude
                           << "\n -Path:");
    // for (int i = 0; i < 10; ++i) {
    int i = 6;
    RCLCPP_INFO_STREAM(get_logger(), i << "\nPostion:\n --x: " << msg.path[i].x
                                       << "\n --y: " << msg.path[i].y
                                       << "\n --z: " << msg.path[i].z);
    //}
    publisher_->publish(msg);
  }
  void trajectory_callback(const fog_msgs::msg::Trajectory::SharedPtr msg) {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "Received message:\n -droneid:"
            << msg->droneid << " (" << msg->droneid.length() << ")\n -priority:"
            << msg->priority << "\n - sec:" << msg->header.stamp.sec
            << "\n -nanosec:" << msg->header.stamp.nanosec << "\n - datum lat:"
            << msg->datum.latitude << "\n -datum lon:" << msg->datum.longitude
            << "\n -datum alt:" << msg->datum.altitude << "\n -Path:");
    // for (int i = 0; i < 10; ++i) {
    int i = 6;
    RCLCPP_INFO_STREAM(get_logger(), i << "\nPostion:\n-- x: " << msg->path[i].x
                                       << "\n --y: " << msg->path[i].y
                                       << "\n --z: " << msg->path[i].z);
    //}
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<fog_msgs::msg::Trajectory>::SharedPtr publisher_;
  rclcpp::Subscription<fog_msgs::msg::Trajectory>::SharedPtr sub_;
  size_t count_;
  std::string drone_id_;
  double time_skew_;
  double path_jump6_;
  int priority_;
  double datum_lat_;
  double datum_lon_;

  double datum_alt_;
  double publish_interval_s_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavMock>());
  rclcpp::shutdown();
  return 0;
}
