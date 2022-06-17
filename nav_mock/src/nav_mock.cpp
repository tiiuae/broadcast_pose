#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <fognav_msgs/msg/trajectory.hpp>

using namespace std::chrono_literals;

class NavMock : public rclcpp::Node {
public:
  NavMock() : Node("nav_mock"), count_(0) {
    publisher_ = this->create_publisher<fognav_msgs::msg::Trajectory>(
        "navigation/trajectory", 10);
    sub_ = this->create_subscription<fognav_msgs::msg::Trajectory>(
        "bc_node/trajectories", rclcpp::SystemDefaultsQoS(),
        std::bind(&NavMock::trajectory_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&NavMock::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = fognav_msgs::msg::Trajectory();
    msg.header.stamp = now();
    msg.priority = 123;
    msg.droneid = "Testi_drone_1";
    msg.datum.latitude = 23.22122;
    msg.datum.longitude = 64.54332;
    msg.datum.altitude = 123.311;
    for (int i = 0; i < 10; ++i) {
      msg.path[i].x = i + 0.2 * i;
      msg.path[i].y = -i + 0.1;
      msg.path[i].z = 0.03 * i;
    }
    RCLCPP_INFO(get_logger(), "Publishing: '%ld'", ++count_);
    RCLCPP_INFO_STREAM(get_logger(),
                       "Published message:\n -droneid:"
                           << msg.droneid << "\n -priority:" << msg.priority
                           << "\n - sec:" << msg.header.stamp.sec
                           << "\n -nanosec:" << msg.header.stamp.nanosec
                           << "\n - datum lat:" << msg.datum.latitude
                           << "\n -datum lon:" << msg.datum.longitude
                           << "\n -datum alt:" << msg.datum.altitude
                           << "\n -Path:");
    // for (int i = 0; i < 10; ++i) {
    int i = 6;
    RCLCPP_INFO_STREAM(get_logger(), i << "\nPostion:\n-- x: " << msg.path[i].x
                                       << "\n --y: " << msg.path[i].y
                                       << "\n --z: " << msg.path[i].z);
    //}
    publisher_->publish(msg);
  }
  void trajectory_callback(const fognav_msgs::msg::Trajectory::SharedPtr msg) {
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
  rclcpp::Publisher<fognav_msgs::msg::Trajectory>::SharedPtr publisher_;
  rclcpp::Subscription<fognav_msgs::msg::Trajectory>::SharedPtr sub_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavMock>());
  rclcpp::shutdown();
  return 0;
}
