#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "broadcast_pose/broadcast_pose.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BroadcastPose>());
  rclcpp::shutdown();
  return 0;
}
