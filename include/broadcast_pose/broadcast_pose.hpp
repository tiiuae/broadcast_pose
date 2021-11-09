#ifndef BROADCAST_POSE_HPP
#define BROADCAST_POSE_HPP

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <fog_msgs/msg/global_position_right_of_way.hpp>

class BroadcastPose : public rclcpp::Node
{
public:
    BroadcastPose();

private:
    void global_pose_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pose_sub_;
    rclcpp::Publisher<fog_msgs::msg::GlobalPositionRightOfWay>::SharedPtr global_pose_pub_;

    std::string device_id_;
    int right_of_way_;
};

#endif // BROADCAST_POSE_HPP
