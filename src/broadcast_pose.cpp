#include "broadcast_pose/broadcast_pose.hpp"

using std::placeholders::_1;

BroadcastPose::BroadcastPose() : Node("broadcast_pose")
{
    this->declare_parameter<std::string>("device_id", "undefined");
    this->declare_parameter<int>("right_of_way", 0);

    global_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "VehicleGlobalPosition_PubSubTopic", rclcpp::SystemDefaultsQoS(),
        std::bind(&BroadcastPose::global_pose_cb, this, _1));
    global_pose_pub_ = this->create_publisher<fog_msgs::msg::GlobalPositionRightOfWay>(
        "/fleet/global_pose_all", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(this->get_logger(), "Node created.");

    this->get_parameter("device_id", device_id_);
    this->get_parameter("right_of_way", right_of_way_);
}

void BroadcastPose::global_pose_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    fog_msgs::msg::GlobalPositionRightOfWay message;

    message.timestamp = msg->timestamp;
    message.timestamp_sample = msg->timestamp_sample;
    message.lat = msg->lat;
    message.lon = msg->lon;
    message.alt = msg->alt;
    message.right_of_way = right_of_way_;
    message.device_id = device_id_;

    global_pose_pub_->publish(message);
}
