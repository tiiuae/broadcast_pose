#ifndef BC_NODE__BC_NODE_HPP_
#define BC_NODE__BC_NODE_HPP_

#include <arpa/inet.h>
#include <fognav_msgs/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
//#include <memory>
//#include <mutex>
//#include <string>

namespace bc_node {

constexpr size_t kUdpBufferLength{1024};
constexpr size_t kSignatureSize{0};

class BCNode : public rclcpp::Node
{
public:
    BCNode(const std::string& node_name, const rclcpp::NodeOptions& node_options);
    virtual ~BCNode();
    bool init();

private:
    /// @ingroup Subscriber callbacks
    void trajectory_callback(const fognav_msgs::msg::Trajectory::SharedPtr msg);

    /// @ingroup Timer callbacks
    void broadcast_timer_callback();
    void signature_clear_callback();

    /// @ingroup Message sending
    void broadcast_udp_message();
    bool send_udp(const std::string& msg);

    /// @ingroup Message receiving
    void receive_broadcast_message();
    std::string receive_udp();

    /// @ingroup Serialization functions
    std::string get_serialized_trajectory();
    fognav_msgs::msg::Trajectory::UniquePtr deserialize_trajectory(const std::string& msg);

    /// @ingroup signing
    std::string sign(const std::string& msg) { return msg; };
    bool verify_signature(const std::string& /*msg*/, std::string /*droneid*/) { return true; }

    bool check_ip(const std::string& /*droneid*/, const struct sockaddr_in /*recv_addr*/)
    {
        return true;
    }

    /// @ingroup Publishers
    rclcpp::Publisher<fognav_msgs::msg::Trajectory>::SharedPtr pub_;

    /// @ingroup Subscribers
    rclcpp::Subscription<fognav_msgs::msg::Trajectory>::SharedPtr sub_;

    /// @ingroup Timers
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    rclcpp::TimerBase::SharedPtr signature_check_timer_;
    rclcpp::TimerBase::SharedPtr receiver_timer_;

    /// @ingroup Parameters
    std::string broadcast_ip_{};
    uint16_t broadcast_port_{0};
    // double broadcast_interval_{0.1};
    bool immediate_broadcast_{false};
    double signature_check_interval_{1.0};
    bool verify_all_signatures_{false};
    uint16_t right_of_way_{100};
    std::string device_id_{"undefined"};
    double max_age_{0.5};
    int serialization_method_{0};

    size_t message_min_size_{614};
    size_t message_max_size_{614};

    /// @ingroup storage, containers

    // Container for observed senders book keeping
    // std::set<std::string> observed_senders_{};
    // Map to store the latest drone signature check timestamp
    std::map<std::string, rclcpp::Time> observed_senders_times_{};
    // Container to count messages per drone - e.g. detect message flooding
    std::map<std::string, uint16_t> sender_count_{};

    /// Previously received own trajectory
    fognav_msgs::msg::Trajectory trajectory_;
    /// Serialization containers for trajectories
    rcutils_uint8_array_t serialized_msg_;
    rcutils_uint8_array_t serialized_in_msg_;

    // ros message types
    const rosidl_message_type_support_t* trajectory_ts_;

    /// @ingroup socket
    struct sockaddr_in my_addr_, send_addr_, recv_addr_;
    socklen_t slen_;
    int socket_;

    /// @ingroup mutex
    std::mutex trajectory_access_;
};
} // namespace bc_node

#endif