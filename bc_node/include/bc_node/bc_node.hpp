#ifndef BC_NODE__BC_NODE_HPP_
#define BC_NODE__BC_NODE_HPP_

#include <arpa/inet.h>
#include <bc_node/broadcast_message.hpp>
#include <bitset>
#include <builtin_interfaces/msg/time.hpp>
#include <fognav_msgs/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bc_node {

constexpr size_t kUdpBufferLength{1024};
constexpr size_t kSignatureSize{4};
constexpr size_t kSerializedTrajectoryMinSize{292};
constexpr size_t kSerializedTrajectoryMaxSize{316};
constexpr size_t kMaxAllowedMessages{100};

struct HeaderV0
{
    std::uint8_t version : 3;           // 000 = this, 111 = reserved
    std::uint8_t ros_serialization : 1; // 0 = BroadcastMessage, 1 = ROS serialization
    std::uint8_t message_type : 2;      // BroadcastMessage:
                                        //  00: <double>
                                        //  01: <float>
                                        //  10: <std::int16_t>
                                        //  11: BroadcastMessagemin
                                        // ROS serialization:
                                        //  00: Trajectory.msg
                                        //  01: undefined (free)
                                        //  10: undefined (free)
                                        //  11: undefined (free)
    std::uint8_t signature : 1;         // 1: Signed, 0: No signature
    std::uint8_t encryption : 1;        // 1: encrypted, 0: No signature
    char to_char() const
    {
        char c;
        c = version;
        c += ros_serialization << 3;
        c += message_type << 4;
        c += signature << 6;
        c += encryption << 7;
        return c;
    }
    std::string serialize() const { return std::string(1, to_char()); }
    void parse(const char c)
    {
        version = c & 0b00000111;
        ros_serialization = (c & 0b00001000) >> 3;
        message_type = (c & 0b00110000) >> 4;
        signature = (c & 0b01000000) >> 6;
        encryption = (c & 0b10000000) >> 7;
    }
    bool parse(const std::string& msg)
    {
        if (msg.length() < 1)
        {
            return false;
        }
        parse(msg.at(0));
        return true;
    }
};
inline std::ostream& operator<<(std::ostream& os, const HeaderV0 header)
{
    os << "\n ┌Enc┬Sig┬MsgType┬Ros┬──Version──┐\n │ " << (header.encryption ? "1" : "0") << " │ "
       << (header.signature ? "1" : "0") << " │ " << (header.message_type >> 1 & 0b1) << "   "
       << (header.message_type & 0b1) << " │ " << (header.ros_serialization ? "1" : "0") << " │ "
       << (header.version >> 2 & 0b1) << "   " << (header.version >> 1 & 0b1) << "   "
       << (header.version & 0b1) << " │ " << std::bitset<8>(header.to_char())
       << "\n └───┴───┴───┴───┴───┴───┴───┴───┘";
    return os;
}

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
    void periodic_clear_callback();

    /// @ingroup Message sending
    void broadcast_udp_message();
    bool send_udp(const std::string& msg);

    /// @ingroup Message receiving
    void receive_broadcast_message();
    std::string receive_udp();
    void print_trajectory(const fognav_msgs::msg::Trajectory::UniquePtr& trajectory,
                          int message_type);

    /// @ingroup Serialization functions
    std::string get_serialized_trajectory();
    bool deserialize_trajectory(const std::string& msg,
                                fognav_msgs::msg::Trajectory::UniquePtr& trajectory);
    template<typename T>
    bool get_trajectory_from_msg(const std::string& msg,
                                 fognav_msgs::msg::Trajectory::UniquePtr& trajectory);
    bool get_trajectory_from_msg(const std::string& msg,
                                 fognav_msgs::msg::Trajectory::UniquePtr& trajectory);
    template<typename T>
    std::string get_msg_from_trajectory();
    std::string get_msg_from_trajectory();

    /// @ingroup Signing
    std::string sign(const std::string& msg);
    bool verify_signature(const std::string& msg, const std::string& droneid);

    /// @ingroup Encryption
    std::string encrypt(const std::string& msg) { return msg; };
    std::string decrypt(const std::string& msg) { return msg; };

    /// @ingroup validation functions
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
    rclcpp::TimerBase::SharedPtr periodic_clear_timer_;
    rclcpp::TimerBase::SharedPtr receiver_timer_;

    /// @ingroup Parameters
    std::string broadcast_ip_{};
    uint16_t broadcast_port_{0};
    bool immediate_broadcast_{false};
    double signature_check_interval_{1.0};
    bool sign_messages_{true};
    bool encrypt_messages_{false};
    bool verify_all_signatures_{false};
    bool print_received_message_{true};
    double max_age_{0.5};
    std::string drone_id_{};
    int serialization_method_{0};

    /// @ingroup storage, containers

    /// Map to store the latest drone signature check timestamp
    std::map<std::string, rclcpp::Time> observed_senders_times_{};
    /// Container to count messages per ip address - e.g. detect message flooding
    std::map<in_addr_t, unsigned long> ip_addr_count_{};

    /// Previously received own trajectory
    fognav_msgs::msg::Trajectory trajectory_;
    /// Serialization containers for trajectories
    rcutils_uint8_array_t serialized_msg_;
    rcutils_uint8_array_t serialized_in_msg_;

    /// ros message types
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