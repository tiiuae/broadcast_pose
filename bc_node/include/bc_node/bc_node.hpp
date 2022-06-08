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
constexpr size_t kSignedMessageSize{612};

struct GeoPoint // 3*8 bytes = 24 Bytes = 192 bits
{
    double lat;
    double lon;
    double alt;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &lat, sizeof(double));
        serialized.append((const char*) &lon, sizeof(double));
        serialized.append((const char*) &alt, sizeof(double));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&lat, &message[0], sizeof(double));
        memcpy(&lon, &message[8], sizeof(double));
        memcpy(&alt, &message[16], sizeof(double));
    }
};

struct Point // 3*8 bytes = 24 Bytes = 192 bits
{
    double x;
    double y;
    double z;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(double));
        serialized.append((const char*) &y, sizeof(double));
        serialized.append((const char*) &z, sizeof(double));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(double));
        memcpy(&y, &message[8], sizeof(double));
        memcpy(&z, &message[16], sizeof(double));
    }
};

struct Quaternion // 4*8 bytes = 32 Bytes = 256 bits
{
    double x;
    double y;
    double z;
    double w;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(double));
        serialized.append((const char*) &y, sizeof(double));
        serialized.append((const char*) &z, sizeof(double));
        serialized.append((const char*) &w, sizeof(double));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(double));
        memcpy(&y, &message[8], sizeof(double));
        memcpy(&z, &message[16], sizeof(double));
        memcpy(&w, &message[24], sizeof(double));
    }
};

struct Pose // 56 Bytes = 448 bits
{
    Point point;
    Quaternion orientation;
    std::string serialize() const
    {
        std::string serialized;
        serialized.append(point.serialize());
        serialized.append(orientation.serialize());
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        point.deserialize(message);
        orientation.deserialize(message.substr(24));
    }
};

struct BroadcastMessage // 20 + 4 + 4 + 24 + 10*56
{
    char droneid[20];
    uint32_t sec;
    uint32_t nsec;
    uint16_t priority;
    bc_node::GeoPoint datum;
    bc_node::Pose path[10];

    std::string serialize() const
    {
        std::string serialized;
        serialized.append(droneid);
        serialized.append((const char*) &priority, sizeof(uint32_t));
        serialized.append((const char*) &sec, sizeof(uint32_t));
        serialized.append((const char*) &nsec, sizeof(uint32_t));
        serialized.append(datum.serialize());
        for (int i = 0; i < 10; ++i)
        {
            serialized.append(path[i].serialize());
        }
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        for (int j = 0; j < 20; j++)
        {
            droneid[j] = message[j];
        }
        // memcpy(&droneid, &message, 20);
        memcpy(&sec, &message[20], sizeof(uint16_t));
        memcpy(&sec, &message[22], sizeof(uint32_t));
        memcpy(&nsec, &message[26], sizeof(uint32_t));
        datum.deserialize(message.substr(30));
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(54 + 56 * i));
        }
    }
};

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
    void broadcast_udp_message_2();
    bool send_udp(const std::string& msg);

    /// @ingroup Message receiving
    void receive_broadcast_message();
    void receive_broadcast_message_2();
    std::string receive_udp();
    // bool receive_serialized_udp(rmw_serialized_message_t *received);

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

    ///@ingroup storage, containers

    // Container for observed senders book keeping
    std::set<std::string> observed_senders_{};
    std::map<std::string, rclcpp::Time> observed_senders_times_{};
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