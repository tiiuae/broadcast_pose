#include "bc_node/bc_node.hpp"
#include "bc_node/broadcast_message.hpp"
#include "bc_node/param_checker.hpp"
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bc_node {

template<typename BCM>
inline void fill_trajectory(fognav_msgs::msg::Trajectory::UniquePtr& trajectory,
                            const BCM& received,
                            const bool fixedpoint_pose = false,
                            const bool fixedpoint_datum = false)
{
    trajectory->droneid = std::string(received.droneid);
    trajectory->priority = received.priority;
    trajectory->header.stamp.sec = received.sec;
    trajectory->header.stamp.nanosec = received.nsec;
    if (fixedpoint_datum)
    {
        trajectory->datum.latitude = 0.000001 * static_cast<double>(received.datum.lat);
        trajectory->datum.longitude = 0.000001 * static_cast<double>(received.datum.lon);
        trajectory->datum.altitude = 0.1 * static_cast<double>(received.datum.alt);
    }
    else
    {
        trajectory->datum.latitude = received.datum.lat;
        trajectory->datum.longitude = received.datum.lon;
        trajectory->datum.altitude = received.datum.alt;
    }
    if (!fixedpoint_pose)
    {
        for (int i = 0; i < 10; i++)
        {
            trajectory->poses[i].position.x = received.path[i].point.x;
            trajectory->poses[i].position.y = received.path[i].point.y;
            trajectory->poses[i].position.z = received.path[i].point.z;
            trajectory->poses[i].orientation.x = received.path[i].orientation.x;
            trajectory->poses[i].orientation.y = received.path[i].orientation.y;
            trajectory->poses[i].orientation.z = received.path[i].orientation.z;
            trajectory->poses[i].orientation.w = received.path[i].orientation.w;
        }
        return;
    }
    for (int i = 0; i < 10; i++)
    {
        trajectory->poses[i].position.x = 0.1 * static_cast<double>(received.path[i].point.x);
        trajectory->poses[i].position.y = 0.1 * static_cast<double>(received.path[i].point.y);
        trajectory->poses[i].position.z = 0.1 * static_cast<double>(received.path[i].point.z);

        trajectory->poses[i].orientation.x = 0.0001
                                             * static_cast<double>(received.path[i].orientation.x);
        trajectory->poses[i].orientation.y = 0.0001
                                             * static_cast<double>(received.path[i].orientation.y);
        trajectory->poses[i].orientation.z = 0.0001
                                             * static_cast<double>(received.path[i].orientation.z);
        trajectory->poses[i].orientation.w = 0.0001
                                             * static_cast<double>(received.path[i].orientation.w);
    }
}

template<typename BCM>
inline void fill_bc_message(BCM& bc_message,
                            fognav_msgs::msg::Trajectory& trajectory,
                            const bool fixedpoint_pose = false)
{
    strncpy(bc_message.droneid, trajectory.droneid.c_str(), 20);
    bc_message.priority = trajectory.priority;
    bc_message.sec = trajectory.header.stamp.sec;
    bc_message.nsec = trajectory.header.stamp.nanosec;
    bc_message.datum.lat = trajectory.datum.latitude;
    bc_message.datum.lon = trajectory.datum.longitude;
    bc_message.datum.alt = trajectory.datum.altitude;
    if (fixedpoint_pose)
    {
        for (int i = 0; i < 10; i++)
        {
            bc_message.path[i].point.x = static_cast<int16_t>(
                std::round(10. * trajectory.poses[i].position.x));
            bc_message.path[i].point.y = static_cast<int16_t>(
                std::round(10. * trajectory.poses[i].position.y));
            bc_message.path[i].point.z = static_cast<int16_t>(
                std::round(10. * trajectory.poses[i].position.z));

            bc_message.path[i].orientation.x = static_cast<int16_t>(
                std::round(10000. * trajectory.poses[i].orientation.x));
            bc_message.path[i].orientation.y = static_cast<int16_t>(
                std::round(10000. * trajectory.poses[i].orientation.y));
            bc_message.path[i].orientation.z = static_cast<int16_t>(
                std::round(10000. * trajectory.poses[i].orientation.z));
            bc_message.path[i].orientation.w = static_cast<int16_t>(
                std::round(10000. * trajectory.poses[i].orientation.w));
        }
    }
    else
    {
        for (int i = 0; i < 10; i++)
        {
            bc_message.path[i].point.x = trajectory.poses[i].position.x;
            bc_message.path[i].point.y = trajectory.poses[i].position.y;
            bc_message.path[i].point.z = trajectory.poses[i].position.z;

            bc_message.path[i].orientation.x = trajectory.poses[i].orientation.x;
            bc_message.path[i].orientation.y = trajectory.poses[i].orientation.y;
            bc_message.path[i].orientation.z = trajectory.poses[i].orientation.z;
            bc_message.path[i].orientation.w = trajectory.poses[i].orientation.w;
        }
    }
}

/// @brief Constructor
///
/// @param[in]  node_name       Node name
/// @param[in]  node_options    Node options
BCNode::BCNode(const std::string& node_name, const rclcpp::NodeOptions& node_options)
    : rclcpp::Node(node_name, node_options)
{}

/// @brief Destructor
///
BCNode::~BCNode()
{
    pub_.reset();
    sub_.reset();
    if (socket_)
    {
        close(socket_);
    };
}

/// @brief Initialize node
///
/// @return true if initialized
bool BCNode::init()
{
    RCLCPP_INFO(get_logger(), "Intializing node");

    if (!(get_mandatory_param(*this, "broadcast_ip_address", broadcast_ip_)
          && get_mandatory_param(*this, "broadcast_port", broadcast_port_)))
    {
        RCLCPP_ERROR(get_logger(),
                     "Node initialization failed. Could not read mandatory parameters.");
        return false;
    }

    const auto broadcast_interval{std::chrono::duration<double>(
        get_gated_param(*this, "broadcast_interval_s", 0.1, 0.0, 60.0))};

    const auto receiver_interval{std::chrono::duration<double>(
        get_gated_param(*this, "receive_check_interval_s", 0.1, 0.0, 60.0))};

    signature_check_interval_ = get_gated_param(*this, "signature_check_interval_s", 1.0, 0.0, 60.0);

    const std::string trajectory_topic_in{
        get_param(*this, "input_trajectory_topic", std::string("navigation/trajectory"))};

    const std::string trajectory_topic_out{
        get_param(*this, "output_trajectory_topic", std::string("bc_node/trajectories"))};

    serialization_method_ = get_param(*this, "serialization_method", 0);

    switch (serialization_method_)
    {
    case 1:
        message_min_size_ = message_max_size_ = 614;
        break;
    case 2:
        message_min_size_ = message_max_size_ = 334;
        break;
    case 3:
        message_min_size_ = message_max_size_ = 194;
        break;
    case 4:
        message_min_size_ = message_max_size_ = 99;
        break;
    default:
        message_min_size_ = 610;
        message_max_size_ = 636;
        break;
    }

    // Own trajectory subscriber
    sub_ = create_subscription<fognav_msgs::msg::Trajectory>(trajectory_topic_in,
                                                             rclcpp::SystemDefaultsQoS(),
                                                             std::bind(&BCNode::trajectory_callback,
                                                                       this,
                                                                       std::placeholders::_1));

    // Received trajectories publisher
    pub_ = create_publisher<fognav_msgs::msg::Trajectory>(trajectory_topic_out, 100);

    // UDP broadcast timer
    if (broadcast_interval.count() > 0.0)
    {
        broadcast_timer_ = rclcpp::create_timer(this,
                                                get_clock(),
                                                broadcast_interval,
                                                std::bind(&BCNode::broadcast_timer_callback, this));
        immediate_broadcast_ = false;
    }
    else
    {
        immediate_broadcast_ = true;
    }

    // UDP receiver timer
    receiver_timer_ = rclcpp::create_timer(this,
                                           get_clock(),
                                           receiver_interval,
                                           std::bind(&BCNode::receive_broadcast_message, this));

    // signature check timer
    if (signature_check_interval_ > 0.0)
    {
        signature_check_timer_ = rclcpp::create_timer(this,
                                                      get_clock(),
                                                      std::chrono::duration<double>(
                                                          signature_check_interval_),
                                                      std::bind(&BCNode::signature_clear_callback,
                                                                this));
        verify_all_signatures_ = false;
    }
    else
    {
        verify_all_signatures_ = true;
    }

    // Open UDP socket
    if ((socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        RCLCPP_ERROR(get_logger(), "Cannot create UDP socket");
        return false;
    }
    // Set socket to broadcast mode
    int broadcast_flag{1};
    setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, &broadcast_flag, sizeof(broadcast_flag));

    // Reveiver configuration:
    memset(&my_addr_, 0, sizeof(my_addr_));
    my_addr_.sin_family = AF_INET;
    my_addr_.sin_port = htons(broadcast_port_);
    my_addr_.sin_addr.s_addr = INADDR_ANY;
    slen_ = sizeof(my_addr_);

    // bind socket
    if (bind(socket_, (sockaddr*) &my_addr_, sizeof(sockaddr)) == -1)
    {
        RCLCPP_ERROR(get_logger(), "Receiver socket bind failed.");
        close(socket_);
        return false;
    }

    // Sender configuration:
    memset((char*) &send_addr_, 0, sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_port = htons(broadcast_port_);
    if (inet_aton(broadcast_ip_.c_str(), &send_addr_.sin_addr) == 0)
    {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Invalid IP address " << broadcast_ip_.c_str()
                                                  << " - inet_aton() failed.");
        close(socket_);
        return false;
    }

    // typesupport for Serilizing trajectory messages
    trajectory_ts_
        = rosidl_typesupport_cpp::get_message_type_support_handle<fognav_msgs::msg::Trajectory>();

    // Serialized broadcast message initialization
    serialized_msg_ = rmw_get_zero_initialized_serialized_message();
    auto allocator = rcutils_get_default_allocator();
    auto initial_capacity = message_max_size_ + kSignatureSize;
    auto ret = rmw_serialized_message_init(&serialized_msg_, initial_capacity, &allocator);
    if (ret != RCL_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "failed to initialize serialized broadcast message");
        close(socket_);
        return false;
    }

    // Serialized receiver message initialization
    serialized_in_msg_ = rmw_get_zero_initialized_serialized_message();
    ret = rmw_serialized_message_init(&serialized_in_msg_, initial_capacity, &allocator);
    if (ret != RCL_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "failed to initialize serialized broadcast message");
        close(socket_);
        return false;
    }

    // Print out node configuration summary
    RCLCPP_INFO(get_logger(),
                "\n[ Node summary ]\n"
                "- input_trajectory_topic: %s\n"
                "- output_trajectory_topic: %s\n"
                "- broadcast_ip_adress: %s\n"
                "- broadcast_port: %d\n"
                "- broadcast_interval: %5.2f s\n"
                "- immediate_broadcast: %s\n"
                "- signature_check_interval: %5.2f s\n"
                "- verify_all_signatures: %s\n"
                "- serialization_method: %d\n",
                trajectory_topic_in.c_str(),
                trajectory_topic_out.c_str(),
                broadcast_ip_.c_str(),
                broadcast_port_,
                broadcast_interval.count(),
                immediate_broadcast_ ? "true" : "false",
                signature_check_interval_,
                verify_all_signatures_ ? "true" : "false",
                serialization_method_);

    RCLCPP_INFO(get_logger(), "Node initialized");

    return true;
}

/// @brief Callback function for trajectory messages
///
/// @param[in] msg  Trajectory containing next few seconds of own flight path
void BCNode::trajectory_callback(const fognav_msgs::msg::Trajectory::SharedPtr msg)
{
    {
        const std::scoped_lock<std::mutex> _(trajectory_access_);
        trajectory_ = *msg;
    }
    if (immediate_broadcast_)
    {
        broadcast_udp_message();
    }
}

/// @brief Timer callback function for own track broadcasting
///
void BCNode::broadcast_timer_callback()
{
    broadcast_udp_message();
}

/// @brief Timer callback function for clearing out the verified senders
///
void BCNode::signature_clear_callback()
{
    // clear all at once
    // observed_senders_.clear();

    // Clear outdated senders
    const rclcpp::Time now_ts = now();

    for (auto it = observed_senders_times_.cbegin(); it != observed_senders_times_.cend();)
    {
        if ((now_ts - it->second).seconds() > signature_check_interval_)
        {
            it = observed_senders_times_.erase(it);
        }
        else
        {
            ++it;
        }
    }
    // Do we need to adjust t
}

/// @brief Function to serialize Trajectory message to UDP packet string
///
/// @return Serialized trajectroy as string
std::string BCNode::get_serialized_trajectory()
{
    if (serialization_method_ == 1)
    {
        // generate message
        BroadcastMessage bc_message;
        {
            const std::scoped_lock<std::mutex> _(trajectory_access_);
            fill_bc_message(bc_message, trajectory_);
        }
        return bc_message.serialize();
    }
    if (serialization_method_ == 2)
    {
        // generate message
        BroadcastMessage32 bc_message;
        {
            const std::scoped_lock<std::mutex> _(trajectory_access_);
            fill_bc_message(bc_message, trajectory_);
        }
        return bc_message.serialize();
    }
    if (serialization_method_ == 3)
    {
        // generate message
        BroadcastMessage16 bc_message;
        {
            const std::scoped_lock<std::mutex> _(trajectory_access_);
            fill_bc_message(bc_message, trajectory_, true);
        }
        return bc_message.serialize();
    }
    if (serialization_method_ == 4)
    {
        // generate message
        BroadcastMessageMin bc_message;
        bool oversized_prio_warn{false};
        {
            const std::scoped_lock<std::mutex> _(trajectory_access_);
            strncpy(bc_message.droneid, trajectory_.droneid.c_str(), 20);

            bc_message.priority = trajectory_.priority;
            if (trajectory_.priority > std::numeric_limits<decltype(bc_message.priority)>::max())
            {
                bc_message.priority = std::numeric_limits<decltype(bc_message.priority)>::max();
                oversized_prio_warn = true;
            }

            bc_message.sec = trajectory_.header.stamp.sec;
            bc_message.nsec = trajectory_.header.stamp.nanosec;

            bc_message.datum.lat = static_cast<int32_t>(
                std::round(trajectory_.datum.latitude * 1000000.));
            bc_message.datum.lon = static_cast<int32_t>(
                std::round(trajectory_.datum.longitude * 1000000.));
            bc_message.datum.alt = static_cast<int16_t>(trajectory_.datum.altitude * 10.);

            for (int i = 0; i < 10; i++)
            {
                bc_message.path[i].x = static_cast<int16_t>(
                    std::round(10. * trajectory_.poses[i].position.x));
                bc_message.path[i].y = static_cast<int16_t>(
                    std::round(10. * trajectory_.poses[i].position.y));
                bc_message.path[i].z = static_cast<int16_t>(
                    std::round(10. * trajectory_.poses[i].position.z));
            }
        }
        if (oversized_prio_warn)
        {
            RCLCPP_WARN_ONCE(get_logger(),
                             "Priority %d exceeds broadcast limit %d",
                             trajectory_.priority,
                             std::numeric_limits<decltype(bc_message.priority)>::max());
        }
        // serialize message
        return bc_message.serialize();
    }

    rmw_ret_t ret;
    {
        const std::scoped_lock<std::mutex> _(trajectory_access_);
        ret = rmw_serialize(&trajectory_, trajectory_ts_, &serialized_msg_);
    }
    if (ret != RMW_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "Failed to serialize broadcast message");
        return {};
    }
    RCLCPP_DEBUG(get_logger(), "Serialized payload size: %ld Bytes", serialized_msg_.buffer_length);
    return std::string((char*) serialized_msg_.buffer, serialized_msg_.buffer_length);
}

/// @brief Function to deserialize UDP packet string to Trajectory message
///
/// @param[in]  msg Message to be deserialized
/// @return Deserialized trajectory (or nullptr)
fognav_msgs::msg::Trajectory::UniquePtr BCNode::deserialize_trajectory(const std::string& msg)
{
    auto trajectory = std::make_unique<fognav_msgs::msg::Trajectory>();
    if (serialization_method_ == 1)
    {
        BroadcastMessage received;
        received.deserialize(msg);
        fill_trajectory(trajectory, received);
    }
    else if (serialization_method_ == 2)
    {
        BroadcastMessage32 received;
        received.deserialize(msg);
        fill_trajectory(trajectory, received);
    }
    else if (serialization_method_ == 3)
    {
        BroadcastMessage16 received;
        received.deserialize(msg);
        fill_trajectory(trajectory, received, true);
    }
    else if (serialization_method_ == 4)
    {
        BroadcastMessageMin received;
        received.deserialize(msg);

        trajectory->droneid = std::string(received.droneid);

        trajectory->priority = received.priority;

        trajectory->header.stamp.sec = received.sec;
        trajectory->header.stamp.nanosec = received.nsec;

        trajectory->datum.latitude = 0.000001 * static_cast<double>(received.datum.lat);
        trajectory->datum.longitude = 0.000001 * static_cast<double>(received.datum.lon);
        trajectory->datum.altitude = 0.1 * static_cast<double>(received.datum.alt);
        for (int i = 0; i < 10; i++)
        {
            trajectory->poses[i].position.x = 0.1 * static_cast<double>(received.path[i].x);
            trajectory->poses[i].position.y = 0.1 * static_cast<double>(received.path[i].y);
            trajectory->poses[i].position.z = 0.1 * static_cast<double>(received.path[i].z);
        }
    }
    else
    {
        if (msg.size() < message_min_size_)
        {
            RCLCPP_ERROR(get_logger(), "Failed to deserialize serialized message. Too little data");
            return {};
        }
        memcpy(serialized_in_msg_.buffer, msg.data(), msg.size() - kSignatureSize);
        serialized_in_msg_.buffer_length = msg.size() - kSignatureSize;
        auto ret = rmw_deserialize(&serialized_in_msg_, trajectory_ts_, trajectory.get());
        if (ret != RMW_RET_OK)
        {
            RCLCPP_ERROR(get_logger(), "failed to deserialize serialized message.");
            return {};
        }
    }
    return trajectory;
}

/// @brief Function to send UDP packet
///
/// @param[in]  msg Message to be sent
/// @return true if send was successful
bool BCNode::send_udp(const std::string& msg)
{
    if (!socket_ || !send_addr_.sin_addr.s_addr || !send_addr_.sin_port)
    {
        RCLCPP_ERROR(get_logger(), "UDP sending failed - no socket iniziaized");
        return false;
    }
    if (msg.empty())
    {
        RCLCPP_WARN(get_logger(), "UDP sending failed - nothing to send");
        return false;
    }
    long int sent_bytes{
        sendto(socket_, msg.c_str(), msg.size(), 0, (struct sockaddr*) &send_addr_, slen_)};
    if (sent_bytes == -1)
    {
        RCLCPP_ERROR(get_logger(), "UDP sending failed - sendto()");
        return false;
    }
    RCLCPP_INFO_STREAM(get_logger(), "UDP " << sent_bytes << " / " << msg.size() << " Bytes sent.");
    return true;
}

/// @brief Function to broadcast own trajectory using UDP
///
void BCNode::broadcast_udp_message()
{
    const double age{(now() - trajectory_.header.stamp).seconds()};
    if (age > max_age_)
    {
        RCLCPP_WARN(get_logger(), "Timestamp too old for publishing: %.2f s", age);
        return;
    }
    else
    {
        RCLCPP_DEBUG(get_logger(), "Time delta %.2f seconds", age);
    }

    // serialize
    std::string message{get_serialized_trajectory()};
    // message = bc_message.serialize();
    RCLCPP_DEBUG(get_logger(), "Serialized message length: %ld Bytes", message.size());

    // sign
    std::string signed_msg{sign(message)};
    RCLCPP_DEBUG(get_logger(), "Signed message length: %ld Bytes", signed_msg.size());

    // send the message
    send_udp(signed_msg);
}

/// @brief Non-blocking function to receive UDP packet
///
/// @return  Message received as a string. Empty if no message received.
std::string BCNode::receive_udp()
{
    std::string received;
    if (!socket_)
    {
        RCLCPP_ERROR(get_logger(), "UDP receiving failed - no socket iniziaized");
        return received;
    }

    char buf[kUdpBufferLength];

    // socklen_t recv_addr_len = sizeof(recv_addr_);
    int received_bytes{0};
    // try to receive some data, this is a non-blocking call
    if ((received_bytes = recvfrom(
             socket_, buf, kUdpBufferLength, MSG_DONTWAIT, (struct sockaddr*) &recv_addr_, &slen_))
        == -1)
    {
        if (errno == EWOULDBLOCK || errno == EAGAIN)
        {
            RCLCPP_ERROR(get_logger(), "recvfrom - No more data: %s", strerror(errno));
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "recvfrom() : %s", strerror(errno));
        }
        return received;
    }

    received.assign(buf, received_bytes);
    // print received data and sender details
    RCLCPP_INFO(get_logger(),
                "Received %d B packet from %s:%d",
                received_bytes,
                inet_ntoa(recv_addr_.sin_addr),
                ntohs(recv_addr_.sin_port));
    if (received.empty())
    {
        RCLCPP_WARN(get_logger(), "UDP receiving failed - nothing received");
    }
    return received;
}

/// @brief Receive (UDP), check and relay (ROS) other drones trajectories
///
void BCNode::receive_broadcast_message()
{
    while (true)
    {
        // Try to receive trajectory info
        std::string received_str = receive_udp();
        if (received_str.empty())
        {
            RCLCPP_DEBUG(get_logger(), "No more data");
            return;
        }
        else if (received_str.size() < message_min_size_ || received_str.size() > message_max_size_)
        {
            RCLCPP_WARN(get_logger(),
                        "Received unexpected message size (%ld) from IP: %s",
                        received_str.size(),
                        //                        expected_udp_packet_size_,
                        inet_ntoa(recv_addr_.sin_addr));
            continue;
        }

        // Deserialize trajectory from UDP packet
        auto trajectory = deserialize_trajectory(received_str);
        if (!trajectory)
        {
            RCLCPP_ERROR(get_logger(), "failed to deserialize serialized message. received nullptr");
            continue;
        }

        sender_count_[trajectory->droneid]++;

        // Check that IP matches the droneid
        if (!check_ip(trajectory->droneid, recv_addr_))
        {
            RCLCPP_WARN_STREAM(get_logger(),
                               "DRONE_ID vs. IP verification failed: "
                                   << trajectory->droneid
                                   << " not matching with:" << inet_ntoa(recv_addr_.sin_addr));
            /// @todo Do some security signalling...
            continue;
        }

        // Verify signature
        if (!observed_senders_times_.count(trajectory->droneid))
        {
            if (!verify_signature(received_str, trajectory->droneid))
            {
                RCLCPP_WARN_STREAM(get_logger(),
                                   "Signature verification failed: " << trajectory->droneid);
                // Do some security signalling...
                continue;
            }
            observed_senders_times_[trajectory->droneid] = now();
            RCLCPP_INFO_STREAM(get_logger(), "Signature verified: " << trajectory->droneid);
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Signature verification skipped.");
        }
        // Relay message forward
        pub_->publish(std::move(trajectory));
    }
}

} // namespace bc_node
