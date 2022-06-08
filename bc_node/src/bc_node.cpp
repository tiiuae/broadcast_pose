#include "bc_node/bc_node.hpp"
#include "bc_node/param_checker.hpp"
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bc_node {

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
          && get_mandatory_param(*this, "broadcast_port", broadcast_port_)
          && get_mandatory_param(*this, "right_of_way_priority", right_of_way_)))
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

    device_id_ = get_param(*this, "device_id", std::string("undefined"));

    const std::string trajectory_topic_in{
        get_param(*this, "input_trajectory_topic", std::string("navigation/trajectory"))};

    const std::string trajectory_topic_out{
        get_param(*this,
                  "output_trajectory_topic",
                  std::string("right_of_way_broadcaster/trajectories"))};

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
    slen_ = sizeof(my_addr_); // vois olla nätinpää jos ottais rcv_stä

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
    auto initial_capacity = 700u;
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
                "- broadcast ip adress: %s\n"
                "- broadcast port: %d\n"
                "- broadcast interval: %5.2f s\n"
                "- immediate broadcast: %s\n"
                "- signature check interval: %5.2f s\n"
                "- verify all signatures: %s\n"
                "- right of way priority: %d\n"
                "- device_id: %s\n",
                trajectory_topic_in.c_str(),
                trajectory_topic_out.c_str(),
                broadcast_ip_.c_str(),
                broadcast_port_,
                broadcast_interval.count(),
                immediate_broadcast_ ? "true" : "false",
                signature_check_interval_,
                verify_all_signatures_ ? "true" : "false",
                right_of_way_,
                device_id_.c_str());

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
    RCLCPP_DEBUG_STREAM(get_logger(), "UDP " << sent_bytes << " / " << msg.size() << " Bytes sent.");
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
        RCLCPP_INFO(get_logger(),
                    "Time delta %.2f seconds",
                    (now() - trajectory_.header.stamp).seconds());
    }

    std::string message;
    // generate message
    BroadcastMessage bc_message;
    strncpy(bc_message.droneid, device_id_.c_str(), 20);
    bc_message.priority = right_of_way_;
    {
        const std::scoped_lock<std::mutex> _(trajectory_access_);
        bc_message.sec = trajectory_.header.stamp.sec;
        bc_message.nsec = trajectory_.header.stamp.nanosec;
        bc_message.datum.lat = trajectory_.datum.latitude;
        bc_message.datum.lon = trajectory_.datum.longitude;
        bc_message.datum.alt = trajectory_.datum.altitude;
        for (int i = 0; i < 10; i++)
        {
            bc_message.path[i].point.x = trajectory_.poses[i].position.x;
            bc_message.path[i].point.y = trajectory_.poses[i].position.y;
            bc_message.path[i].point.z = trajectory_.poses[i].position.z;

            bc_message.path[i].orientation.x = trajectory_.poses[i].orientation.x;
            bc_message.path[i].orientation.y = trajectory_.poses[i].orientation.y;
            bc_message.path[i].orientation.z = trajectory_.poses[i].orientation.z;
            bc_message.path[i].orientation.w = trajectory_.poses[i].orientation.w;
        }
    }
    // serialize
    message = bc_message.serialize();

    // sign
    std::string signed_msg{sign(message)};

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
    RCLCPP_DEBUG(get_logger(),
                 "Received %d B packet from %s:%d\n",
                 received_bytes,
                 inet_ntoa(recv_addr_.sin_addr),
                 ntohs(recv_addr_.sin_port));
    RCLCPP_DEBUG(get_logger(), "Data: %s\n", buf);
    RCLCPP_DEBUG_STREAM(get_logger(), "Str: " << received);

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
        else if (received_str.size() != kSignedMessageSize)
        {
            RCLCPP_WARN(get_logger(),
                        "Received unexpected message size from IP: %s",
                        inet_ntoa(recv_addr_.sin_addr));
            continue;
        }

        // Deserialize
        BroadcastMessage received;
        received.deserialize(received_str);

        // Check that IP matches the droneid
        if (!check_ip(received.droneid, recv_addr_))
        {
            RCLCPP_WARN_STREAM(get_logger(),
                               "DRONE_ID vs. IP verification failed: "
                                   << received.droneid
                                   << " not matching with:" << inet_ntoa(recv_addr_.sin_addr));
            /// @todo Do some security signalling...
            continue;
        }

        // Verify signature
        if (/*!observed_senders_.count(received.droneid)*/
            !observed_senders_times_.count(received.droneid))
        {
            if (!verify_signature(received_str, received.droneid))
            {
                RCLCPP_WARN_STREAM(get_logger(),
                                   "Signature verification failed: " << received.droneid);
                /// @todo Do some security signalling...
                continue;
            }
            // observed_senders_.insert(received.droneid);
            observed_senders_times_[received.droneid] = now();
            RCLCPP_INFO_STREAM(get_logger(), "Signature verified: " << received.droneid);
        }
        sender_count_[received.droneid]++;

        // Relay message forward
        auto trajectory = std::make_unique<fognav_msgs::msg::Trajectory>();
        trajectory->droneid = std::string(received.droneid);
        trajectory->priority = received.priority;
        trajectory->header.stamp.sec = received.sec;
        trajectory->header.stamp.nanosec = received.nsec;
        trajectory->datum.latitude = received.datum.lat;
        trajectory->datum.longitude = received.datum.lon;
        trajectory->datum.altitude = received.datum.alt;
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
        pub_->publish(std::move(trajectory));
    }
}

////// Below is solution using ROS/DDS provided serialization for UDP
/// sending/receiving

/// @brief Receive (UDP), check and relay (ROS) other drones trajectories
///
void BCNode::broadcast_udp_message_2()
{
    // Check that message is fresh enough
    const double age{(now() - trajectory_.header.stamp).seconds()};
    if (age > max_age_)
    {
        RCLCPP_WARN(get_logger(), "Timestamp too old for publishing: %.2f s", age);
        return;
    }
    else
    {
        RCLCPP_INFO(get_logger(),
                    "Time delta %.2f seconds",
                    (now() - trajectory_.header.stamp).seconds());
    }

    // Serialize the message
    auto ret = rmw_serialize(&trajectory_, trajectory_ts_, &serialized_msg_);
    if (ret != RMW_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "Failed to serialize broadcast message");
        return;
    }
    RCLCPP_DEBUG(get_logger(), "Serialized payload size: %ld Bytes", serialized_msg_.buffer_length);

    // Sign the message
    std::string message((char*) serialized_msg_.buffer, serialized_msg_.buffer_length);
    std::string signed_msg{sign(message)};

    // Send the message
    send_udp(signed_msg);
}

/// @brief Function to broadcast own trajectory using UDP
///
void BCNode::receive_broadcast_message_2()
{
    while (true)
    {
        // try to receive some data
        std::string received_str = receive_udp();
        if (received_str.empty())
        {
            RCLCPP_DEBUG(get_logger(), "No more data");
            return;
        }

        // Conpy to to serialized msg
        ///@todo fix the size check
        if (received_str.size() < kSignatureSize + 100)
        {
            memcpy(serialized_in_msg_.buffer,
                   received_str.data(),
                   received_str.size() - kSignatureSize);
            serialized_in_msg_.buffer_length = received_str.size() - kSignatureSize;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Invalid size UDP packet received.");
            continue;
        }

        // Deserialize
        auto received = std::make_unique<fognav_msgs::msg::Trajectory>();
        auto ret = rmw_deserialize(&serialized_in_msg_, trajectory_ts_, received.get());
        if (ret != RMW_RET_OK)
        {
            RCLCPP_ERROR(get_logger(), "failed to deserialize serialized message.");
            continue;
        }

        // Verify signature
        if (!observed_senders_times_.count(received->droneid))
        {
            if (!verify_signature(received_str, received->droneid))
            {
                RCLCPP_WARN_STREAM(get_logger(),
                                   "Signature verification failed: " << received->droneid);
                // Do some security signalling...
                continue;
            }
            observed_senders_times_[received->droneid] = now();
            RCLCPP_INFO_STREAM(get_logger(), "Signature verified: " << received->droneid);
        }

        sender_count_[received->droneid]++;
        // Relay message forward
        pub_->publish(std::move(received));
    }
}

} // namespace bc_node
