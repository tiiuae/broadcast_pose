#include "bc_node/bc_node.hpp"
#include "bc_node/broadcast_message.hpp"
#include "bc_node/param_checker.hpp"
#include "bc_node/validation.hpp"
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

    sign_messages_ = get_param(*this, "sign_messages", true);
    encrypt_messages_ = get_param(*this, "encrypt_messages", false);
    signature_check_interval_ = get_gated_param(*this, "signature_check_interval_s", 1.0, 0.0, 60.0);
    verify_all_signatures_ = (signature_check_interval_ <= 0.0);

    max_age_ = get_gated_param(*this, "broadcast_trajectory_max_age_s", 0.5, 0.0, 60.0);

    const std::string trajectory_topic_in{
        get_param(*this, "input_trajectory_topic", std::string("navigation/trajectory"))};

    const std::string trajectory_topic_out{
        get_param(*this, "output_trajectory_topic", std::string("bc_node/trajectories"))};

    drone_id_ = get_param(*this, "drone_id", std::string("Testi_drone_1"));
    serialization_method_ = get_param(*this, "serialization_method", 0);
    print_received_message_ = get_param(*this, "print_received_message", true);
    size_t message_min_size{0};
    size_t message_max_size{0};
    switch (serialization_method_)
    {
    case 1:
        message_min_size = message_max_size = BroadcastMessagePoint<double>::size(); // 294;
        break;
    case 2:
        message_min_size = message_max_size = BroadcastMessagePoint<float>::size(); // 174;
        break;
    case 3:
        message_min_size = message_max_size = BroadcastMessagePoint<std::int16_t>::size(); // 114;
        break;
    case 4:
        message_min_size = message_max_size = BroadcastMessageMin::size(); // 99;
        break;
    default:
        message_min_size = kSerializedTrajectoryMinSize;
        message_max_size = kSerializedTrajectoryMaxSize;
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
    /// Periodic clearing of some counters
    periodic_clear_timer_ = rclcpp::create_timer(this,
                                                 get_clock(),
                                                 std::chrono::duration<double>(1.0),
                                                 std::bind(&BCNode::periodic_clear_callback, this));

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

    // typesupport for Serializing trajectory messages
    trajectory_ts_
        = rosidl_typesupport_cpp::get_message_type_support_handle<fognav_msgs::msg::Trajectory>();

    // Serialized broadcast message initialization
    serialized_msg_ = rmw_get_zero_initialized_serialized_message();
    auto allocator = rcutils_get_default_allocator();
    auto initial_capacity = kSerializedTrajectoryMaxSize + kSignatureSize;
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
                "- input_trajectory_topic:\t %s\n"
                "- output_trajectory_topic:\t %s\n"
                "- broadcast_ip_adress:\t %s\n"
                "- broadcast_port:\t\t %d\n"
                "- broadcast_interval:\t %.2f s\n"
                "- immediate_broadcast:\t %s\n"
                "- message_max_age:\t\t %.2f s\n"
                "- sign_messages:\t\t %s\n"
                "- encrypt_messages:\t\t %s\n"
                "- print_received_message:\t%s\n"
                "- signature_check_interval:\t %.2f s\n"
                "- verify_all_signatures:\t %s\n"
                "- serialization_method:\t %d\n"
                "- drone_id:\t\t\t %s\n"
                "- BroadcastMessage size:\t [%ld .. %ld] Bytes",

                trajectory_topic_in.c_str(),
                trajectory_topic_out.c_str(),
                broadcast_ip_.c_str(),
                broadcast_port_,
                broadcast_interval.count(),
                immediate_broadcast_ ? "true" : "false",
                max_age_,
                sign_messages_ ? "true" : "false",
                encrypt_messages_ ? "true" : "false",
                print_received_message_ ? "true" : "false",
                signature_check_interval_,
                verify_all_signatures_ ? "true" : "false",
                serialization_method_,
                drone_id_.c_str(),
                message_min_size,
                message_max_size);

    RCLCPP_INFO(get_logger(), "Node initialized");

    return true;
}

/// @brief Callback function for trajectory messages
///
/// @param[in] msg  Trajectory containing next few seconds of own flight path
void BCNode::trajectory_callback(const fognav_msgs::msg::Trajectory::SharedPtr msg)
{
    if (!validate_geopoint(*this, msg->datum)) //! validate_trajectory(*this, msg))
    {
        return;
    }
    if (!validate_droneid(*this, msg->droneid, drone_id_))
    {
        /// @todo Do some security signalling...
        return;
    }
    else
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

/// @brief Timer callback function for clearing the counters
///
void BCNode::periodic_clear_callback()
{
    // Clear ip address counters
    ip_addr_count_.clear();
}

/// @brief Function to serialize Trajectory message to UDP packet string
///
/// @tparam T   type used in BroadcastMessage path points (double, float, std::int16_t)
/// @return Serialized trajectroy as string
template<typename T>
std::string BCNode::get_msg_from_trajectory()
{
    BroadcastMessagePoint<T> bc_message;
    {
        const std::scoped_lock<std::mutex> _(trajectory_access_);
        bc_message.from_rosmsg(trajectory_);
    }
    return bc_message.serialize();
}

/// @brief Function to serialize Trajectory message to UDP packet string
///
/// @return Serialized trajectroy as string
std::string BCNode::get_msg_from_trajectory()
{
    BroadcastMessageMin bc_message;
    {
        const std::scoped_lock<std::mutex> _(trajectory_access_);
        bc_message.from_rosmsg(trajectory_);
    }
    return bc_message.serialize();
}

/*
/// @brief Function to serialize ROS message to UDP packet string
/// @tparam RosMsgType  Ros message type tp be serializaed
/// @param  ros_msg ROS message to be serialized
/// @return Serialized ROS message as string
template<typename RosMsgType>
std::string BCNode::serialize_ros_msg(const RosMsgType ros_msg)
{
    const rosidl_message_type_support_t* ts
        = rosidl_typesupport_cpp::get_message_type_support_handle<RosMsgType>();
    rmw_ret_t ret = rmw_serialize(&ros_msg, ts, &serialized_msg_);

    if (ret != RMW_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "Failed to serialize broadcast message");
        return {};
    }
    RCLCPP_DEBUG(get_logger(), "Serialized payload size: %ld Bytes", serialized_msg_.buffer_length);
    return std::string((char*) serialized_msg_.buffer, serialized_msg_.buffer_length);
}
*/

/// @brief Function to serialize Trajectory message to UDP packet string
///
/// @return Serialized trajectroy as string
std::string BCNode::get_serialized_trajectory()
{
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

/// @brief Function to convert received message to trajectory message
///
/// @tparam     T           type used in BroadcastMessage path points (double, float, std::int16_t)
/// @param[in]  msg         Received UDP message without header
/// @param[out] trajectory  Trajectory message
/// @return true if message conversion to trajectory was successful
template<typename T>
bool BCNode::get_trajectory_from_msg(const std::string& msg,
                                     fognav_msgs::msg::Trajectory::UniquePtr& trajectory)
{
    BroadcastMessagePoint<T> received;
    if (msg.size() != received.length())
    {
        RCLCPP_WARN(get_logger(),
                    "Received unexpected message size %ld B (expected: %ld B) from IP: %s",
                    msg.size(),
                    received.length(),
                    inet_ntoa(recv_addr_.sin_addr));
        return false;
    }
    received.deserialize(msg);
    received.to_rosmsg(trajectory);
    return true;
}

/// @brief Function to convert received message to trajectory message
///
/// @param[in]      msg         Received UDP message without header
/// @param[out]     trajectory  Trajectory message
/// @return true if message conversion to trajectory was successful
bool BCNode::get_trajectory_from_msg(const std::string& msg,
                                     fognav_msgs::msg::Trajectory::UniquePtr& trajectory)
{
    BroadcastMessageMin received;
    if (msg.size() != received.length())
    {
        RCLCPP_WARN(get_logger(),
                    "Received unexpected message size %ld B (expected: %ld B) from IP: %s",
                    msg.size(),
                    received.length(),
                    inet_ntoa(recv_addr_.sin_addr));
        return false;
    }
    received.deserialize(msg);
    received.to_rosmsg(trajectory);
    return true;
}

/// @brief Function to deserialize UDP packet string to Trajectory message
///
/// @param[in]  msg         Message to be deserialized
/// @param[out] trajectory  Deserialized trajectory (or nullptr)
/// @return true if deserialization was successful
bool BCNode::deserialize_trajectory(const std::string& msg,
                                    fognav_msgs::msg::Trajectory::UniquePtr& trajectory)
{
    if (msg.size() < kSerializedTrajectoryMinSize || msg.size() > kSerializedTrajectoryMaxSize)
    {
        RCLCPP_WARN(get_logger(),
                    "Received unexpected message size %ld B, expected: [%ld .. %ld] B from IP: %s",
                    msg.size(),
                    kSerializedTrajectoryMinSize,
                    kSerializedTrajectoryMaxSize,
                    inet_ntoa(recv_addr_.sin_addr));
        return false;
    }
    memcpy(serialized_in_msg_.buffer, msg.data(), msg.size());
    serialized_in_msg_.buffer_length = msg.size();
    auto ret = rmw_deserialize(&serialized_in_msg_, trajectory_ts_, trajectory.get());
    if (ret != RMW_RET_OK)
    {
        RCLCPP_ERROR(get_logger(), "failed to deserialize serialized message.");
        return false;
    }
    return true;
}

/// @brief Function to send UDP packet
///
/// @param[in]  msg Message to be sent
/// @return true if send was successful
bool BCNode::send_udp(const std::string& msg)
{
    if (!socket_ || !send_addr_.sin_addr.s_addr || !send_addr_.sin_port)
    {
        RCLCPP_ERROR(get_logger(), "UDP sending failed - no socket initialized");
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
    // Check whether we have fresh enough timestamp
    if (!validate_stamp(*this, trajectory_.header.stamp, max_age_))
    {
        /// @todo Do some security signalling...
        return;
    }

    HeaderV0 header;
    header.version = 0;
    header.ros_serialization = 0;
    header.signature = sign_messages_ ? 1 : 0;
    header.encryption = encrypt_messages_ ? 1 : 0;

    // Serialize trajectory
    std::string message;
    if (serialization_method_ == 0)
    {
        message = get_msg_from_trajectory<double>();
        header.message_type = 0;
    }
    else if (serialization_method_ == 1)
    {
        message = get_msg_from_trajectory<float>();
        header.message_type = 1;
    }
    else if (serialization_method_ == 2)
    {
        message = get_msg_from_trajectory<std::int16_t>();
        header.message_type = 2;
    }
    else if (serialization_method_ == 3)
    {
        message = get_msg_from_trajectory();
        header.message_type = 3;
    }
    else
    {
        message = get_serialized_trajectory(); // serialize_ros_msg(trajectory_);
        header.ros_serialization = 1;
        header.message_type = 0;
    }
    RCLCPP_DEBUG(get_logger(), "Serialized message length: %ld Bytes", message.size());

    // sign
    std::string signed_msg{sign(message)};
    RCLCPP_DEBUG(get_logger(), "Signed message length: %ld Bytes", signed_msg.size());

    // encrypt
    std::string encrypted_msg{encrypt(signed_msg)};

    // send the message
    send_udp(header.serialize() + encrypted_msg);
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
            RCLCPP_DEBUG(get_logger(), "recvfrom - No more data: %s", strerror(errno));
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

        /// @todo Discuss if and how we want to limit message flooding. If we just block we might
        /// loose the real messages from IP.
        if (++ip_addr_count_[recv_addr_.sin_addr.s_addr] > kMaxAllowedMessages)
        {
            RCLCPP_WARN_ONCE(get_logger(),
                             "Received too many messages (> %ld) from IP:%s. Blocking further "
                             "reception from this IP temporarily.",
                             kMaxAllowedMessages,
                             inet_ntoa(recv_addr_.sin_addr));
            /// @todo Do some security signalling...
            continue;
        }

        // Parse header
        HeaderV0 header;
        if (!header.parse(received_str))
        {
            continue;
        }
        if (print_received_message_)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Header" << header);
        }

        // Check version
        if (header.version > 0)
        {
            RCLCPP_ERROR(get_logger(),
                         "Received unsupported message version %d from IP:%s",
                         header.version,
                         inet_ntoa(recv_addr_.sin_addr));
            continue;
        }

        // Remove header from message
        std::string signed_msg{received_str.substr(1)};

        // Decrypt message if needed
        if (header.encryption)
        {
            signed_msg = decrypt(signed_msg);
        }

        // Remove possible signature from message
        std::string msg(signed_msg);
        if (header.signature)
        {
            msg.resize(msg.size() - kSignatureSize);
        }

        // Deserialize message to trajectory
        bool deserialize_successful{false};
        auto trajectory = std::make_unique<fognav_msgs::msg::Trajectory>();
        if (!header.ros_serialization)
        {
            if (header.message_type == 0)
            {
                if (get_trajectory_from_msg<double>(msg, trajectory))
                {
                    deserialize_successful = true;
                }
            }
            else if (header.message_type == 1)
            {
                if (get_trajectory_from_msg<float>(msg, trajectory))
                {
                    deserialize_successful = true;
                }
            }
            else if (header.message_type == 2)
            {
                if (get_trajectory_from_msg<std::int16_t>(msg, trajectory))
                {
                    deserialize_successful = true;
                }
            }
            else
            {
                if (get_trajectory_from_msg(msg, trajectory))
                {
                    deserialize_successful = true;
                }
            }
        }
        else // Deserialize ROS message from UDP packet
        {
            if (header.message_type == 0) // Serialize Trajectory.msg
            {
                if (deserialize_trajectory(msg, trajectory))
                {
                    deserialize_successful = true;
                }
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Unknown message type.");
            }
        }
        if (!trajectory || !deserialize_successful)
        {
            RCLCPP_ERROR(get_logger(), "Failed to deserialize serialized message.");
            continue;
        }

        if (print_received_message_
            && (!header.ros_serialization || (header.ros_serialization && header.message_type == 0)))
        {
            print_trajectory(trajectory, header.message_type);
        }

        // Check message age
        if (!validate_stamp(*this, trajectory->header.stamp, max_age_))
        {
            /// @todo Do some security signalling...
            continue;
        }

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
        // header has signature AND (all sigantures need to be verified OR drone has not been observed
        // earlier OR (drone has been observed earlier AND there is enough time from last check))
        if (header.signature
            && (verify_all_signatures_ || !observed_senders_times_.count(trajectory->droneid)
                || (observed_senders_times_.count(trajectory->droneid)
                    && ((now() - observed_senders_times_[trajectory->droneid]).seconds()
                        > signature_check_interval_))))
        {
            if (!verify_signature(signed_msg, trajectory->droneid))
            {
                continue;
            }
            observed_senders_times_[trajectory->droneid] = now();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Signature verification skipped.");
        }

        // Relay message forward
        pub_->publish(std::move(trajectory));
    }
}

/// @brief Print received message
///
/// ┌───────DroneID──────┬Priority┬Stamp.sec─┬Stamp.nsec┬─Dat.lat.─┬─Dat.lon.─┬─Dat.alt.─┐
/// │12345678901234567890│  32678 │2147483647│2147483647│-79.123456│-179.12345│ 10000.00 │
/// └────────20/20───────┴────2───┴─────4────┴─────4────┴─────8────┴─────8────┴─────8────┘
/// Path
/// ┌───x0───┬───x1───┬───x2───┬───x3───┬───x4───┬───x5───┬───x6───┬───x7───┬───x8───┬───x9───┐
/// │-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│
/// ├───y0───┼───y1───┼───y2───┼───y3───┼───y4───┼───y5───┼───y6───┼───y7───┼───y8───┼───y9───┤
/// │-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│
/// ├───z0───┼───z1───┼───z2───┼───z3───┼───z4───┼───z5───┼───z6───┼───z7───┼───z8───┼───z9───┤
/// │-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│-1234.50│
/// └────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
///
/// @param[in] trajectory   Trajectory message what is formatted and printed out
/// @param[in] message_type Original received message type
void BCNode::print_trajectory(const fognav_msgs::msg::Trajectory::UniquePtr& trajectory,
                              const int message_type)
{
    char paths[1500];
    int paths_len{0};
    paths_len += sprintf(paths + paths_len, "%s path\n┌", trajectory->droneid.c_str());
    for (int i = 0; i < 10; ++i)
    {
        paths_len += sprintf(paths + paths_len, "───x%d───%s", i, (i < 9 ? "┬" : "┐\n"));
    }
    for (int i = 0; i < 10; ++i)
    {
        paths_len += sprintf(paths + paths_len, "|% 8.1f", trajectory->path[i].x);
    }
    paths_len += sprintf(paths + paths_len, "|\n├");
    for (int i = 0; i < 10; ++i)
    {
        paths_len += sprintf(paths + paths_len, "───y%d───%s", i, (i < 9 ? "┼" : "┤\n"));
    }
    for (int i = 0; i < 10; ++i)
    {
        paths_len += sprintf(paths + paths_len, "|% 8.1f", trajectory->path[i].y);
    }
    paths_len += sprintf(paths + paths_len, "|\n├");
    for (int i = 0; i < 10; ++i)
    {
        paths_len += sprintf(paths + paths_len, "───z%d───%s", i, (i < 9 ? "┼" : "┤\n"));
    }
    for (int i = 0; i < 10; ++i)
    {
        paths_len += sprintf(paths + paths_len, "|% 8.1f", trajectory->path[i].z);
    }
    paths_len += sprintf(paths + paths_len,
                         "|\n└────────┴────────┴────────┴────────┴────────┴────────┴────────┴─────"
                         "───┴────────┴────────┘");

    std::string sizes_str;
    sizes_str = (message_type == 3
                     ? "1───┴─────4────┴─────4────┴─────4────┴─────4────┴─────2────┘\n"
                     : "2───┴─────4────┴─────4────┴─────8────┴─────8────┴─────8────┘\n");
    RCLCPP_INFO_STREAM(get_logger(),
                       "\n "
                       "┌───────DroneID──────┬Priority┬Stamp.sec─┬Stamp.nsec┬─Dat.lat."
                       "─┬─Dat.lon.─┬─Dat.alt.─┐"
                       "\n │"
                           << std::setw(20) << std::left << trajectory->droneid << "│"
                           << std::setw(6) << std::right << trajectory->priority << "  │"
                           << std::setw(10) << trajectory->header.stamp.sec << "│" << std::setw(10)
                           << trajectory->header.stamp.nanosec << "│" << std::setw(10)
                           << trajectory->datum.latitude << "│" << std::setw(10)
                           << trajectory->datum.longitude << "|" << std::setw(10)
                           << trajectory->datum.altitude << "│\n └────────" << std::setw(2)
                           << trajectory->droneid.length() << "/20───────┴────" << sizes_str
                           << paths);
}

/// @brief Function to sign message
///
/// @param[in]  msg Original message to be signed
/// @return signed message (Original message + signature)
std::string BCNode::sign(const std::string& msg)
{
    if (!sign_messages_)
    {
        return msg;
    }
    else
    {
        return (msg + "ABCD");
    }
}

/// @brief Function to verify message signature
///
/// @param[in]  msg Signed message to be verified
/// @param[in]  droneid DroneID of claimed sender
/// @return true if signature was considered valid
bool BCNode::verify_signature(const std::string& msg, const std::string& droneid)
{
    if (msg.substr(msg.length() - 4) == "ABCD")
    {
        RCLCPP_INFO_STREAM(get_logger(),
                           droneid << ": signature verified: " << msg.substr(msg.length() - 4));
        return true;
    }
    RCLCPP_WARN_STREAM(get_logger(),
                       droneid << ": signature verification failed: "
                               << msg.substr(msg.length() - 4));
    /// @todo Do some security signalling...
    return false;
}

} // namespace bc_node
