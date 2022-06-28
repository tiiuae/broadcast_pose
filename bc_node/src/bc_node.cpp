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

    max_age_ = get_gated_param(*this, "broadcast_trajectory_max_age_s", 0.5, 0.0, 60.0);

    const std::string trajectory_topic_in{
        get_param(*this, "input_trajectory_topic", std::string("navigation/trajectory"))};

    const std::string trajectory_topic_out{
        get_param(*this, "output_trajectory_topic", std::string("bc_node/trajectories"))};

    drone_id_ = get_param(*this, "drone_id", std::string("Testi_drone_1"));
    serialization_method_ = get_param(*this, "serialization_method", 0);

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

    // Signature check timer
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
                "- input_trajectory_topic: %s\n"
                "- output_trajectory_topic: %s\n"
                "- broadcast_ip_adress: %s\n"
                "- broadcast_port: %d\n"
                "- broadcast_interval: %5.2f s\n"
                "- immediate_broadcast: %s\n"
                "- signature_check_interval: %5.2f s\n"
                "- verify_all_signatures: %s\n"
                "- serialization_method: %d\n"
                "- BroadcastMessage size: [%ld .. %ld] Bytes",
                trajectory_topic_in.c_str(),
                trajectory_topic_out.c_str(),
                broadcast_ip_.c_str(),
                broadcast_port_,
                broadcast_interval.count(),
                immediate_broadcast_ ? "true" : "false",
                signature_check_interval_,
                verify_all_signatures_ ? "true" : "false",
                serialization_method_,
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
    // Clear ip address counters
    ip_addr_count_.clear();
    /// @todo: Do we need to adjust t
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
/// @param[in]  msg Message to be deserialized
/// @param[out] trajectory Deserialized trajectory (or nullptr)
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
    if (!validate_age(*this, trajectory_.header.stamp, max_age_))
    {
        /// @todo Do some security signalling...
        return;
    }
    if (!validate_droneid(*this, trajectory_.droneid, drone_id_))
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
    send_udp(header.to_string() + encrypted_msg);
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
        header.from_string(received_str);

        // Print header
        RCLCPP_INFO(
            get_logger(),
            "Header:\n- Version: %d\n- ROS Serialization: %s\n- Message Type: %d\n- Signed: "
            "%s\n- Encrypted: %s",
            header.version,
            (header.ros_serialization ? "True" : "False"),
            header.message_type,
            (header.signature ? "True" : "False"),
            (header.encryption ? "True" : "False"));

        // Check version
        if (header.version > 0)
        {
            RCLCPP_ERROR(get_logger(),
                         "Received unsupported message version %d from IP:%s",
                         header.version,
                         inet_ntoa(recv_addr_.sin_addr));
            return;
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
        else
        {
            // Deserialize trajectory from UDP packet
            if (deserialize_trajectory(msg, trajectory))
            {
                deserialize_successful = true;
            }
        }
        if (!trajectory || !deserialize_successful)
        {
            RCLCPP_ERROR(get_logger(), "failed to deserialize serialized message. received nullptr");
            continue;
        }

        // Check message age
        if (!validate_age(*this, trajectory->header.stamp, max_age_))
        {
            /// @todo Do some security signalling...
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
        if (header.signature && !observed_senders_times_.count(trajectory->droneid))
        {
            if (!verify_signature(signed_msg, trajectory->droneid))
            {
                continue;
            }
            observed_senders_times_[trajectory->droneid] = now();
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Signature verification skipped.");
        }

        // Relay message forward
        pub_->publish(std::move(trajectory));
    }
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
bool BCNode::verify_signature(const std::string& msg, std::string droneid)
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
