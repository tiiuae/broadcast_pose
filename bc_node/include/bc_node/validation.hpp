#ifndef VALIDATION_HPP_
#define VALIDATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

/// @brief Message age validation function
///
/// @param[in] node  Node handle
/// @param[in] stamp Time stamp to be validated
/// @param[in] max_age max alloved age in seconds
inline bool validate_age(rclcpp::Node& node,
                         const builtin_interfaces::msg::Time stamp,
                         const double max_age = 0.5)
{
    const double age{(node.now() - stamp).seconds()};
    if (age > max_age)
    {
        RCLCPP_WARN(node.get_logger(),
                    "Timestamp too old for publishing: %.2f s (Limit: %.2f s)",
                    age,
                    max_age);
        return false;
    }
    else
    {
        RCLCPP_DEBUG(node.get_logger(), "Time delta %.2f seconds", age);
        return true;
    }
}

/// @brief Drone ID validation function
///
/// @param[in] node  Node handle
/// @param[in] drone_id Time stamp to be validated
/// @param[in] max_age max alloved age in seconds
inline bool validate_droneid(rclcpp::Node& node,
                             const std::string& drone_id_1,
                             const std::string& drone_id_2)
{
    if (drone_id_1 != drone_id_2)
    {
        RCLCPP_WARN_STREAM(node.get_logger(),
                           "DroneIDs mot matcing. [" << drone_id_1 << "] vs. [" << drone_id_2
                                                     << "]");
        return false;
    }
    return true;
}

#endif