#ifndef VALIDATION_HPP_
#define VALIDATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

/// @brief Message timestamp age validation function
///
/// @param[in] node     Node handle
/// @param[in] stamp    Timestamp to be validated
/// @param[in] max_age  max alloved age in seconds
/// @return true if validation is passed
inline bool validate_stamp(const rclcpp::Node& node,
                           const builtin_interfaces::msg::Time stamp,
                           const double max_age = 0.5)
{
    const double age{(node.now() - stamp).seconds()};
    if (std::fabs(age) > max_age)
    {
        RCLCPP_WARN(node.get_logger(),
                    "Timestamp too old (or new) for publishing: %.2f s (Limit: %.2f s)",
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
/// @param[in] node         Node handle
/// @param[in] drone_id_1   Drone 1 ID
/// @param[in] drone_id_2   Drone 2 ID
/// @return true if validation is passed
inline bool validate_droneid(const rclcpp::Node& node,
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

/// @brief GeoPoint validation function
///
/// @param[in] node     Node handle
/// @param[in] geopoint GeoPoint msg to be validated
/// @return true if validation is passed
inline bool validate_geopoint(const rclcpp::Node& node,
                              const geographic_msgs::msg::GeoPoint& geopoint,
                              const bool logging = true)
{
    if (std::fabs(geopoint.latitude) > 90 || std::fabs(geopoint.longitude > 180))
    {
        if (logging)
        {
            RCLCPP_WARN_STREAM(node.get_logger(),
                               "Geopoint out of bounds. lat: "
                                   << geopoint.latitude << " [-90, 90], lon: " << geopoint.longitude
                                   << " [-180, 180]");
        }
        return false;
    }
    return true;
}

#endif
