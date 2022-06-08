#ifndef PARAM_CHECKER_HPP_
#define PARAM_CHECKER_HPP_

#ifdef NDEBUG
#define ASSERT(condition, message) (void) 0
#else
#define ASSERT(condition, message) \
    (!(condition)) ? (std::cerr << "Assertion failed: (" << #condition << "), " \
                                << "function " << __FUNCTION__ << ", file " << __FILE__ \
                                << ", line " << __LINE__ << "." << std::endl \
                                << message << std::endl, \
                      abort(), \
                      0) \
                   : 1
#endif

#include <optional>
#include <rclcpp/logging.hpp>

/// @brief Value gating function.
///
/// @param[in] node             Node handle
/// @param[in] name             Parameter name
/// @param[in] value            Value to be checked
/// @param[in] min_value        Minimum parameter value
/// @param[in] max_value        Maximum parameter value
/// @param[in] default_value    Optional default parameter value
/// @return Value adjusted inside [min_value/max_value] gate. Default value is returned for out of
/// bound values if defined otherwise: <min_value returns min_value and >max_value returns max_value.
template<typename ParameterT>
inline ParameterT gate_value(rclcpp::Node& node,
                             const std::string& name,
                             const ParameterT& value,
                             const ParameterT& min_value,
                             const ParameterT& max_value,
                             const std::optional<ParameterT>& default_value = std::nullopt)
{
    if (!default_value)
    {
        ASSERT(min_value <= max_value,
               "\'" << name << "\' min value \'" << min_value << "\' bigger than max value \'"
                    << max_value << "\'");
    }
    else
    {
        ASSERT(min_value <= *default_value && *default_value <= max_value,
               "\'" << name << "\' default value \'" << *default_value << "\' out of bounds ["
                    << min_value << ", " << max_value << "]");
    }
    if (value < min_value || value > max_value)
    {
        const ParameterT adjusted_value{
            !default_value ? std::max(std::min(value, max_value), min_value) : *default_value};
        RCLCPP_WARN_STREAM(rclcpp::get_logger(std::string(node.get_name()) + "-params"),
                           "Parameter \"" << name << "\" has value " << value
                                          << " which is outside the valid range: [" << min_value
                                          << ", " << max_value << "]."
                                          << " Value adjusted to"
                                          << (!default_value ? "" : " default value") << ": "
                                          << adjusted_value);
        return adjusted_value;
    }
    return value;
}

/// @brief Declare parameter, instead of throwing InvalidParameterTypeException use the default value
///
/// @tparam Parameter type
/// @param[in] node             Node handle
/// @param[in] name             Parameter name
/// @param[in] default_value    Default parameter value
/// @return resolved parameter value
template<typename ParameterT>
inline ParameterT get_param(rclcpp::Node& node,
                            const std::string& name,
                            const ParameterT& default_value)
{
    try
    {
        ParameterT value{};
        if (!node.get_parameter(name, value))
        {
            value = node.declare_parameter(name, default_value);
        }
        else
        {
            RCLCPP_INFO(node.get_logger(), "Parameter \'%s\' earlier declared.", name.c_str());
        }
        return value;
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException& ex)
    {
        RCLCPP_WARN_STREAM(node.get_logger(),
                           ex.what() << ". Using default parameter value " << default_value);

        return default_value;
    }
}

/// @brief Get parameter value inside given min-max gate. Out of gate parameter value causes
/// default_value to be returned
///
/// @tparam Parameter type
/// @param[in] node             Node handle
/// @param[in] name             Parameter name
/// @param[in] default_value    Default parameter value
/// @param[in] min_value        Min parameter value
/// @param[in] max_value        Max parameter value
/// @return resolved parameter value
template<typename ParameterT>
inline ParameterT get_gated_param(rclcpp::Node& node,
                                  const std::string& name,
                                  const ParameterT& default_value,
                                  const ParameterT& min_value,
                                  const ParameterT& max_value)
{
    const ParameterT value{get_param(node, name, default_value)};
    return gate_value(node, name, value, min_value, max_value, std::make_optional(default_value));
}

/// @brief Try to fetch mandatory parameter. No default values allowed. In case of error return false.
///
/// @tparam Parameter type
/// @param[in]  node    Node handle
/// @param[in]  name    Parameter name
/// @param[out] value   Parameter value when succesfully resolved
/// @return true if parameter value was resolved
template<typename T>
inline bool get_mandatory_param(rclcpp::Node& node, const std::string& name, T& value)
{
    bool value_valid{false};
    try
    {
        node.declare_parameter<T>(name);
        value_valid = node.get_parameter(name, value);
        if (!value_valid)
        {
            const rclcpp::exceptions::ParameterNotDeclaredException e("Mandatory parameter \""
                                                                      + name + "\" not defined.");
            throw e;
        }
    }
    // If parmeter was not declared
    catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex)
    {
        RCLCPP_ERROR(node.get_logger(), ex.what());
    }
    // If wrong type of the parameter
    catch (const rclcpp::exceptions::InvalidParameterTypeException& ex)
    {
        RCLCPP_ERROR_STREAM(node.get_logger(), "Mandatory " << ex.what());
    }
    // Other errors
    catch (...)
    {
        RCLCPP_ERROR_STREAM(node.get_logger(),
                            "Not able to resolve mandatory parameter \"" << name << "\".");
    }
    return value_valid;
}

#endif
