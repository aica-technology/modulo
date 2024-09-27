#pragma once

#include <rclcpp/parameter.hpp>
#include <state_representation/parameters/Parameter.hpp>

/**
 * @namespace modulo_core::translators
 * @brief Modulo Core translation module for converting between ROS2 and state_representation data types.
 */
namespace modulo_core::translators {

/**
 * @brief Copy the value of one parameter interface into another.
 * @details When referencing a Parameter instance through a ParameterInterface pointer, it is necessary
 * to know the parameter type to get or set the parameter value. Sometimes it is desirable to update the
 * parameter value from another compatible ParameterInterface, while still preserving the reference
 * to the original instance. This helper function calls the getters and setters of the appropriate parameter type
 * to modify the value of the parameter instance while preserving the reference of the original pointer.
 * @param source_parameter The ParameterInterface with a value to copy
 * @param parameter The destination ParameterInterface to be updated
 * @throws modulo_core::exceptions::ParameterTranslationException if the copying failed
 */
void copy_parameter_value(
    const std::shared_ptr<const state_representation::ParameterInterface>& source_parameter,
    const std::shared_ptr<state_representation::ParameterInterface>& parameter);

/**
 * @brief Write a ROS Parameter from a ParameterInterface pointer.
 * @param parameter The ParameterInterface pointer with a name and value
 * @throws modulo_core::exceptions::ParameterTranslationException if the ROS parameter could not be written
 * @return A new ROS Parameter object
 */
rclcpp::Parameter write_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter);

/**
 * @brief Create a new ParameterInterface from a ROS Parameter object.
 * @param ros_parameter The ROS parameter object to read
 * @throws modulo_core::exceptions::ParameterTranslationException if the ROS parameter could not be read
 * @return A new ParameterInterface pointer
 */
std::shared_ptr<state_representation::ParameterInterface> read_parameter(const rclcpp::Parameter& ros_parameter);

/**
 * @brief Update the parameter value of a ParameterInterface from a ROS Parameter object only if the two parameters have
 * the same name and the ROS Parameter value can be interpreted as a ParameterInterface value.
 * @param ros_parameter The ROS parameter object to read
 * @param parameter An existing ParameterInterface pointer
 * @throws modulo_core::exceptions::ParameterTranslationException if the ROS parameter could not be read
 * @return A new ParameterInterface pointer with the updated value
 */
std::shared_ptr<state_representation::ParameterInterface> read_parameter_const(
    const rclcpp::Parameter& ros_parameter,
    const std::shared_ptr<const state_representation::ParameterInterface>& parameter);

/**
 * @brief Update the parameter value of a ParameterInterface from a ROS Parameter object.
 * @details The destination ParameterInterface must have a compatible parameter name and type.
 * @param ros_parameter The ROS parameter object to read
 * @param parameter An existing ParameterInterface pointer
 * @throws modulo_core::exceptions::ParameterTranslationException if the ROS parameter could not be read
 */
void read_parameter(
    const rclcpp::Parameter& ros_parameter, const std::shared_ptr<state_representation::ParameterInterface>& parameter);

/**
 * @brief Given a state representation parameter type, get the corresponding ROS parameter type.
 * @param parameter_type The state representation parameter type
 * @return The corresponding ROS parameter type
 */
rclcpp::ParameterType get_ros_parameter_type(const state_representation::ParameterType& parameter_type);

}// namespace modulo_core::translators
