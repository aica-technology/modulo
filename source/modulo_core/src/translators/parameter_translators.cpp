#include "modulo_core/translators/parameter_translators.hpp"

#include <clproto.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "modulo_core/exceptions/ParameterTranslationException.hpp"

using namespace state_representation;

namespace modulo_core::translators {

void copy_parameter_value(
    const std::shared_ptr<const ParameterInterface>& source_parameter,
    const std::shared_ptr<ParameterInterface>& parameter
) {
  if (source_parameter->get_parameter_type() != parameter->get_parameter_type()) {
    throw exceptions::ParameterTranslationException(
        "Source parameter " + source_parameter->get_name()
            + " to be copied does not have the same type as destination parameter " + parameter->get_name());
  }
  switch (source_parameter->get_parameter_type()) {
    case ParameterType::BOOL:
      parameter->set_parameter_value(source_parameter->get_parameter_value<bool>());
      return;
    case ParameterType::BOOL_ARRAY:
      parameter->set_parameter_value(source_parameter->get_parameter_value<std::vector<bool>>());
      return;
    case ParameterType::INT:
      parameter->set_parameter_value(source_parameter->get_parameter_value<int>());
      return;
    case ParameterType::INT_ARRAY:
      parameter->set_parameter_value(source_parameter->get_parameter_value<std::vector<int>>());
      return;
    case ParameterType::DOUBLE:
      parameter->set_parameter_value(source_parameter->get_parameter_value<double>());
      return;
    case ParameterType::DOUBLE_ARRAY:
      parameter->set_parameter_value(source_parameter->get_parameter_value<std::vector<double>>());
      return;
    case ParameterType::STRING:
      parameter->set_parameter_value(source_parameter->get_parameter_value<std::string>());
      return;
    case ParameterType::STRING_ARRAY:
      parameter->set_parameter_value(source_parameter->get_parameter_value<std::vector<std::string>>());
      return;
    case ParameterType::VECTOR:
      parameter->set_parameter_value(source_parameter->get_parameter_value<Eigen::VectorXd>());
      return;
    case ParameterType::MATRIX:
      parameter->set_parameter_value(source_parameter->get_parameter_value<Eigen::MatrixXd>());
      return;
    case ParameterType::STATE:
      if (source_parameter->get_parameter_state_type() != parameter->get_parameter_state_type()) {
        throw exceptions::ParameterTranslationException(
            "Source parameter " + source_parameter->get_name()
                + " to be copied does not have the same parameter state type as destination parameter "
                + parameter->get_name());
      }
      switch (source_parameter->get_parameter_state_type()) {
        case StateType::CARTESIAN_STATE:
          parameter->set_parameter_value(source_parameter->get_parameter_value<CartesianState>());
          return;
        case StateType::CARTESIAN_POSE:
          parameter->set_parameter_value(source_parameter->get_parameter_value<CartesianPose>());
          return;
        case StateType::JOINT_STATE:
          parameter->set_parameter_value(source_parameter->get_parameter_value<JointState>());
          return;
        case StateType::JOINT_POSITIONS:
          parameter->set_parameter_value(source_parameter->get_parameter_value<JointPositions>());
          return;
        default:
          break;
      }
      break;
    default:
      break;
  }
  throw exceptions::ParameterTranslationException(
      "Could not copy the value from source parameter " + source_parameter->get_name() + " into parameter "
          + parameter->get_name());
}

rclcpp::Parameter write_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (parameter->is_empty()) {
    return rclcpp::Parameter(parameter->get_name());
  }
  switch (parameter->get_parameter_type()) {
    case ParameterType::BOOL:
      return {parameter->get_name(), parameter->get_parameter_value<bool>()};
    case ParameterType::BOOL_ARRAY:
      return {parameter->get_name(), parameter->get_parameter_value<std::vector<bool>>()};
    case ParameterType::INT:
      return {parameter->get_name(), parameter->get_parameter_value<int>()};
    case ParameterType::INT_ARRAY:
      return {parameter->get_name(), parameter->get_parameter_value<std::vector<int>>()};
    case ParameterType::DOUBLE:
      return {parameter->get_name(), parameter->get_parameter_value<double>()};
    case ParameterType::DOUBLE_ARRAY:
      return {parameter->get_name(), parameter->get_parameter_value<std::vector<double>>()};
    case ParameterType::STRING:
      return {parameter->get_name(), parameter->get_parameter_value<std::string>()};
    case ParameterType::STRING_ARRAY:
      return {parameter->get_name(), parameter->get_parameter_value<std::vector<std::string>>()};
    case ParameterType::STATE: {
      switch (parameter->get_parameter_state_type()) {
        case StateType::CARTESIAN_STATE:
          return {parameter->get_name(), clproto::to_json(parameter->get_parameter_value<CartesianState>())};
        case StateType::CARTESIAN_POSE:
          return {parameter->get_name(), clproto::to_json(parameter->get_parameter_value<CartesianPose>())};
        case StateType::JOINT_STATE:
          return {parameter->get_name(), clproto::to_json(parameter->get_parameter_value<JointState>())};
        case StateType::JOINT_POSITIONS:
          return {parameter->get_name(), clproto::to_json(parameter->get_parameter_value<JointPositions>())};
        default:
          break;
      }
      break;
    }
    case ParameterType::VECTOR: {
      auto eigen_vector = parameter->get_parameter_value<Eigen::VectorXd>();
      std::vector<double> vec(eigen_vector.data(), eigen_vector.data() + eigen_vector.size());
      return {parameter->get_name(), vec};
    }
    case ParameterType::MATRIX: {
      auto eigen_matrix = parameter->get_parameter_value<Eigen::MatrixXd>();
      std::vector<double> vec(eigen_matrix.data(), eigen_matrix.data() + eigen_matrix.size());
      return {parameter->get_name(), vec};
    }
    default:
      break;
  }
  throw exceptions::ParameterTranslationException("Parameter " + parameter->get_name() + " could not be written!");
}

std::shared_ptr<ParameterInterface> read_parameter(const rclcpp::Parameter& parameter) {
  switch (parameter.get_type()) {
    case rclcpp::PARAMETER_BOOL:
      return make_shared_parameter(parameter.get_name(), parameter.as_bool());
    case rclcpp::PARAMETER_BOOL_ARRAY:
      return make_shared_parameter(parameter.get_name(), parameter.as_bool_array());
    case rclcpp::PARAMETER_INTEGER:
      return make_shared_parameter(parameter.get_name(), static_cast<int>(parameter.as_int()));
    case rclcpp::PARAMETER_INTEGER_ARRAY: {
      auto array = parameter.as_integer_array();
      std::vector<int> int_array(array.begin(), array.end());
      return make_shared_parameter(parameter.get_name(), int_array);
    }
    case rclcpp::PARAMETER_DOUBLE:
      return make_shared_parameter(parameter.get_name(), parameter.as_double());
    case rclcpp::PARAMETER_DOUBLE_ARRAY:
      return make_shared_parameter(parameter.get_name(), parameter.as_double_array());
    case rclcpp::PARAMETER_STRING_ARRAY:
      return make_shared_parameter<std::vector<std::string>>(parameter.get_name(), parameter.as_string_array());
    case rclcpp::PARAMETER_STRING: {
      // TODO: consider dedicated clproto::decode<std::shared_ptr<ParameterInterface>>(msg) specialization in library
      std::string encoding;
      try {
        encoding = clproto::from_json(parameter.as_string());
      } catch (const clproto::JsonParsingException&) {}
      if (!clproto::is_valid(encoding)) {
        return make_shared_parameter<std::string>(parameter.get_name(), parameter.as_string());
      }
      switch (clproto::check_message_type(encoding)) {
        case clproto::CARTESIAN_STATE_MESSAGE:
          return make_shared_parameter<CartesianState>(parameter.get_name(), clproto::decode<CartesianState>(encoding));
        case clproto::CARTESIAN_POSE_MESSAGE:
          return make_shared_parameter<CartesianPose>(parameter.get_name(), clproto::decode<CartesianPose>(encoding));
        case clproto::JOINT_STATE_MESSAGE:
          return make_shared_parameter<JointState>(parameter.get_name(), clproto::decode<JointState>(encoding));
        case clproto::JOINT_POSITIONS_MESSAGE:
          return make_shared_parameter<JointPositions>(parameter.get_name(), clproto::decode<JointPositions>(encoding));
        default:
          throw exceptions::ParameterTranslationException(
              "Parameter " + parameter.get_name() + " has an unsupported encoded message type");
      }
    }
    case rclcpp::PARAMETER_BYTE_ARRAY:
      // TODO: try clproto decode, re-use logic from above
      throw exceptions::ParameterTranslationException("Parameter byte arrays are not currently supported.");
    default:
      break;
  }
  throw exceptions::ParameterTranslationException("Parameter " + parameter.get_name() + " could not be read!");
}

std::shared_ptr<ParameterInterface> read_parameter_const(
    const rclcpp::Parameter& ros_parameter, const std::shared_ptr<const ParameterInterface>& parameter
) {
  if (ros_parameter.get_name() != parameter->get_name()) {
    throw exceptions::ParameterTranslationException(
        "The ROS parameter " + ros_parameter.get_name()
            + " to be read does not have the same name as the reference parameter " + parameter->get_name());
  }
  if (ros_parameter.get_type() == rclcpp::PARAMETER_NOT_SET) {
    return make_shared_parameter_interface(
        parameter->get_name(), parameter->get_parameter_type(), parameter->get_parameter_state_type());
  }
  auto new_parameter = read_parameter(ros_parameter);
  if (new_parameter->get_parameter_type() == parameter->get_parameter_type()) {
    if (new_parameter->get_parameter_state_type() != parameter->get_parameter_state_type()) {
      throw exceptions::ParameterTranslationException(
          "The received state parameter " + ros_parameter.get_name()
              + " does not have the same state type as the reference parameter");
    }
    return new_parameter;
  }
  switch (new_parameter->get_parameter_type()) {
    case ParameterType::DOUBLE_ARRAY: {
      auto value = new_parameter->get_parameter_value<std::vector<double>>();
      switch (parameter->get_parameter_type()) {
        case ParameterType::VECTOR: {
          Eigen::VectorXd vector = Eigen::Map<Eigen::VectorXd>(value.data(), static_cast<Eigen::Index>(value.size()));
          new_parameter = make_shared_parameter(parameter->get_name(), vector);
          break;
        }
        case ParameterType::MATRIX: {
          auto matrix = parameter->get_parameter_value<Eigen::MatrixXd>();
          if (static_cast<std::size_t>(matrix.size()) != value.size()) {
            throw exceptions::ParameterTranslationException(
                "The ROS parameter " + ros_parameter.get_name() + " with type double array has size "
                    + std::to_string(value.size()) + " while the reference parameter matrix " + parameter->get_name()
                    + " has size " + std::to_string(matrix.size()));
          }
          matrix = Eigen::Map<Eigen::MatrixXd>(value.data(), matrix.rows(), matrix.cols());
          new_parameter = make_shared_parameter(parameter->get_name(), matrix);
          break;
        }
        default:
          throw exceptions::ParameterTranslationException(
              "The ROS parameter " + ros_parameter.get_name()
                  + " with type double array cannot be interpreted by reference parameter " + parameter->get_name()
                  + " (type code " + std::to_string(static_cast<int>(parameter->get_parameter_type())) + ")");
      }
      break;
    }
    default:
      throw exceptions::ParameterTranslationException(
          "Incompatible parameter type encountered while reading parameter '" + parameter->get_name() + "'.");
  }
  return new_parameter;
}

void read_parameter(
    const rclcpp::Parameter& ros_parameter, const std::shared_ptr<ParameterInterface>& parameter
) {
  auto new_parameter = read_parameter_const(ros_parameter, parameter);
  copy_parameter_value(new_parameter, parameter);
}

rclcpp::ParameterType get_ros_parameter_type(const ParameterType& parameter_type) {
  switch (parameter_type) {
    case ParameterType::BOOL:
      return rclcpp::ParameterType::PARAMETER_BOOL;
    case ParameterType::BOOL_ARRAY:
      return rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
    case ParameterType::INT:
      return rclcpp::ParameterType::PARAMETER_INTEGER;
    case ParameterType::INT_ARRAY:
      return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    case ParameterType::DOUBLE:
      return rclcpp::ParameterType::PARAMETER_DOUBLE;
    case ParameterType::DOUBLE_ARRAY:
    case ParameterType::VECTOR:
    case ParameterType::MATRIX:
      return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
    case ParameterType::STRING:
      return rclcpp::ParameterType::PARAMETER_STRING;
    case ParameterType::STRING_ARRAY:
    case ParameterType::STATE:
      return rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
    default:
      return rclcpp::ParameterType::PARAMETER_NOT_SET;
  }
}
}// namespace modulo_core::translators
