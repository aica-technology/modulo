#pragma once

#include <set>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <clproto.hpp>
#include <state_representation/DigitalIOState.hpp>
#include <state_representation/AnalogIOState.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/Jacobian.hpp>

#include "modulo_core/EncodedState.hpp"
#include "modulo_core/exceptions/MessageTranslationException.hpp"

namespace modulo_core::translators {

/**
 * @brief Convert a ROS geometry_msgs::msg::Accel to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Accel& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::AccelStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::AccelStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Pose to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Pose& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::PoseStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::PoseStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Transform to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Transform& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::TransformStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::TransformStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Twist to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Twist& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::TwistStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::TwistStamped& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::Wrench to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::Wrench& message);

/**
 * @brief Convert a ROS geometry_msgs::msg::WrenchStamped to a CartesianState
 * @param state The CartesianState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::CartesianState& state, const geometry_msgs::msg::WrenchStamped& message);

/**
 * @brief Convert a ROS sensor_msgs::msg::JointState to a JointState
 * @param state The JointState to populate
 * @param message The ROS message to read from
 */
void read_message(state_representation::JointState& state, const sensor_msgs::msg::JointState& message);

/**
 * @brief Template function to convert a ROS std_msgs::msg::T to a Parameter<T>
 * @tparam T All types of parameters supported in ROS std messages
 * @tparam U All types of parameters supported in ROS std messages
 * @param state The Parameter<T> to populate
 * @param message The ROS message to read from
 */
template<typename T, typename U>
void read_message(state_representation::Parameter<T>& state, const U& message) {
  state.set_value(message.data);
}

/**
 * @brief Convert a ROS std_msgs::msg::Bool to a boolean
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(bool& state, const std_msgs::msg::Bool& message);

/**
 * @brief Convert a ROS std_msgs::msg::Float64 to a double
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(double& state, const std_msgs::msg::Float64& message);

/**
 * @brief Convert a ROS std_msgs::msg::Float64MultiArray to a vector of double
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(std::vector<double>& state, const std_msgs::msg::Float64MultiArray& message);

/**
 * @brief Convert a ROS std_msgs::msg::Int32 to an integer
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(int& state, const std_msgs::msg::Int32& message);

/**
 * @brief Convert a ROS std_msgs::msg::String to a string
 * @param state The state to populate
 * @param message The ROS message to read from
 */
void read_message(std::string& state, const std_msgs::msg::String& message);

/**
 * @brief Convert a ROS std_msgs::msg::UInt8MultiArray message to a State using protobuf decoding
 * @tparam T A state_representation::State type
 * @param state The state to populate
 * @param message The ROS message to read from
 * @throws modulo_core::exceptions::MessageTranslationException if the translation failed or is not supported.
 */
template<typename T>
inline void read_message(T& state, const EncodedState& message) {
  try {
    std::string tmp(message.data.begin(), message.data.end());
    state = clproto::decode<T>(tmp);
  } catch (const std::exception& ex) {
    throw exceptions::MessageTranslationException(ex.what());
  }
}

/**
 * @brief Safely downcast a base state pointer to a derived state pointer
 * @details This utility function asserts that result of the dynamic pointer cast is not null,
 * and throws an exception otherwise. This helps to prevent uncaught segmentation faults from
 * a null pointer dynamic cast result being passed to subsequent execution steps.
 * @tparam T The derived state type
 * @param state A base state pointer referencing a state-derived instance
 * @throws modulo_core::exceptions::MessageTranslationException if the dynamic cast results in a null pointer
 * @return The derived pointer of type T
 */
template<typename T>
inline std::shared_ptr<T> safe_dynamic_pointer_cast(std::shared_ptr<state_representation::State>& state) {
  auto derived_state_ptr = std::dynamic_pointer_cast<T>(state);
  if (derived_state_ptr == nullptr) {
    throw modulo_core::exceptions::MessageTranslationException(
        "Dynamic casting of state " + state->get_name() + " failed.");
  }
  return derived_state_ptr;
}

/**
 * @brief Update the value referenced by a state pointer from a new_state pointer
 * @details The base state and new_state pointers must reference state-derived instances of type T.
 * This function dynamically downcasts each pointer to a pointer of the derived type and assigns
 * the de-referenced value from the new_state parameter to the address of the state pointer.
 * @tparam T The derived state type
 * @param state A base state pointer referencing a state-derived instance to be modified
 * @param new_state A base state pointer referencing a state-derived instance to be copied into the state parameter
 * @throws modulo_core::exceptions::MessageTranslationException if either parameter cannot be cast to state type T
 */
template<typename T>
inline void safe_dynamic_cast(
    std::shared_ptr<state_representation::State>& state, std::shared_ptr<state_representation::State>& new_state
) {
  auto derived_state_ptr = safe_dynamic_pointer_cast<T>(state);
  auto received_state_ptr = safe_dynamic_pointer_cast<T>(new_state);
  *derived_state_ptr = *received_state_ptr;
}

/**
 * @brief Update the value referenced by a state pointer by dynamically casting and converting the value referenced
 * by a new_state pointer
 * @details This utility function is intended to help the conversion between spatial-state-derived instances
 * encapsulated in base state pointers. It sets the name and reference frame of the referenced state instance from
 * the new_state, and then passes the referenced values to a user-defined conversion callback function. For example,
 * if state is referencing a CartesianPose instance A and new_state is referencing a CartesianState instance B, the
 * conversion callback function should invoke A.set_pose(B.get_pose()).
 * @tparam StateT The derived spatial state type of the destination state
 * @tparam NewStateT The derived spatial state type of the new state
 * @param state A base state pointer referencing a spatial-state-derived instance to be modified
 * @param new_state A base state pointer referencing a spatial-state-derived instance to be converted into the state
 * parameter
 * @param conversion_callback A callback function taking parameters of type StateT and NewStateT, where the
 * referenced StateT instance can be modified as needed from the NewStateT value
 */
template<typename StateT, typename NewStateT>
inline void safe_spatial_state_conversion(
    std::shared_ptr<state_representation::State>& state, std::shared_ptr<state_representation::State>& new_state,
    std::function<void(StateT&, const NewStateT&)> conversion_callback = {}
) {
  auto derived_state_ptr = safe_dynamic_pointer_cast<StateT>(state);
  auto derived_new_state_ptr = safe_dynamic_pointer_cast<NewStateT>(new_state);
  StateT tmp_new_state;
  tmp_new_state.set_name(derived_new_state_ptr->get_name());
  tmp_new_state.set_reference_frame(derived_new_state_ptr->get_reference_frame());
  if (!derived_new_state_ptr->is_empty()) {
    if (conversion_callback) {
      conversion_callback(tmp_new_state, *derived_new_state_ptr);
    }
  }
  *derived_state_ptr = tmp_new_state;
}

/**
 * @brief Update the value referenced by a state pointer by dynamically casting and converting the value referenced
 * by a new_state pointer
 * @details This utility function is intended to help the conversion between joint-state-derived instances
 * encapsulated in base state pointers. It sets the name and joint names of the referenced state instance from
 * the new_state, and then passes the referenced values to a user-defined conversion callback function. For example,
 * if state is referencing a JointPositions instance A and new_state is referencing a JointState instance B, the
 * conversion callback function should invoke A.set_positions(B.get_positions()).
 * @tparam StateT The derived joint state type of the destination state
 * @tparam NewStateT The derived joint state type of the new state
 * @param state A base state pointer referencing a joint-state-derived instance to be modified
 * @param new_state A base state pointer referencing a joint-state-derived instance to be converted into the state
 * parameter
 * @param conversion_callback A callback function taking parameters of type StateT and NewStateT, where the
 * referenced StateT instance can be modified as needed from the NewStateT value
 */
template<typename StateT, typename NewStateT>
inline void safe_state_with_names_conversion(
    std::shared_ptr<state_representation::State>& state, std::shared_ptr<state_representation::State>& new_state,
    std::function<void(StateT&, const NewStateT&)> conversion_callback = {}
) {
  auto derived_state_ptr = safe_dynamic_pointer_cast<StateT>(state);
  auto derived_new_state_ptr = safe_dynamic_pointer_cast<NewStateT>(new_state);
  StateT tmp_new_state(derived_new_state_ptr->get_name(), derived_new_state_ptr->get_names());
  if (!derived_new_state_ptr->is_empty()) {
    if (conversion_callback) {
      conversion_callback(tmp_new_state, *derived_new_state_ptr);
    }
  }
  *derived_state_ptr = tmp_new_state;
}

template<>
inline void read_message(std::shared_ptr<state_representation::State>& state, const EncodedState& message) {
  using namespace state_representation;
  std::string tmp(message.data.begin(), message.data.end());
  std::shared_ptr<State> new_state;
  try {
    new_state = clproto::decode<std::shared_ptr<State>>(tmp);
  } catch (const std::exception& ex) {
    throw exceptions::MessageTranslationException(ex.what());
  }
  try {
    switch (state->get_type()) {
      case StateType::STATE: {
        if (new_state->get_type() == StateType::STATE) {
          *state = *new_state;
        } else {
          *state = State(new_state->get_name());
        }
        break;
      }
      case StateType::DIGITAL_IO_STATE: {
        if (new_state->get_type() == StateType::DIGITAL_IO_STATE) {
          safe_state_with_names_conversion<DigitalIOState, DigitalIOState>(
              state, new_state, [](DigitalIOState& a, const DigitalIOState& b) {
                a.set_data(b.data());
              });
        } else {
          safe_dynamic_cast<DigitalIOState>(state, new_state);
        }
        break;
      }
      case StateType::ANALOG_IO_STATE: {
        if (new_state->get_type() == StateType::ANALOG_IO_STATE) {
          safe_state_with_names_conversion<AnalogIOState, AnalogIOState>(
              state, new_state, [](AnalogIOState& a, const AnalogIOState& b) {
                a.set_data(b.data());
              });
        } else {
          safe_dynamic_cast<AnalogIOState>(state, new_state);
        }
        break;
      }
      case StateType::SPATIAL_STATE: {
        std::set<StateType> spatial_state_types = {
            StateType::SPATIAL_STATE, StateType::CARTESIAN_STATE, StateType::CARTESIAN_POSE, StateType::CARTESIAN_TWIST,
            StateType::CARTESIAN_ACCELERATION, StateType::CARTESIAN_WRENCH
        };
        if (spatial_state_types.find(new_state->get_type()) != spatial_state_types.cend()) {
          safe_spatial_state_conversion<SpatialState, SpatialState>(state, new_state);
        } else {
          safe_dynamic_cast<SpatialState>(state, new_state);
        }
        break;
      }
      case StateType::CARTESIAN_STATE: {
        if (new_state->get_type() == StateType::CARTESIAN_POSE) {
          safe_spatial_state_conversion<CartesianState, CartesianPose>(
              state, new_state, [](CartesianState& a, const CartesianPose& b) {
                a.set_pose(b.get_pose());
              });
        } else if (new_state->get_type() == StateType::CARTESIAN_TWIST) {
          safe_spatial_state_conversion<CartesianState, CartesianTwist>(
              state, new_state, [](CartesianState& a, const CartesianTwist& b) {
                a.set_twist(b.get_twist());
              });
        } else if (new_state->get_type() == StateType::CARTESIAN_ACCELERATION) {
          safe_spatial_state_conversion<CartesianState, CartesianAcceleration>(
              state, new_state, [](CartesianState& a, const CartesianAcceleration& b) {
                a.set_acceleration(b.get_acceleration());
              });
        } else if (new_state->get_type() == StateType::CARTESIAN_WRENCH) {
          safe_spatial_state_conversion<CartesianState, CartesianWrench>(
              state, new_state, [](CartesianState& a, const CartesianWrench& b) {
                a.set_wrench(b.get_wrench());
              });
        } else {
          safe_dynamic_cast<CartesianState>(state, new_state);
        }
        break;
      }
      case StateType::CARTESIAN_POSE: {
        if (new_state->get_type() == StateType::CARTESIAN_STATE) {
          safe_spatial_state_conversion<CartesianPose, CartesianState>(
              state, new_state, [](CartesianPose& a, const CartesianState& b) {
                a.set_pose(b.get_pose());
              });
        } else {
          safe_dynamic_cast<CartesianPose>(state, new_state);
        }
        break;
      }
      case StateType::CARTESIAN_TWIST: {
        if (new_state->get_type() == StateType::CARTESIAN_STATE) {
          safe_spatial_state_conversion<CartesianTwist, CartesianState>(
              state, new_state, [](CartesianTwist& a, const CartesianState& b) {
                a.set_twist(b.get_twist());
              });
        } else {
          safe_dynamic_cast<CartesianTwist>(state, new_state);
        }
        break;
      }
      case StateType::CARTESIAN_ACCELERATION: {
        if (new_state->get_type() == StateType::CARTESIAN_STATE) {
          safe_spatial_state_conversion<CartesianAcceleration, CartesianState>(
              state, new_state, [](CartesianAcceleration& a, const CartesianState& b) {
                a.set_acceleration(b.get_acceleration());
              });
        } else {
          safe_dynamic_cast<CartesianAcceleration>(state, new_state);
        }
        break;
      }
      case StateType::CARTESIAN_WRENCH: {
        if (new_state->get_type() == StateType::CARTESIAN_STATE) {
          safe_spatial_state_conversion<CartesianWrench, CartesianState>(
              state, new_state, [](CartesianWrench& a, const CartesianState& b) {
                a.set_wrench(b.get_wrench());
              });
        } else {
          safe_dynamic_cast<CartesianWrench>(state, new_state);
        }
        break;
      }
      case StateType::JOINT_STATE: {
        auto derived_state = safe_dynamic_pointer_cast<JointState>(state);
        if (new_state->get_type() == StateType::JOINT_POSITIONS) {
          safe_state_with_names_conversion<JointState, JointPositions>(
              state, new_state, [](JointState& a, const JointPositions& b) {
                a.set_positions(b.get_positions());
              });
        } else if (new_state->get_type() == StateType::JOINT_VELOCITIES) {
          safe_state_with_names_conversion<JointState, JointVelocities>(
              state, new_state, [](JointState& a, const JointVelocities& b) {
                a.set_velocities(b.get_velocities());
              });
        } else if (new_state->get_type() == StateType::JOINT_ACCELERATIONS) {
          safe_state_with_names_conversion<JointState, JointAccelerations>(
              state, new_state, [](JointState& a, const JointAccelerations& b) {
                a.set_accelerations(b.get_accelerations());
              });
        } else if (new_state->get_type() == StateType::JOINT_TORQUES) {
          safe_state_with_names_conversion<JointState, JointTorques>(
              state, new_state, [](JointState& a, const JointTorques& b) {
                a.set_torques(b.get_torques());
              });
        } else {
          safe_dynamic_cast<JointState>(state, new_state);
        }
        break;
      }
      case StateType::JOINT_POSITIONS: {
        if (new_state->get_type() == StateType::JOINT_STATE) {
          safe_state_with_names_conversion<JointPositions, JointState>(
              state, new_state, [](JointPositions& a, const JointState& b) {
                a.set_positions(b.get_positions());
              });
        } else {
          safe_dynamic_cast<JointPositions>(state, new_state);
        }
        break;
      }
      case StateType::JOINT_VELOCITIES: {
        if (new_state->get_type() == StateType::JOINT_STATE) {
          safe_state_with_names_conversion<JointVelocities, JointState>(
              state, new_state, [](JointVelocities& a, const JointState& b) {
                a.set_velocities(b.get_velocities());
              });
        } else {
          safe_dynamic_cast<JointVelocities>(state, new_state);
        }
        break;
      }
      case StateType::JOINT_ACCELERATIONS: {
        if (new_state->get_type() == StateType::JOINT_STATE) {
          safe_state_with_names_conversion<JointAccelerations, JointState>(
              state, new_state, [](JointAccelerations& a, const JointState& b) {
                a.set_accelerations(b.get_accelerations());
              });
        } else {
          safe_dynamic_cast<JointAccelerations>(state, new_state);
        }
        break;
      }
      case StateType::JOINT_TORQUES: {
        if (new_state->get_type() == StateType::JOINT_STATE) {
          safe_state_with_names_conversion<JointTorques, JointState>(
              state, new_state, [](JointTorques& a, const JointState& b) {
                a.set_torques(b.get_torques());
              });
        } else {
          safe_dynamic_cast<JointTorques>(state, new_state);
        }
        break;
      }
      case StateType::JACOBIAN:
        safe_dynamic_cast<Jacobian>(state, new_state);
        break;
      case StateType::PARAMETER: {
        auto param_ptr = std::dynamic_pointer_cast<ParameterInterface>(state);
        switch (param_ptr->get_parameter_type()) {
          case ParameterType::BOOL:
            safe_dynamic_cast<Parameter<bool>>(state, new_state);
            break;
          case ParameterType::BOOL_ARRAY:
            safe_dynamic_cast<Parameter<std::vector<bool>>>(state, new_state);
            break;
          case ParameterType::INT:
            safe_dynamic_cast<Parameter<int>>(state, new_state);
            break;
          case ParameterType::INT_ARRAY:
            safe_dynamic_cast<Parameter<std::vector<int>>>(state, new_state);
            break;
          case ParameterType::DOUBLE:
            safe_dynamic_cast<Parameter<double>>(state, new_state);
            break;
          case ParameterType::DOUBLE_ARRAY:
            safe_dynamic_cast<Parameter<std::vector<double>>>(state, new_state);
            break;
          case ParameterType::STRING:
            safe_dynamic_cast<Parameter<std::string>>(state, new_state);
            break;
          case ParameterType::STRING_ARRAY:
            safe_dynamic_cast<Parameter<std::vector<std::string>>>(state, new_state);
            break;
          case ParameterType::VECTOR:
            safe_dynamic_cast<Parameter<Eigen::VectorXd>>(state, new_state);
            break;
          case ParameterType::MATRIX:
            safe_dynamic_cast<Parameter<Eigen::MatrixXd>>(state, new_state);
            break;
          default:
            throw exceptions::MessageTranslationException(
                "The ParameterType contained by parameter " + param_ptr->get_name() + " is unsupported.");
        }
        break;
      }
      default:
        throw exceptions::MessageTranslationException(
            "The StateType contained by state " + new_state->get_name() + " is unsupported.");
    }
  } catch (const exceptions::MessageTranslationException& ex) {
    throw;
  }
}
}// namespace modulo_core::translators
