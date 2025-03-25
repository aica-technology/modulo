#include "modulo_core/JointPositionsListener.hpp"

#include "modulo_core/exceptions.hpp"

namespace modulo_core {

static state_representation::JointPositions
translate_joint_positions(const modulo_interfaces::msg::JointPositions& joint_positions) {
  auto state = state_representation::JointPositions(joint_positions.header.frame_id, joint_positions.joint_names);
  state.set_positions(joint_positions.positions);
  return state;
}

JointPositionsListener::~JointPositionsListener() {
  if (spin_thread_) {
    executor_->cancel();
    dedicated_listener_thread_->join();
  }
}

void JointPositionsListener::subscription_callback(
    const std::shared_ptr<modulo_interfaces::msg::JointPositionsCollection> message) {
  for (const auto& msg : message->joint_positions) {
    buffer_.push_back(msg);
  }
}

template<>
modulo_interfaces::msg::JointPositions JointPositionsListener::lookup(const std::string& name) const {
  auto result =
      std::find_if(buffer_.begin(), buffer_.end(), [name](modulo_interfaces::msg::JointPositions joint_positions) {
        return joint_positions.header.frame_id == name;
      });
  if (result == buffer_.end()) {
    throw exceptions::LookupJointPositionsException("Joint positions " + name + " is not available");
  }
  return *result;
}

template<>
state_representation::JointPositions JointPositionsListener::lookup(const std::string& name) const {
  return translate_joint_positions(lookup<modulo_interfaces::msg::JointPositions>(name));
}

template<>
std::vector<modulo_interfaces::msg::JointPositions> JointPositionsListener::get_buffer() const {
  return buffer_;
}

template<>
std::vector<state_representation::JointPositions> JointPositionsListener::get_buffer() const {
  std::vector<state_representation::JointPositions> states;
  for (const auto& joint_positions : buffer_) {
    states.push_back(translate_joint_positions(joint_positions));
  }
  return states;
}
}// namespace modulo_core
