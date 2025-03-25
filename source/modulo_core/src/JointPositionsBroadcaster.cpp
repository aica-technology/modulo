#include "modulo_core/JointPositionsBroadcaster.hpp"

namespace modulo_core {

template<>
void JointPositionsBroadcaster::send(const std::vector<modulo_interfaces::msg::JointPositions>& joint_positions) {
  send<modulo_interfaces::msg::JointPositions>(
      joint_positions,
      [](const modulo_interfaces::msg::JointPositions& joint_positions) { return joint_positions.header.frame_id; },
      [](const modulo_interfaces::msg::JointPositions& joint_positions) { return joint_positions; });
}

template<>
void JointPositionsBroadcaster::send(const std::vector<state_representation::JointPositions>& joint_positions) {
  send<state_representation::JointPositions>(
      joint_positions,
      [](const state_representation::JointPositions& joint_positions) { return joint_positions.get_name(); },
      [](const state_representation::JointPositions& joint_positions) {
        modulo_interfaces::msg::JointPositions message;
        message.header.frame_id = joint_positions.get_name();
        message.joint_names = joint_positions.get_names();
        message.positions = joint_positions.to_std_vector();
        return message;
      });
}
}// namespace modulo_core
