#include <gtest/gtest.h>

#include "modulo_core/JointPositionsBroadcaster.hpp"
#include "modulo_core/JointPositionsListener.hpp"

using namespace modulo_core;

TEST(TestJointPositions, SendLookup) {
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_joint_positions");

  JointPositionsListener listener(node);
  JointPositionsBroadcaster broadcaster(node);

  modulo_interfaces::msg::JointPositions joint_pos;
  joint_pos.header.frame_id = "foo";
  joint_pos.joint_names = {"one", "two"};
  joint_pos.positions = {1.0, 2.0};
  broadcaster.send(joint_pos);

  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(10));

  state_representation::JointPositions joint_positions;
  EXPECT_NO_THROW(joint_positions = listener.lookup<state_representation::JointPositions>("foo"));
  EXPECT_EQ(joint_positions.get_name(), joint_pos.header.frame_id);
  EXPECT_EQ(joint_positions.get_names(), joint_pos.joint_names);
  EXPECT_EQ(joint_positions.to_std_vector(), joint_pos.positions);
}
