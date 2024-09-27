#include <gtest/gtest.h>

#include "modulo_core/communication/MessagePair.hpp"

using namespace modulo_core::communication;

template<typename MsgT, typename DataT>
static void
test_message_interface(const DataT& initial_value, const DataT& new_value, const std::shared_ptr<rclcpp::Clock> clock) {
  auto data = std::make_shared<DataT>(initial_value);
  auto message_pair = std::make_shared<MessagePair<MsgT, DataT>>(data, clock);
  EXPECT_EQ(initial_value, *message_pair->get_data());
  EXPECT_EQ(initial_value, message_pair->write_message().data);

  std::shared_ptr<MessagePairInterface> message_pair_interface(message_pair);
  auto message = message_pair_interface->template write<MsgT, DataT>();
  EXPECT_EQ(initial_value, message.data);

  *data = new_value;
  EXPECT_EQ(new_value, *message_pair->get_data());
  EXPECT_EQ(new_value, message_pair->write_message().data);
  message = message_pair_interface->template write<MsgT, DataT>();
  EXPECT_EQ(new_value, message.data);

  message = MsgT();
  message.data = initial_value;
  message_pair_interface->template read<MsgT, DataT>(message);
  EXPECT_EQ(initial_value, *message_pair->get_data());
}

class MessagePairTest : public ::testing::Test {
protected:
  void SetUp() override { clock_ = std::make_shared<rclcpp::Clock>(); }

  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(MessagePairTest, BasicTypes) {
  test_message_interface<std_msgs::msg::Bool, bool>(false, true, clock_);
  test_message_interface<std_msgs::msg::Float64, double>(0.1, 0.2, clock_);
  test_message_interface<std_msgs::msg::Float64MultiArray, std::vector<double>>(
      {0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, clock_);
  test_message_interface<std_msgs::msg::Int32, int>(1, 2, clock_);
  test_message_interface<std_msgs::msg::String, std::string>("this", "that", clock_);
}

TEST_F(MessagePairTest, EncodedState) {
  auto initial_value = state_representation::CartesianState::Random("test");
  auto data = state_representation::make_shared_state(initial_value);
  auto message_pair =
      std::make_shared<MessagePair<modulo_core::EncodedState, state_representation::State>>(data, clock_);
  EXPECT_TRUE(initial_value.data().isApprox(
      std::dynamic_pointer_cast<state_representation::CartesianState>(message_pair->get_data())->data()));

  std::shared_ptr<MessagePairInterface> message_pair_interface(message_pair);
  auto message = message_pair_interface->write<modulo_core::EncodedState, state_representation::State>();
  std::string tmp(message.data.begin(), message.data.end());
  auto decoded = clproto::decode<state_representation::CartesianState>(tmp);
  EXPECT_TRUE(initial_value.data().isApprox(decoded.data()));

  auto new_value = state_representation::CartesianState::Identity("world");
  message_pair->set_data(state_representation::make_shared_state(new_value));
  message = message_pair_interface->write<modulo_core::EncodedState, state_representation::State>();
  tmp = std::string(message.data.begin(), message.data.end());
  decoded = clproto::decode<state_representation::CartesianState>(tmp);
  EXPECT_TRUE(new_value.data().isApprox(decoded.data()));

  data = state_representation::make_shared_state(initial_value);
  message_pair->set_data(data);
  message = modulo_core::EncodedState();
  modulo_core::translators::write_message(message, data, clock_->now());
  message_pair_interface->read<modulo_core::EncodedState, state_representation::State>(message);
  EXPECT_TRUE(initial_value.data().isApprox(
      std::dynamic_pointer_cast<state_representation::CartesianState>(message_pair->get_data())->data()));
}
