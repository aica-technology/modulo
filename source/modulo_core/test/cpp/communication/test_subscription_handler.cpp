#include <gtest/gtest.h>

#include <rclcpp/node.hpp>

#include "modulo_core/communication/SubscriptionHandler.hpp"

using namespace modulo_core::communication;

template<typename MsgT, typename DataT>
static void test_subscription_interface(const std::shared_ptr<rclcpp::Node>& node, const DataT& value) {
  // create message pair
  auto data = std::make_shared<DataT>(value);
  auto message_pair = std::make_shared<MessagePair<MsgT, DataT>>(data, node->get_clock());

  // create subscription handler
  auto subscription_handler = std::make_shared<SubscriptionHandler<MsgT>>(message_pair);
  auto subscription = node->template create_subscription<MsgT>(
      "topic", 10, subscription_handler->get_callback());

  // use in subscription interface
  auto subscription_interface = subscription_handler->create_subscription_interface(subscription);
}

class SubscriptionTest : public ::testing::Test {
public:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

protected:
  void SetUp() {
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(SubscriptionTest, BasicTypes) {
  test_subscription_interface<std_msgs::msg::Bool, bool>(node, true);
  test_subscription_interface<std_msgs::msg::Float64, double>(node, 0.1);
  test_subscription_interface<std_msgs::msg::Float64MultiArray, std::vector<double>>(node, {0.1, 0.2, 0.3});
  test_subscription_interface<std_msgs::msg::Int32, int>(node, 1);
  test_subscription_interface<std_msgs::msg::String, std::string>(node, "this");
}

TEST_F(SubscriptionTest, EncodedState) {
  // create message pair
  auto data =
      std::make_shared<state_representation::CartesianState>(state_representation::CartesianState::Random("test"));
  auto message_pair = std::make_shared<MessagePair<modulo_core::EncodedState, state_representation::State>>(
      data, node->get_clock());

  // create subscription handler
  auto subscription_handler = std::make_shared<SubscriptionHandler<modulo_core::EncodedState>>(message_pair);
  auto subscription = node->create_subscription<modulo_core::EncodedState>(
      "topic", 10, subscription_handler->get_callback());

  // use in subscription interface
  auto subscription_interface = subscription_handler->create_subscription_interface(subscription);
}
