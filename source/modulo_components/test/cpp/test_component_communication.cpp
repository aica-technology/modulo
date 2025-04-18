#include <gtest/gtest.h>

#include <modulo_utils/testutils/PredicatesListener.hpp>

#include "test_modulo_components/communication_components.hpp"
#include "test_modulo_components/component_public_interfaces.hpp"

using namespace modulo_components;

class ComponentCommunicationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(ComponentCommunicationTest, InputOutput) {
  auto cartesian_state = state_representation::CartesianState::Random("test");
  auto input_node = std::make_shared<MinimalCartesianInput<Component>>(rclcpp::NodeOptions(), "/topic");
  auto output_node =
      std::make_shared<MinimalCartesianOutput<Component>>(rclcpp::NodeOptions(), "/topic", cartesian_state, true);
  this->exec_->add_node(input_node);
  this->exec_->add_node(output_node);
  auto return_code = this->exec_->spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(cartesian_state.get_name(), input_node->input->get_name());
  EXPECT_TRUE(cartesian_state.data().isApprox(input_node->input->data()));
  EXPECT_THROW(output_node->publish(), modulo_core::exceptions::CoreException);
}

TEST_F(ComponentCommunicationTest, ExceptionInputOutput) {
  auto cartesian_state = state_representation::CartesianState::Random("test");
  auto input_node = std::make_shared<ExceptionCartesianInput<Component>>(rclcpp::NodeOptions(), "/topic");
  auto output_node =
      std::make_shared<MinimalCartesianOutput<Component>>(rclcpp::NodeOptions(), "/topic", cartesian_state, true);
  this->exec_->add_node(input_node);
  this->exec_->add_node(output_node);
  auto return_code = this->exec_->spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS);
}

TEST_F(ComponentCommunicationTest, InputOutputManual) {
  auto cartesian_state = state_representation::CartesianState::Random("test");
  auto input_node = std::make_shared<MinimalCartesianInput<Component>>(rclcpp::NodeOptions(), "/topic");
  auto output_node =
      std::make_shared<MinimalCartesianOutput<Component>>(rclcpp::NodeOptions(), "/topic", cartesian_state, false);
  this->exec_->add_node(input_node);
  this->exec_->add_node(output_node);
  auto return_code = this->exec_->spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::TIMEOUT);
  output_node->publish();
  return_code = this->exec_->template spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(cartesian_state.get_name(), input_node->input->get_name());
  EXPECT_TRUE(cartesian_state.data().isApprox(input_node->input->data()));
}

TEST_F(ComponentCommunicationTest, TwistInputOutput) {
  auto twist = std::make_shared<geometry_msgs::msg::Twist>();
  twist->linear.x = 1.0;
  auto input_node = std::make_shared<MinimalTwistInput<Component>>(rclcpp::NodeOptions(), "/topic");
  auto output_node = std::make_shared<MinimalTwistOutput<Component>>(rclcpp::NodeOptions(), "/topic", twist, true);
  this->exec_->add_node(input_node);
  this->exec_->add_node(output_node);
  auto return_code = this->exec_->spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(twist->linear.x, input_node->input->linear.x);
  EXPECT_THROW(output_node->publish(), modulo_core::exceptions::CoreException);
}

TEST_F(ComponentCommunicationTest, Trigger) {
  auto trigger = std::make_shared<MinimalTrigger<ComponentPublicInterface>>(rclcpp::NodeOptions());
  auto listener =
      std::make_shared<modulo_utils::testutils::PredicatesListener>("/trigger", std::vector<std::string>{"test"});
  this->exec_->add_node(listener);
  this->exec_->add_node(trigger);
  auto result_code = this->exec_->spin_until_future_complete(listener->get_predicate_future(), 500ms);
  ASSERT_EQ(result_code, rclcpp::FutureReturnCode::TIMEOUT);
  EXPECT_FALSE(listener->get_predicate_values().at("test"));
  trigger->trigger();
  result_code = this->exec_->spin_until_future_complete(listener->get_predicate_future(), 500ms);
  ASSERT_EQ(result_code, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(listener->get_predicate_values().at("test"));
  EXPECT_FALSE(trigger->get_predicate("trigger"));
}
