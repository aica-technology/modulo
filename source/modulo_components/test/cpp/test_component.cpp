#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <modulo_core/EncodedState.hpp>

#include "test_modulo_components/component_public_interfaces.hpp"

using namespace state_representation;
using namespace std::chrono_literals;

namespace modulo_components {

class ComponentTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { component_ = std::make_shared<ComponentPublicInterface>(rclcpp::NodeOptions()); }

  std::shared_ptr<ComponentPublicInterface> component_;
};

TEST_F(ComponentTest, RateParameter) {
  std::shared_ptr<ComponentPublicInterface> component;
  auto node_options = rclcpp::NodeOptions();
  component = std::make_shared<ComponentPublicInterface>(node_options);
  EXPECT_EQ(component->template get_parameter_value<double>("rate"), 10.0);
  EXPECT_EQ(component->get_rate(), 10.0);
  auto double_period = component->template get_period<double>();
  EXPECT_EQ(double_period, 0.1);
  auto chrono_period = component->template get_period<std::chrono::nanoseconds>();
  EXPECT_EQ(chrono_period, 100ms);
  auto ros_duration = component->template get_period<rclcpp::Duration>();
  EXPECT_EQ(ros_duration.seconds(), 0.1);
  EXPECT_EQ(ros_duration.nanoseconds(), 1e8);

  node_options = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("rate", 200.0)});
  component = std::make_shared<ComponentPublicInterface>(node_options);
  EXPECT_EQ(component->template get_parameter_value<double>("rate"), 200.0);
  EXPECT_EQ(component->get_rate(), 200.0);
  double_period = component->template get_period<double>();
  EXPECT_EQ(double_period, 0.005);
  chrono_period = component->template get_period<std::chrono::nanoseconds>();
  EXPECT_EQ(chrono_period, 5000000ns);
  ros_duration = component->template get_period<rclcpp::Duration>();
  EXPECT_EQ(ros_duration.seconds(), 0.005);
  EXPECT_EQ(ros_duration.nanoseconds(), 5e6);
}

TEST_F(ComponentTest, AddRemoveOutput) {
  std::shared_ptr<State> data = make_shared_state(CartesianState::Random("test"));
  component_->add_output("8_teEsTt_#1@3", data);
  EXPECT_TRUE(component_->outputs_.find("test_13") != component_->outputs_.end());
  EXPECT_NO_THROW(component_->outputs_.at("test_13")->publish());
  EXPECT_THROW(component_->publish_output("test_13"), modulo_core::exceptions::CoreException);

  auto new_data = std::make_shared<bool>(false);
  component_->add_output("test_13", new_data);
  EXPECT_EQ(
      component_->outputs_.at("test_13")->get_message_pair()->get_type(),
      modulo_core::communication::MessageType::ENCODED_STATE);

  component_->remove_output("test_13");
  EXPECT_TRUE(component_->outputs_.find("test_13") == component_->outputs_.end());

  component_->add_output("8_teEsTt_#1@3", data, "", true, false);
  EXPECT_FALSE(component_->periodic_outputs_.at("test_13"));
  EXPECT_NO_THROW(component_->publish_output("8_teEsTt_#1@3"));
  EXPECT_NO_THROW(component_->publish_output("test_13"));
  EXPECT_THROW(component_->publish_output(""), modulo_core::exceptions::CoreException);

  auto std_msg_data = std::make_shared<std_msgs::msg::String>();
  std_msg_data->data = "foo";
  component_->add_output("custom_msg_test", std_msg_data);
  EXPECT_TRUE(component_->outputs_.find("custom_msg_test") != component_->outputs_.end());
  EXPECT_NO_THROW(component_->outputs_.at("custom_msg_test")->publish());
  EXPECT_THROW(component_->publish_output("custom_msg_test"), modulo_core::exceptions::CoreException);

  auto geometry_msg_data = std::make_shared<geometry_msgs::msg::Twist>();
  component_->add_output("geometry_msg_test", geometry_msg_data);
  EXPECT_TRUE(component_->outputs_.find("geometry_msg_test") != component_->outputs_.end());
  EXPECT_NO_THROW(component_->outputs_.at("geometry_msg_test")->publish());
  EXPECT_THROW(component_->publish_output("geometry_msg_test"), modulo_core::exceptions::CoreException);

  auto sensor_msg_data = std::make_shared<sensor_msgs::msg::Image>();
  component_->add_output("sensor_msg_test", sensor_msg_data);
  EXPECT_TRUE(component_->outputs_.find("sensor_msg_test") != component_->outputs_.end());
  EXPECT_NO_THROW(component_->outputs_.at("sensor_msg_test")->publish());
  EXPECT_THROW(component_->publish_output("sensor_msg_test"), modulo_core::exceptions::CoreException);
}
}// namespace modulo_components
