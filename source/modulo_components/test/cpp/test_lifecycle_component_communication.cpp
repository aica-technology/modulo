#include <gtest/gtest.h>

#include <modulo_utils/testutils/PredicatesListener.hpp>

#include "modulo_components/LifecycleComponent.hpp"
#include "test_modulo_components/communication_components.hpp"

using namespace modulo_components;

class LifecycleTrigger : public LifecycleComponent {
public:
  explicit LifecycleTrigger(const rclcpp::NodeOptions& node_options) : LifecycleComponent(node_options, "trigger") {}

  bool on_configure_callback() final {
    this->add_trigger("test");
    return true;
  }

  bool on_activate_callback() final {
    this->trigger("test");
    return true;
  }
};

class LifecycleComponentCommunicationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(LifecycleComponentCommunicationTest, InputOutput) {
  auto cartesian_state = state_representation::CartesianState::Random("test");
  auto input_node = std::make_shared<MinimalCartesianInput<LifecycleComponent>>(rclcpp::NodeOptions(), "/topic");
  auto output_node = std::make_shared<MinimalCartesianOutput<LifecycleComponent>>(
      rclcpp::NodeOptions(), "/topic", cartesian_state, true);
  add_configure_activate(this->exec_, input_node);
  add_configure_activate(this->exec_, output_node);
  auto return_code = this->exec_->spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(cartesian_state.get_name(), input_node->input->get_name());
  EXPECT_TRUE(cartesian_state.data().isApprox(input_node->input->data()));
  EXPECT_THROW(output_node->publish(), modulo_components::exceptions::ComponentException);
}

TEST_F(LifecycleComponentCommunicationTest, InputOutputManual) {
  auto cartesian_state = state_representation::CartesianState::Random("test");
  auto input_node = std::make_shared<MinimalCartesianInput<LifecycleComponent>>(rclcpp::NodeOptions(), "/topic");
  auto output_node = std::make_shared<MinimalCartesianOutput<LifecycleComponent>>(
      rclcpp::NodeOptions(), "/topic", cartesian_state, false);
  add_configure_activate(this->exec_, input_node);
  add_configure_activate(this->exec_, output_node);
  auto return_code = this->exec_->spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::TIMEOUT);
  output_node->publish();
  return_code = this->exec_->template spin_until_future_complete(input_node->received_future, 500ms);
  ASSERT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(cartesian_state.get_name(), input_node->input->get_name());
  EXPECT_TRUE(cartesian_state.data().isApprox(input_node->input->data()));
}

TEST_F(LifecycleComponentCommunicationTest, Trigger) {
  auto trigger = std::make_shared<LifecycleTrigger>(rclcpp::NodeOptions());
  auto listener =
      std::make_shared<modulo_utils::testutils::PredicatesListener>("trigger", std::vector<std::string>{"test"});
  this->exec_->add_node(trigger->get_node_base_interface());
  trigger->configure();
  this->exec_->add_node(listener);
  auto result_code = this->exec_->spin_until_future_complete(listener->get_predicate_future(), 500ms);
  ASSERT_EQ(result_code, rclcpp::FutureReturnCode::TIMEOUT);
  EXPECT_FALSE(listener->get_predicate_values().at("test"));
  trigger->activate();
  result_code = this->exec_->spin_until_future_complete(listener->get_predicate_future(), 500ms);
  ASSERT_EQ(result_code, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(listener->get_predicate_values().at("test"));
}
