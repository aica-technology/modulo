#include <gtest/gtest.h>

#include "modulo_components/LifecycleComponent.hpp"
#include "test_modulo_components/communication_components.hpp"

using namespace modulo_components;

class Trigger : public LifecycleComponent {
public:
  Trigger(const rclcpp::NodeOptions& node_options) : LifecycleComponent(node_options, "trigger") {}

  bool on_configure_callback() final {
    this->add_trigger("test");
    return true;
  }

  bool on_activate_callback() final {
    this->trigger("test");
    return true;
  }
};

class TriggerListener : public LifecycleComponent {
public:
  TriggerListener(const rclcpp::NodeOptions& node_options, const std::string& topic) :
      LifecycleComponent(node_options, "trigger_listener"), input(std::make_shared<bool>()) {
    this->received_future = this->received_.get_future();
    this->add_input("trigger", this->input, [this]() { this->received_.set_value(); }, topic);
  }

  void reset_future() {
    this->received_ = std::promise<void>();
    this->received_future = this->received_.get_future();
  }

  std::shared_ptr<bool> input;
  std::shared_future<void> received_future;

private:
  std::promise<void> received_;
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
  auto output_node =
      std::make_shared<MinimalCartesianOutput<LifecycleComponent>>(rclcpp::NodeOptions(), "/topic", cartesian_state);
  add_configure_activate(this->exec_, input_node);
  add_configure_activate(this->exec_, output_node);
  this->exec_->template spin_until_future_complete(input_node->received_future, 500ms);
  EXPECT_EQ(cartesian_state.get_name(), input_node->input->get_name());
  EXPECT_TRUE(cartesian_state.data().isApprox(input_node->input->data()));
  this->exec_.reset();
}

TEST_F(LifecycleComponentCommunicationTest, Trigger) {
  auto output_node = std::make_shared<Trigger>(rclcpp::NodeOptions());
  auto input_node = std::make_shared<TriggerListener>(rclcpp::NodeOptions(), "/predicates/trigger/test");
  add_configure_activate(this->exec_, input_node);
  this->exec_->add_node(output_node->get_node_base_interface());
  output_node->configure();
  this->exec_->template spin_until_future_complete(input_node->received_future, 500ms);
  EXPECT_FALSE(*input_node->input);
  input_node->reset_future();
  output_node->activate();
  this->exec_->template spin_until_future_complete(input_node->received_future, 500ms);
  EXPECT_TRUE(*input_node->input);
}
