#include <gtest/gtest.h>

#include <rclcpp/node_options.hpp>

#include "modulo_core/EncodedState.hpp"
#include "test_modulo_components/component_public_interfaces.hpp"

using namespace state_representation;

namespace modulo_components {

class ComponentTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    component_ = std::make_shared<ComponentPublicInterface>(rclcpp::NodeOptions());
  }

  std::shared_ptr<ComponentPublicInterface> component_;
};

TEST_F(ComponentTest, AddRemoveOutput) {
  std::shared_ptr<State> data = make_shared_state(CartesianState::Random("test"));
  component_->add_output("_tEsT_#1@3", data);
  EXPECT_TRUE(component_->outputs_.find("test_13") != component_->outputs_.end());
  EXPECT_NO_THROW(component_->outputs_.at("test_13")->publish());
  EXPECT_THROW(component_->publish_output("test_13"), exceptions::ComponentException);

  auto new_data = std::make_shared<bool>(false);
  component_->add_output("test_13", new_data);
  EXPECT_EQ(component_->outputs_.at("test_13")->get_message_pair()->get_type(),
            modulo_core::communication::MessageType::ENCODED_STATE);

  component_->remove_output("test_13");
  EXPECT_TRUE(component_->outputs_.find("test_13") == component_->outputs_.end());

  component_->add_output("_tEsT_#1@3", data, "", true, false);
  EXPECT_FALSE(component_->periodic_outputs_.at("test_13"));
  EXPECT_NO_THROW(component_->publish_output("_tEsT_#1@3"));
  EXPECT_NO_THROW(component_->publish_output("test_13"));
  EXPECT_THROW(component_->publish_output(""), exceptions::ComponentException);
}
} // namespace modulo_components
