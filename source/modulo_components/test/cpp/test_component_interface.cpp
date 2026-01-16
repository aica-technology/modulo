#include <gtest/gtest.h>

#include <chrono>

#include <modulo_core/EncodedState.hpp>
#include <modulo_core/translators/message_writers.hpp>
#include <modulo_utils/testutils/ServiceClient.hpp>

#include "test_modulo_components/component_public_interfaces.hpp"

#include <sensor_msgs/msg/image.hpp>

namespace modulo_components {

using namespace std::chrono_literals;

template<class NodeT>
class ComponentInterfaceTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->node_ = std::make_shared<NodeT>("ComponentInterfacePublicInterface", rclcpp::NodeOptions());
    this->component_ = std::make_shared<ComponentInterfacePublicInterface>(this->node_);
    if (std::is_same<NodeT, rclcpp::Node>::value) {
      this->pub_type_ = modulo_core::communication::PublisherType::PUBLISHER;
    } else if (std::is_same<NodeT, rclcpp_lifecycle::LifecycleNode>::value) {
      this->pub_type_ = modulo_core::communication::PublisherType::LIFECYCLE_PUBLISHER;
    }
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<ComponentInterfacePublicInterface> component_;
  std::shared_ptr<NodeT> node_;
  modulo_core::communication::PublisherType pub_type_;
};

using NodeTypes = ::testing::Types<rclcpp::Node, rclcpp_lifecycle::LifecycleNode>;
TYPED_TEST_SUITE(ComponentInterfaceTest, NodeTypes);

TYPED_TEST(ComponentInterfaceTest, AddAssignment) {
  this->component_->add_assignment("an_assignment", state_representation::ParameterType::INT);
  const auto one = this->component_->assignments_.size();
  // adding an assignment with empty name should fail
  EXPECT_THROW(this->component_->add_assignment("", state_representation::ParameterType::INT), modulo_core::exceptions::AddAssignmentException);
  // adding an assignment with the same name should just overwrite
  this->component_->add_assignment("an_assignment", state_representation::ParameterType::INT);
  EXPECT_EQ(this->component_->assignments_.size(), one);
  // names should be cleaned up
  EXPECT_NO_THROW(this->component_->add_assignment("7cleEaGn_AaSssiGNgn#ment", state_representation::ParameterType::DOUBLE));
  EXPECT_NE(this->component_->assignments_.size(), one);
  // names without valid characters should fail
  EXPECT_THROW(this->component_->add_assignment("@@@@@@", state_representation::ParameterType::DOUBLE), modulo_core::exceptions::AddAssignmentException);

  auto assignment_iterator = this->component_->assignments_.find("an_assignment");
  auto another_assignment_iterator = this->component_->assignments_.find("clean_assignment");
  auto no_assignment_iterator = this->component_->assignments_.find("no_assignment");

  EXPECT_TRUE(assignment_iterator != this->component_->assignments_.end());
  EXPECT_TRUE(another_assignment_iterator != this->component_->assignments_.end());
  EXPECT_TRUE(no_assignment_iterator == this->component_->assignments_.end());
}

TYPED_TEST(ComponentInterfaceTest, TriggerAssignment) {
  this->component_->add_assignment("trigger_assignment_string", state_representation::ParameterType::STRING);
  this->component_->add_assignment("trigger_assignment_int", state_representation::ParameterType::INT);

  EXPECT_NO_THROW(this->component_->trigger_assignment("trigger_assignment_string", std::string("test")));
  EXPECT_NO_THROW(this->component_->trigger_assignment("trigger_assignment_int", 5));
  EXPECT_THROW(this->component_->trigger_assignment("trigger_assignment_string", 5), std::runtime_error);
  EXPECT_THROW(this->component_->trigger_assignment("trigger_assignment_int", std::string("test")), std::runtime_error);
}

TYPED_TEST(ComponentInterfaceTest, AddBoolPredicate) {
  this->component_->add_predicate("foo", true);
  auto predicate_iterator = this->component_->predicates_.find("foo");
  EXPECT_TRUE(predicate_iterator != this->component_->predicates_.end());
  EXPECT_TRUE(predicate_iterator->second.get_value());
}

TYPED_TEST(ComponentInterfaceTest, AddFunctionPredicate) {
  this->component_->add_predicate("bar", [&]() { return false; });
  auto predicate_iterator = this->component_->predicates_.find("bar");
  EXPECT_TRUE(predicate_iterator != this->component_->predicates_.end());
  EXPECT_FALSE(predicate_iterator->second.get_value());
}

TYPED_TEST(ComponentInterfaceTest, GetPredicateValue) {
  this->component_->add_predicate("foo", true);
  EXPECT_TRUE(this->component_->get_predicate("foo"));
  this->component_->add_predicate("bar", [&]() { return true; });
  EXPECT_TRUE(this->component_->get_predicate("bar"));
  // predicate does not exist, expect false
  EXPECT_FALSE(this->component_->get_predicate("test"));
  // error in callback function except false
  this->component_->add_predicate("error", [&]() -> bool { throw std::runtime_error("An error occurred"); });
  EXPECT_FALSE(this->component_->get_predicate("error"));
}

TYPED_TEST(ComponentInterfaceTest, SetPredicateValue) {
  this->component_->add_predicate("foo", true);
  this->component_->set_predicate("foo", false);
  EXPECT_FALSE(this->component_->get_predicate("foo"));
  // predicate does not exist so setting won't do anything
  this->component_->set_predicate("bar", true);
  EXPECT_FALSE(this->component_->get_predicate("bar"));
  this->component_->add_predicate("bar", true);
  this->component_->set_predicate("bar", [&]() { return false; });
  EXPECT_FALSE(this->component_->get_predicate("bar"));
}

TYPED_TEST(ComponentInterfaceTest, DeclareSignal) {
  this->component_->declare_input("input", "test");
  EXPECT_EQ(this->component_->template get_parameter_value<std::string>("input_topic"), "test");
  EXPECT_TRUE(this->component_->inputs_.find("input") == this->component_->inputs_.end());
  this->component_->declare_output("output", "test_again");
  EXPECT_EQ(this->component_->template get_parameter_value<std::string>("output_topic"), "test_again");
  EXPECT_TRUE(this->component_->outputs_.find("output") == this->component_->outputs_.end());
}

TYPED_TEST(ComponentInterfaceTest, AddRemoveInput) {
  auto data = std::make_shared<bool>(true);
  EXPECT_NO_THROW(this->component_->add_input("8_teEsTt_#1@3", data));
  EXPECT_FALSE(this->component_->inputs_.find("test_13") == this->component_->inputs_.end());
  EXPECT_EQ(this->component_->template get_parameter_value<std::string>("test_13_topic"), "~/test_13");

  EXPECT_NO_THROW(this->component_->template add_input<std_msgs::msg::Bool>(
      "9_tEestT_#1@5", [](const std::shared_ptr<std_msgs::msg::Bool>) {}, "/topic", true));
  EXPECT_FALSE(this->component_->inputs_.find("test_15") == this->component_->inputs_.end());
  EXPECT_EQ(this->component_->template get_parameter_value<std::string>("test_15_topic"), "/topic");

  // trying to add an input that already exists should fail
  this->component_->template add_input<std_msgs::msg::String>(
      "test_13", [](const std::shared_ptr<std_msgs::msg::String>) {});
  EXPECT_EQ(
      this->component_->inputs_.at("test_13")->get_message_pair()->get_type(),
      modulo_core::communication::MessageType::BOOL);

  // remove input
  this->component_->remove_input("test_13");
  EXPECT_TRUE(this->component_->inputs_.find("test_13") == this->component_->inputs_.end());

  EXPECT_NO_THROW(this->component_->add_input("sensor_msg_data", std::make_shared<sensor_msgs::msg::Image>()));
  EXPECT_FALSE(this->component_->inputs_.find("sensor_msg_data") == this->component_->inputs_.end());
}

TYPED_TEST(ComponentInterfaceTest, AddInputWithUserCallback) {
  auto state_data = std::make_shared<state_representation::CartesianState>("A");
  bool callback_triggered = false;
  auto user_callback = [&callback_triggered] {
    callback_triggered = true;
  };

  EXPECT_NO_THROW(this->component_->add_input("state", state_data, user_callback));
  EXPECT_FALSE(this->component_->inputs_.find("state") == this->component_->inputs_.end());
  auto callback =
      this->component_->inputs_.at("state")->template get_handler<modulo_core::EncodedState>()->get_callback();
  state_representation::CartesianState new_state("B");
  auto message = std::make_shared<modulo_core::EncodedState>();
  modulo_core::translators::write_message(*message, new_state, this->component_->node_clock_->get_clock()->now());
  EXPECT_STREQ(state_data->get_name().c_str(), "A");
  EXPECT_NO_THROW(callback(message));
  EXPECT_TRUE(callback_triggered);
  EXPECT_STREQ(state_data->get_name().c_str(), "B");

  // if the subscription callback fails (for example, the message is invalid), the user callback should be skipped
  message->data.clear();
  callback_triggered = false;
  EXPECT_NO_THROW(callback(message));
  EXPECT_FALSE(callback_triggered);
}

TYPED_TEST(ComponentInterfaceTest, AddService) {
  EXPECT_EQ(static_cast<int>(this->component_->empty_services_.size()), 0);
  EXPECT_EQ(static_cast<int>(this->component_->string_services_.size()), 0);

  auto empty_callback = []() -> ComponentServiceResponse {
    return {true, "test"};
  };
  EXPECT_NO_THROW(this->component_->add_service("empty", empty_callback));
  EXPECT_EQ(static_cast<int>(this->component_->empty_services_.size()), 1);
  EXPECT_NE(this->component_->empty_services_.find("empty"), this->component_->empty_services_.cend());

  auto string_callback = [](const std::string& payload) -> ComponentServiceResponse {
    return {true, payload};
  };
  EXPECT_NO_THROW(this->component_->add_service("string", string_callback));
  EXPECT_EQ(static_cast<int>(this->component_->string_services_.size()), 1);
  EXPECT_NE(this->component_->string_services_.find("string"), this->component_->string_services_.cend());

  // adding a service under an existing name should fail for either callback type but is exception safe
  EXPECT_NO_THROW(this->component_->add_service("empty", empty_callback));
  EXPECT_NO_THROW(this->component_->add_service("empty", string_callback));
  EXPECT_EQ(static_cast<int>(this->component_->empty_services_.size()), 1);
  EXPECT_EQ(static_cast<int>(this->component_->string_services_.size()), 1);

  EXPECT_NO_THROW(this->component_->add_service("string", empty_callback));
  EXPECT_NO_THROW(this->component_->add_service("string", string_callback));
  EXPECT_EQ(static_cast<int>(this->component_->empty_services_.size()), 1);
  EXPECT_EQ(static_cast<int>(this->component_->string_services_.size()), 1);

  // adding an empty service name should fail
  EXPECT_NO_THROW(this->component_->add_service("", empty_callback));
  EXPECT_NO_THROW(this->component_->add_service("", string_callback));
  EXPECT_EQ(static_cast<int>(this->component_->empty_services_.size()), 1);
  EXPECT_EQ(static_cast<int>(this->component_->string_services_.size()), 1);

  // adding a mangled service name should succeed under a sanitized name
  EXPECT_NO_THROW(this->component_->add_service("8_teEsTt_#1@3", empty_callback));
  EXPECT_EQ(static_cast<int>(this->component_->empty_services_.size()), 2);
  EXPECT_NE(this->component_->empty_services_.find("test_13"), this->component_->empty_services_.cend());
}

TYPED_TEST(ComponentInterfaceTest, CallEmptyService) {
  using namespace modulo_utils::testutils;
  auto empty_callback = []() -> ComponentServiceResponse {
    return {true, "test"};
  };
  EXPECT_NO_THROW(this->component_->add_service("empty", empty_callback));

  auto client = std::make_shared<ServiceClient<modulo_interfaces::srv::EmptyTrigger>>(
      rclcpp::NodeOptions(), "/" + std::string(this->component_->node_base_->get_name()) + "/empty");
  this->exec_->add_node(this->component_->node_base_);
  this->exec_->add_node(client);
  auto request = std::make_shared<modulo_interfaces::srv::EmptyTrigger::Request>();
  auto future = client->call_async(request);
  auto success = this->exec_->spin_until_future_complete(future, 1s);
  EXPECT_EQ(success, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "test");
}

TYPED_TEST(ComponentInterfaceTest, CallStringService) {
  using namespace modulo_utils::testutils;
  auto string_callback = [](const std::string& payload) -> ComponentServiceResponse {
    return {true, payload};
  };
  EXPECT_NO_THROW(this->component_->add_service("string", string_callback));

  auto client = std::make_shared<ServiceClient<modulo_interfaces::srv::StringTrigger>>(
      rclcpp::NodeOptions(), "/" + std::string(this->component_->node_base_->get_name()) + "/string");
  this->exec_->add_node(this->component_->node_base_);
  this->exec_->add_node(client);
  auto request = std::make_shared<modulo_interfaces::srv::StringTrigger::Request>();
  request->payload = "payload";
  auto future = client->call_async(request);
  auto success = this->exec_->spin_until_future_complete(future, 0.5s);
  EXPECT_EQ(success, rclcpp::FutureReturnCode::SUCCESS);
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "payload");
}

TYPED_TEST(ComponentInterfaceTest, CreateOutput) {
  auto data = std::make_shared<bool>(true);
  EXPECT_NO_THROW(this->component_->create_output(this->pub_type_, "test", data, "/topic", true, true));
  EXPECT_FALSE(this->component_->outputs_.find("test") == this->component_->outputs_.end());
  EXPECT_EQ(this->component_->template get_parameter_value<std::string>("test_topic"), "/topic");
  EXPECT_TRUE(this->component_->periodic_outputs_.at("test"));

  auto pub_interface = this->component_->outputs_.at("test");
  if (std::is_same<TypeParam, rclcpp::Node>::value) {
    EXPECT_EQ(pub_interface->get_type(), modulo_core::communication::PublisherType::PUBLISHER);
  } else if (std::is_same<TypeParam, rclcpp::Node>::value) {
    EXPECT_EQ(pub_interface->get_type(), modulo_core::communication::PublisherType::LIFECYCLE_PUBLISHER);
  }
  EXPECT_EQ(pub_interface->get_message_pair()->get_type(), modulo_core::communication::MessageType::BOOL);
  EXPECT_THROW(pub_interface->publish(), modulo_core::exceptions::CoreException);

  EXPECT_NO_THROW(this->component_->create_output(this->pub_type_, "8_teEsTt_#1@3", data, "", true, false));
  EXPECT_FALSE(this->component_->periodic_outputs_.at("test_13"));
  EXPECT_NO_THROW(this->component_->publish_output("8_teEsTt_#1@3"));
  EXPECT_NO_THROW(this->component_->publish_output("test_13"));
  EXPECT_THROW(this->component_->publish_output(""), modulo_core::exceptions::CoreException);
}

TYPED_TEST(ComponentInterfaceTest, TF) {
  this->component_->add_tf_broadcaster();
  this->component_->add_static_tf_broadcaster();
  this->component_->add_tf_listener();
  auto send_tf = state_representation::CartesianPose::Random("test", "world");
  EXPECT_NO_THROW(this->component_->send_transform(send_tf));
  sleep(1);
  state_representation::CartesianPose lookup_tf;
  EXPECT_NO_THROW(lookup_tf = this->component_->lookup_transform("test", "world"));
  auto identity = send_tf * lookup_tf.inverse();
  EXPECT_FLOAT_EQ(identity.data().norm(), 1.);
  EXPECT_FLOAT_EQ(abs(identity.get_orientation().w()), 1.);

  sleep(1);
  EXPECT_THROW(
      lookup_tf = this->component_->lookup_transform("test", "world", 0.9),
      modulo_core::exceptions::LookupTransformException);

  auto send_static_tf = state_representation::CartesianPose::Random("static_test", "world");
  EXPECT_NO_THROW(this->component_->send_static_transform(send_static_tf));
  EXPECT_THROW(
      auto throw_tf = this->component_->lookup_transform("dummy", "world"),
      modulo_core::exceptions::LookupTransformException);

  EXPECT_NO_THROW(lookup_tf = this->component_->lookup_transform("static_test", "world"));
  identity = send_static_tf * lookup_tf.inverse();
  EXPECT_FLOAT_EQ(identity.data().norm(), 1.);
  EXPECT_FLOAT_EQ(abs(identity.get_orientation().w()), 1.);

  std::vector<state_representation::CartesianPose> send_tfs;
  send_tfs.reserve(3);
  for (std::size_t idx = 0; idx < 3; ++idx) {
    send_tfs.emplace_back(state_representation::CartesianPose::Random("test_" + std::to_string(idx), "world"));
  }
  EXPECT_NO_THROW(this->component_->send_transforms(send_tfs));
  for (const auto& tf : send_tfs) {
    lookup_tf = this->component_->lookup_transform(tf.get_name(), tf.get_reference_frame());
    identity = tf * lookup_tf.inverse();
    EXPECT_FLOAT_EQ(identity.data().norm(), 1.);
    EXPECT_FLOAT_EQ(abs(identity.get_orientation().w()), 1.);
  }

  std::vector<state_representation::CartesianPose> send_static_tfs;
  send_static_tfs.reserve(3);
  for (std::size_t idx = 0; idx < 3; ++idx) {
    send_static_tfs.emplace_back(
        state_representation::CartesianPose::Random("test_static_" + std::to_string(idx), "world"));
  }
  EXPECT_NO_THROW(this->component_->send_static_transforms(send_static_tfs));
  for (const auto& tf : send_static_tfs) {
    lookup_tf = this->component_->lookup_transform(tf.get_name(), tf.get_reference_frame());
    identity = tf * lookup_tf.inverse();
    EXPECT_FLOAT_EQ(identity.data().norm(), 1.);
    EXPECT_FLOAT_EQ(abs(identity.get_orientation().w()), 1.);
  }
}

TYPED_TEST(ComponentInterfaceTest, GetSetQoS) {
  auto qos = rclcpp::QoS(5);
  this->component_->set_qos(qos);
  EXPECT_EQ(qos, this->component_->get_qos());
}

TYPED_TEST(ComponentInterfaceTest, AddTrigger) {
  EXPECT_NO_THROW(this->component_->add_trigger("trigger"));
  ASSERT_FALSE(
      std::find(this->component_->triggers_.cbegin(), this->component_->triggers_.cend(), "trigger")
      == this->component_->triggers_.cend());
  EXPECT_FALSE(this->component_->get_predicate("trigger"));
  EXPECT_NO_THROW(this->component_->trigger("trigger"));
}
}// namespace modulo_components
