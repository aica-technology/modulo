#include <gtest/gtest.h>

#include "test_modulo_components/component_public_interfaces.hpp"

namespace modulo_components {

template<class NodeT>
class EmptyParameterInterface : public ComponentInterfacePublicInterface {
public:
  explicit EmptyParameterInterface(
      const std::shared_ptr<NodeT>& node, bool allow_empty = true, bool add_parameter = true,
      bool empty_parameter = true
  ) : ComponentInterfacePublicInterface(node), allow_empty_(allow_empty) {
    if (add_parameter) {
      if (empty_parameter) {
        this->add_parameter(std::make_shared<Parameter<std::string>>("name"), "Name parameter");
      } else {
        this->add_parameter(std::make_shared<Parameter<std::string>>("name", "test"), "Name parameter");
      }
    }
    this->add_parameter(std::make_shared<Parameter<double>>("value", 0.0), "Value parameter");
  };

private:
  bool
  on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override {
    if (parameter->get_name() == "name") {
      if (parameter->is_empty()) {
        return this->allow_empty_;
      } else if (parameter->get_parameter_value<std::string>().empty()) {
        RCLCPP_ERROR(this->node_logging_->get_logger(), "Provide a non empty value for parameter 'name'");
        return false;
      }
    } else if (parameter->get_name() == "value") {
      auto value = parameter->get_parameter_value<double>();
      if (value < 0) {
        parameter->set_parameter_value<double>(std::abs(value));
      }
      return true;
    }
    return true;
  }

  bool allow_empty_;
};

template<class NodeT>
class ComponentInterfaceEmptyParameterTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    this->component_ = std::make_shared<EmptyParameterInterface<NodeT>>(
        std::make_shared<NodeT>("EmptyParameterInterface", rclcpp::NodeOptions()));
  }

  std::shared_ptr<EmptyParameterInterface<NodeT>> component_;
};
using NodeTypes = ::testing::Types<rclcpp::Node, rclcpp_lifecycle::LifecycleNode>;
TYPED_TEST_SUITE(ComponentInterfaceEmptyParameterTest, NodeTypes);

TYPED_TEST(ComponentInterfaceEmptyParameterTest, NotAllowEmptyOnConstruction) {
  auto node = std::make_shared<TypeParam>("EmptyParameterComponent", rclcpp::NodeOptions());
  EXPECT_THROW(std::make_shared<EmptyParameterInterface<TypeParam>>(node, false),
               modulo_utils::exceptions::ParameterException);
}

TYPED_TEST(ComponentInterfaceEmptyParameterTest, NotAllowEmpty) {
  std::shared_ptr<EmptyParameterInterface<TypeParam>> component;
  auto node = std::make_shared<TypeParam>("EmptyParameterComponent", rclcpp::NodeOptions());
  component = std::make_shared<EmptyParameterInterface<TypeParam>>(node, false, false);
  EXPECT_THROW(component->add_parameter(std::make_shared<Parameter<std::string>>("name"), "Test parameter"),
               modulo_utils::exceptions::ParameterException);
  EXPECT_THROW(auto param = component->get_parameter("name"), modulo_utils::exceptions::ParameterException);
  EXPECT_THROW(component->get_ros_parameter("name"), rclcpp::exceptions::ParameterNotDeclaredException);
}

TYPED_TEST(ComponentInterfaceEmptyParameterTest, AllowEmpty) {
  auto node = std::make_shared<TypeParam>("EmptyParameterInterface", rclcpp::NodeOptions());
  auto component = std::make_shared<EmptyParameterInterface<TypeParam>>(node, true, false);
  EXPECT_NO_THROW(component->add_parameter(std::make_shared<Parameter<std::string>>("name"), "Test parameter"));
  EXPECT_EQ(component->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_TRUE(component->get_parameter("name")->is_empty());
  EXPECT_EQ(component->get_ros_parameter("name").get_type(), rclcpp::PARAMETER_NOT_SET);
}

TYPED_TEST(ComponentInterfaceEmptyParameterTest, ValidateEmptyParameter) {
  // component comes with empty parameter 'name'
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_TRUE(this->component_->get_parameter("name")->is_empty());
  auto ros_param = rclcpp::Parameter();
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET);

  // Trying to overwrite a parameter is not possible
  this->component_->add_parameter(std::make_shared<Parameter<bool>>("name"), "Test parameter");
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_TRUE(this->component_->get_parameter("name")->is_empty());
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET);

  // Set parameter value from ROS interface should update ROS parameter type
  this->component_->set_ros_parameter({"name", "test"});
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_EQ(this->component_->get_parameter("name")->template get_parameter_value<std::string>(), "test");
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(ros_param.template get_value<std::string>(), "test");

  // Set parameter value from component interface
  this->component_->template set_parameter_value<std::string>("name", "again");
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_EQ(this->component_->get_parameter("name")->template get_parameter_value<std::string>(), "again");
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(ros_param.template get_value<std::string>(), "again");

  // Setting it with empty value should be rejected in parameter evaluation
  this->component_->template set_parameter_value<std::string>("name", "");
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_EQ(this->component_->get_parameter("name")->template get_parameter_value<std::string>(), "again");
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(ros_param.template get_value<std::string>(), "again");

  // Setting it with empty value should be rejected in parameter evaluation
  this->component_->set_ros_parameter({"name", ""});
  EXPECT_EQ(this->component_->get_parameter("name")->get_parameter_type(), ParameterType::STRING);
  EXPECT_EQ(this->component_->get_parameter("name")->template get_parameter_value<std::string>(), "again");
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("name"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(ros_param.template get_value<std::string>(), "again");

  // TODO clarify that behavior somewhere
  // Setting a parameter with type NOT_SET undeclares that parameter
  this->component_->set_ros_parameter(rclcpp::Parameter("name", rclcpp::ParameterValue{}));
  EXPECT_THROW(this->component_->get_ros_parameter("name"), rclcpp::exceptions::ParameterNotDeclaredException);
}

TYPED_TEST(ComponentInterfaceEmptyParameterTest, ChangeParameterType) {
  // Add parameter from component interface
  this->component_->add_parameter(std::make_shared<Parameter<int>>("int"), "Test parameter");
  EXPECT_TRUE(this->component_->node_parameters_->describe_parameters({"int"}).at(0).dynamic_typing);
  this->component_->template set_parameter_value<int>("int", 1);
  EXPECT_EQ(this->component_->get_parameter("int")->get_parameter_type(), ParameterType::INT);
  EXPECT_EQ(this->component_->get_parameter("int")->template get_parameter_value<int>(), 1);
  auto ros_param = rclcpp::Parameter();
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("int"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(ros_param.template get_value<int>(), 1);

  // Set parameter value from component interface with different type should not work
  this->component_->template set_parameter_value<double>("int", 2.0);
  EXPECT_EQ(this->component_->get_parameter("int")->get_parameter_type(), ParameterType::INT);
  EXPECT_EQ(this->component_->get_parameter("int")->template get_parameter_value<int>(), 1);
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("int"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(ros_param.template get_value<int>(), 1);

  // Set parameter value from ROS interface with different type should not work
  auto result = this->component_->set_ros_parameter(rclcpp::Parameter("int", 2.0));
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(this->component_->get_parameter("int")->get_parameter_type(), ParameterType::INT);
  EXPECT_EQ(this->component_->get_parameter("int")->template get_parameter_value<int>(), 1);
  EXPECT_NO_THROW(ros_param = this->component_->get_ros_parameter("int"););
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(ros_param.template get_value<int>(), 1);
}

TYPED_TEST(ComponentInterfaceEmptyParameterTest, ParameterOverrides) {
  // Construction with not allowing empty parameters but providing the parameter override should succeed
  auto node = std::make_shared<TypeParam>(
      "EmptyParameterInterface", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("name", "test")}));
  EXPECT_NO_THROW(std::make_shared<EmptyParameterInterface<TypeParam>>(node, false));
}

TYPED_TEST(ComponentInterfaceEmptyParameterTest, ParameterOverridesEmpty) {
  // Construction with not allowing empty parameters and providing an uninitialized parameter override should not succeed
  auto node = std::make_shared<TypeParam>(
      "EmptyParameterInterface", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("name")}));
  EXPECT_THROW(std::make_shared<EmptyParameterInterface<TypeParam>>(node, false, true, false),
               modulo_utils::exceptions::ParameterException);
}

TYPED_TEST(ComponentInterfaceEmptyParameterTest, ValueParameter) {
  EXPECT_EQ(this->component_->get_parameter("value")->template get_parameter_value<double>(), 0);
  this->component_->template set_parameter_value<double>("value", 1);
  EXPECT_EQ(this->component_->get_parameter("value")->template get_parameter_value<double>(), 1);
  EXPECT_EQ(this->component_->get_ros_parameter("value").as_double(), 1);
  this->component_->template set_parameter_value<double>("value", -2);
  EXPECT_EQ(this->component_->get_parameter("value")->template get_parameter_value<double>(), 2);
  EXPECT_EQ(this->component_->get_ros_parameter("value").as_double(), 2);

  this->component_->set_ros_parameter({"value", 3.0});
  EXPECT_EQ(this->component_->get_parameter("value")->template get_parameter_value<double>(), 3);
  EXPECT_EQ(this->component_->get_ros_parameter("value").as_double(), 3);
  this->component_->set_ros_parameter({"value", -4.0});
  EXPECT_EQ(this->component_->get_parameter("value")->template get_parameter_value<double>(), 4);
  EXPECT_EQ(this->component_->get_ros_parameter("value").as_double(), 4);
}
} // namespace modulo_components
