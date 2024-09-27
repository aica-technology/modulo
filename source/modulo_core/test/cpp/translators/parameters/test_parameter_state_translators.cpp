#include <gtest/gtest.h>

#include <clproto.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "modulo_core/exceptions.hpp"
#include "modulo_core/translators/parameter_translators.hpp"

using namespace modulo_core::translators;

// Parameterized test fixture by type and expected value
// See also: http://www.ashermancinelli.com/gtest-type-val-param

static std::tuple<
    state_representation::CartesianState, state_representation::CartesianPose, state_representation::JointState,
    state_representation::JointPositions>
    parameter_state_test_cases{
        state_representation::CartesianState::Random("frame", "reference"),
        state_representation::CartesianPose::Random("frame", "reference"),
        state_representation::JointState::Random("robot", 3),
        state_representation::JointPositions::Random("robot", 3),
    };

template<typename T>
class ParameterStateTranslationTest : public testing::Test {
public:
  ParameterStateTranslationTest() : test_state_{std::get<T>(parameter_state_test_cases)} {}

protected:
  void check_state_equality(const T& new_state) {
    EXPECT_STREQ(new_state.get_name().c_str(), this->test_state_.get_name().c_str());
    EXPECT_EQ(new_state.get_type(), this->test_state_.get_type());
    EXPECT_NEAR(state_representation::dist(new_state, this->test_state_), 0.0, 1e-9);
  }

  T test_state_;
};
TYPED_TEST_SUITE_P(ParameterStateTranslationTest);

TEST(ParameterStateTranslationTest, ConstReadInvalid) {
  using namespace state_representation;
  auto param = make_shared_parameter("test", CartesianState("test"));
  auto ros_param = write_parameter(param);

  auto expected_param = make_shared_parameter("test", JointState("test", 3));

  EXPECT_THROW(
      auto new_param = read_parameter_const(ros_param, expected_param),
      modulo_core::exceptions::ParameterTranslationException);
}

TYPED_TEST_P(ParameterStateTranslationTest, Write) {
  // ROS parameter of a state parameter
  auto param = state_representation::make_shared_parameter("test", this->test_state_);
  auto ros_param = write_parameter(param);
  EXPECT_STREQ(ros_param.get_name().c_str(), param->get_name().c_str());
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  // string equality of encoding cannot be checked because the timestamp is not preserved
  // EXPECT_STREQ(ros_param_.as_string().c_str(), clproto::to_json<T>(this->test_state_).c_str())
  this->check_state_equality(clproto::from_json<TypeParam>(ros_param.as_string()));
}

TYPED_TEST_P(ParameterStateTranslationTest, ReadAndReWrite) {
  std::string json = clproto::to_json<TypeParam>(this->test_state_);
  auto ros_param = rclcpp::Parameter("test", json);
  std::shared_ptr<state_representation::ParameterInterface> param;
  ASSERT_NO_THROW(param = read_parameter(ros_param));
  EXPECT_EQ(param->get_name(), ros_param.get_name());
  EXPECT_EQ(param->get_parameter_type(), state_representation::ParameterType::STATE);
  EXPECT_EQ(param->get_parameter_state_type(), this->test_state_.get_type());

  ASSERT_NO_THROW(this->check_state_equality(param->get_parameter_value<TypeParam>()));

  rclcpp::Parameter new_ros_param;
  ASSERT_NO_THROW(new_ros_param = write_parameter(param));
  EXPECT_STREQ(new_ros_param.get_name().c_str(), ros_param.get_name().c_str());
  EXPECT_EQ(new_ros_param.get_type(), ros_param.get_type());
  EXPECT_NO_THROW(this->check_state_equality(clproto::from_json<TypeParam>(new_ros_param.as_string())));
}

TYPED_TEST_P(ParameterStateTranslationTest, ConstRead) {
  auto param = state_representation::make_shared_parameter("test", this->test_state_);
  auto ros_param = write_parameter(param);

  std::shared_ptr<state_representation::ParameterInterface> new_param;
  EXPECT_NO_THROW(new_param = read_parameter_const(ros_param, param));
  EXPECT_EQ(new_param->get_name(), param->get_name());
  EXPECT_EQ(new_param->get_parameter_type(), state_representation::ParameterType::STATE);
  EXPECT_EQ(new_param->get_parameter_type(), param->get_parameter_type());
  EXPECT_EQ(new_param->get_parameter_state_type(), this->test_state_.get_type());

  this->check_state_equality(new_param->get_parameter_value<TypeParam>());
}

REGISTER_TYPED_TEST_SUITE_P(ParameterStateTranslationTest, Write, ReadAndReWrite, ConstRead);

using ParameterStateTestTypes = testing::Types<
    state_representation::CartesianState, state_representation::CartesianPose, state_representation::JointState,
    state_representation::JointPositions>;
INSTANTIATE_TYPED_TEST_SUITE_P(TestPrefix, ParameterStateTranslationTest, ParameterStateTestTypes);
