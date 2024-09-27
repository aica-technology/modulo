#include <gtest/gtest.h>

#include "modulo_core/exceptions.hpp"
#include "modulo_core/translators/parameter_translators.hpp"

using namespace modulo_core::translators;

// Parameterized test fixture by type and expected value
// See also: http://www.ashermancinelli.com/gtest-type-val-param

template<typename T>
using ParamT = std::vector<std::tuple<T, state_representation::ParameterType>>;

static std::tuple<
    ParamT<bool>, ParamT<std::vector<bool>>, ParamT<int>, ParamT<std::vector<int>>, ParamT<double>,
    ParamT<std::vector<double>>, ParamT<std::string>, ParamT<std::vector<std::string>>>
    parameter_test_cases{
        {
            std::make_tuple(true, state_representation::ParameterType::BOOL),
            std::make_tuple(false, state_representation::ParameterType::BOOL),
        },
        {
            std::make_tuple(std::vector<bool>({true, false, true}), state_representation::ParameterType::BOOL_ARRAY),
        },
        {
            std::make_tuple(0, state_representation::ParameterType::INT),
            std::make_tuple(1, state_representation::ParameterType::INT),
        },
        {
            std::make_tuple(std::vector<int>({1, 2, 3}), state_representation::ParameterType::INT_ARRAY),
        },
        {
            std::make_tuple(1.0, state_representation::ParameterType::DOUBLE),
        },
        {
            std::make_tuple(std::vector<double>({1.0, 2.0, 3.0}), state_representation::ParameterType::DOUBLE_ARRAY),
        },
        {
            std::make_tuple("test", state_representation::ParameterType::STRING),
        },
        {
            std::make_tuple(
                std::vector<std::string>({"1", "2", "3"}), state_representation::ParameterType::STRING_ARRAY),
        },
    };

template<typename T>
class ParameterTranslationTest : public testing::Test {
public:
  ParameterTranslationTest() : test_cases_{std::get<ParamT<T>>(parameter_test_cases)} {}

protected:
  ParamT<T> test_cases_;
};
TYPED_TEST_SUITE_P(ParameterTranslationTest);

TYPED_TEST_P(ParameterTranslationTest, Write) {
  for (auto const& test_case : this->test_cases_) {
    auto param = std::make_shared<state_representation::Parameter<TypeParam>>("test");
    rclcpp::Parameter ros_param;
    EXPECT_NO_THROW(ros_param = write_parameter(param));
    EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET);
    param = state_representation::make_shared_parameter("test", std::get<0>(test_case));
    EXPECT_NO_THROW(ros_param = write_parameter(param));
    EXPECT_EQ(ros_param, rclcpp::Parameter("test", std::get<0>(test_case)));
  }
}

TYPED_TEST_P(ParameterTranslationTest, ReadAndReWrite) {
  auto ros_param = rclcpp::Parameter("test");
  EXPECT_THROW(read_parameter(ros_param), modulo_core::exceptions::ParameterTranslationException);
  for (auto const& [value, type] : this->test_cases_) {
    ros_param = rclcpp::Parameter("test", value);
    std::shared_ptr<state_representation::ParameterInterface> param;
    ASSERT_NO_THROW(param = read_parameter(ros_param));
    EXPECT_EQ(param->get_name(), ros_param.get_name());
    EXPECT_EQ(param->get_parameter_type(), type);
    EXPECT_EQ(param->get_parameter_value<TypeParam>(), value);

    rclcpp::Parameter new_ros_param;
    ASSERT_NO_THROW(new_ros_param = write_parameter(param));
    EXPECT_EQ(new_ros_param, ros_param);
  }
}

TYPED_TEST_P(ParameterTranslationTest, ConstRead) {
  for (auto const& [value, type] : this->test_cases_) {
    auto param = state_representation::make_shared_parameter("test", value);
    rclcpp::Parameter ros_param("test", value);

    std::shared_ptr<state_representation::ParameterInterface> new_param;
    EXPECT_NO_THROW(new_param = read_parameter_const(ros_param, param));
    EXPECT_EQ(new_param->get_name(), param->get_name());
    EXPECT_EQ(new_param->get_parameter_type(), type);
    EXPECT_EQ(new_param->get_parameter_type(), param->get_parameter_type());
    EXPECT_EQ(new_param->get_parameter_value<TypeParam>(), value);

    // the const reader constructs a new parameter, so it should not be possible to reference or modify the original
    new_param->set_name("other");
    EXPECT_NE(new_param->get_name(), param->get_name());
  }
}

TYPED_TEST_P(ParameterTranslationTest, NonConstRead) {
  for (auto const& [value, type] : this->test_cases_) {
    auto param = std::make_shared<state_representation::Parameter<TypeParam>>("test");
    rclcpp::Parameter ros_param("test", value);

    // make a copy of the pointer referencing the parameter
    auto param_ref = param;
    ASSERT_NO_THROW(read_parameter(ros_param, param));
    EXPECT_EQ(param->get_name(), ros_param.get_name());
    EXPECT_EQ(param->get_parameter_type(), type);
    EXPECT_EQ(param->get_value(), value);

    // The reference should be preserved
    EXPECT_EQ(param_ref->get_name(), ros_param.get_name());
    EXPECT_EQ(param_ref->get_parameter_type(), type);
    EXPECT_EQ(param_ref->get_value(), value);
  }
}

REGISTER_TYPED_TEST_SUITE_P(ParameterTranslationTest, Write, ReadAndReWrite, ConstRead, NonConstRead);

using ParameterTestTypes = testing::Types<
    bool, std::vector<bool>, int, std::vector<int>, double, std::vector<double>, std::string, std::vector<std::string>>;
INSTANTIATE_TYPED_TEST_SUITE_P(TestPrefix, ParameterTranslationTest, ParameterTestTypes);
