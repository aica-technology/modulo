#include <gtest/gtest.h>

#include "modulo_core/translators/parameter_translators.hpp"

using namespace modulo_core::translators;

TEST(ParameterEigenTranslationTest, EigenVector) {
  Eigen::VectorXd vec = Eigen::VectorXd::Random(5);
  auto param = state_representation::make_shared_parameter("vector", vec);

  // writing the eigen parameter converts it to a double array
  auto ros_param = write_parameter(param);
  EXPECT_EQ(ros_param.get_name(), param->get_name());
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  std::vector<double> ros_vec;
  EXPECT_NO_THROW(ros_vec = ros_param.as_double_array());
  EXPECT_EQ(ros_vec.size(), static_cast<std::size_t>(vec.size()));
  for (std::size_t ind = 0; ind < ros_vec.size(); ++ind) {
    EXPECT_FLOAT_EQ(ros_vec.at(ind), vec(ind));
  }

  // reading the parameter with no context does not retain the eigen type
  auto new_param = read_parameter(ros_param);
  EXPECT_EQ(new_param->get_name(), ros_param.get_name());
  EXPECT_EQ(new_param->get_parameter_type(), state_representation::ParameterType::DOUBLE_ARRAY);
  EXPECT_EQ(new_param->get_parameter_value<std::vector<double>>(), ros_vec);

  // reading the parameter with a const reference to an existing parameter preserves the eigen type
  new_param = read_parameter_const(ros_param, param);
  EXPECT_EQ(new_param->get_name(), ros_param.get_name());
  EXPECT_EQ(new_param->get_parameter_type(), state_representation::ParameterType::VECTOR);
  Eigen::VectorXd new_vec;
  EXPECT_NO_THROW(new_vec = new_param->get_parameter_value<Eigen::VectorXd>());
  EXPECT_EQ(new_vec, vec);
}

TEST(ParameterEigenTranslationTest, EigenMatrix) {
  Eigen::MatrixXd mat = Eigen::MatrixXd::Random(3, 4);
  auto param = state_representation::make_shared_parameter("matrix", mat);

  // writing the eigen parameter converts it to a double array
  auto ros_param = write_parameter(param);
  EXPECT_EQ(ros_param.get_name(), param->get_name());
  EXPECT_EQ(ros_param.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  std::vector<double> ros_vec;
  EXPECT_NO_THROW(ros_vec = ros_param.as_double_array());
  EXPECT_EQ(ros_vec.size(), static_cast<std::size_t>(mat.size()));
  for (std::size_t ind = 0; ind < ros_vec.size(); ++ind) {
    EXPECT_FLOAT_EQ(ros_vec.at(ind), mat(ind));
  }

  // reading the parameter with no context does not retain the eigen type
  auto new_param = read_parameter(ros_param);
  EXPECT_EQ(new_param->get_name(), ros_param.get_name());
  EXPECT_EQ(new_param->get_parameter_type(), state_representation::ParameterType::DOUBLE_ARRAY);
  EXPECT_EQ(new_param->get_parameter_value<std::vector<double>>(), ros_vec);

  // reading the parameter with a const reference to an existing parameter preserves the eigen type
  new_param = read_parameter_const(ros_param, param);
  EXPECT_EQ(new_param->get_name(), ros_param.get_name());
  EXPECT_EQ(new_param->get_parameter_type(), state_representation::ParameterType::MATRIX);
  Eigen::MatrixXd new_mat;
  EXPECT_NO_THROW(new_mat = new_param->get_parameter_value<Eigen::MatrixXd>());
  EXPECT_EQ(new_mat, mat);
}
