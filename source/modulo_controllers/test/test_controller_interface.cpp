#include "gmock/gmock.h"

#include <chrono>

#include <lifecycle_msgs/msg/state.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/joint/JointState.hpp>

#include "modulo_controllers/ControllerInterface.hpp"
#include "test_modulo_controllers/communication_nodes.hpp"

using namespace modulo_controllers;
using namespace state_representation;
using namespace std::chrono_literals;

class FriendControllerInterface : public ControllerInterface {
public:
  using ControllerInterface::add_input;
  using ControllerInterface::add_output;
  using ControllerInterface::read_input;
  using ControllerInterface::write_output;

private:
  controller_interface::return_type evaluate(const rclcpp::Time&, const std::chrono::nanoseconds&) {
    return controller_interface::return_type::OK;
  }
};

using BoolT = std::tuple<bool, std_msgs::msg::Bool>;
using DoubleT = std::tuple<double, std_msgs::msg::Float64>;
using DoubleVecT = std::tuple<std::vector<double>, std_msgs::msg::Float64MultiArray>;
using IntT = std::tuple<int, std_msgs::msg::Int32>;
using StringT = std::tuple<std::string, std_msgs::msg::String>;
using CartesianStateT = std::tuple<CartesianState, modulo_core::EncodedState>;
using JointStateT = std::tuple<JointState, modulo_core::EncodedState>;

template<typename T>
T write_std_msg(const T& message_data) {
  auto copy = message_data;
  std::get<1>(copy).data = std::get<0>(message_data);
  return copy;
}

template<typename T>
T write_state_msg(const T& message_data) {
  auto copy = message_data;
  modulo_core::translators::write_message(
      std::get<1>(copy), make_shared_state(std::get<0>(message_data)), rclcpp::Clock().now());
  return copy;
}

template<typename T>
T read_std_msg(const T& message_data) {
  auto copy = message_data;
  std::get<0>(copy) = std::get<1>(message_data).data;
  return copy;
}

template<typename T>
T read_state_msg(const T& message_data) {
  auto copy = message_data;
  modulo_core::translators::read_message(std::get<0>(copy), std::get<1>(message_data));
  return copy;
}

template<typename T>
bool std_msg_equal(const T& sent, const T& received) {
  return std::get<0>(sent) == std::get<0>(received);
}

template<typename T>
bool encoded_state_equal(const T& sent, const T& received) {
  auto equal = std::get<0>(sent).get_name() == std::get<0>(received).get_name();
  return equal && std::get<0>(sent).data().isApprox(std::get<0>(received).data());
}

template<typename T>
using SignalT = std::vector<std::tuple<T, std::function<T(T)>, std::function<T(T)>, std::function<bool(T, T)>>>;

static std::tuple<
    SignalT<BoolT>, SignalT<DoubleT>, SignalT<DoubleVecT>, SignalT<IntT>, SignalT<StringT>, SignalT<CartesianStateT>,
    SignalT<JointStateT>>
    signal_test_cases{
        {std::make_tuple(
            std::make_tuple(true, std_msgs::msg::Bool()), write_std_msg<BoolT>, read_std_msg<BoolT>,
            std_msg_equal<BoolT>)},
        {std::make_tuple(
            std::make_tuple(1.0, std_msgs::msg::Float64()), write_std_msg<DoubleT>, read_std_msg<DoubleT>,
            std_msg_equal<DoubleT>)},
        {std::make_tuple(
            std::make_tuple(std::vector<double>({1.0, 2.0, 3.0}), std_msgs::msg::Float64MultiArray()),
            write_std_msg<DoubleVecT>, read_std_msg<DoubleVecT>, std_msg_equal<DoubleVecT>)},
        {std::make_tuple(
            std::make_tuple(2, std_msgs::msg::Int32()), write_std_msg<IntT>, read_std_msg<IntT>, std_msg_equal<IntT>)},
        {std::make_tuple(
            std::make_tuple("test", std_msgs::msg::String()), write_std_msg<StringT>, read_std_msg<StringT>,
            std_msg_equal<StringT>)},
        {std::make_tuple(
            std::make_tuple(CartesianState::Random("test"), modulo_core::EncodedState()),
            write_state_msg<CartesianStateT>, read_state_msg<CartesianStateT>, encoded_state_equal<CartesianStateT>)},
        {std::make_tuple(
            std::make_tuple(JointState::Random("test", 3), modulo_core::EncodedState()), write_state_msg<JointStateT>,
            read_state_msg<JointStateT>, encoded_state_equal<JointStateT>)}};

template<typename T>
class ControllerInterfaceTest : public ::testing::Test {
public:
  ControllerInterfaceTest() : test_cases_{std::get<SignalT<T>>(signal_test_cases)} {}
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() { interface_ = std::make_unique<FriendControllerInterface>(); }
  void TearDown() { interface_.reset(nullptr); }

  void init() {
    const auto result = interface_->init("controller_interface", "", 0, "", interface_->define_custom_node_options());
    ASSERT_EQ(result, controller_interface::return_type::OK);
    interface_->get_node()->set_parameter({"hardware_name", "test"});
    interface_->get_node()->set_parameter({"input_validity_period", 0.1});
  }

protected:
  std::unique_ptr<FriendControllerInterface> interface_;
  SignalT<T> test_cases_;
};

TYPED_TEST_CASE_P(ControllerInterfaceTest);

TYPED_TEST_P(ControllerInterfaceTest, ConfigureErrorTest) {
  ASSERT_THROW(this->interface_->on_configure(rclcpp_lifecycle::State()), std::exception);

  this->init();
  ASSERT_NO_THROW(this->interface_->on_configure(rclcpp_lifecycle::State()));
}

TYPED_TEST_P(ControllerInterfaceTest, InputTest) {
  using DataT = typename std::tuple_element<0, TypeParam>::type;
  using MsgT = typename std::tuple_element<1, TypeParam>::type;

  this->init();
  this->interface_->template add_input<DataT>("input", "/input");
  auto node_state = this->interface_->get_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = this->interface_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::Node test_node("test_node");
  auto publisher = test_node.create_publisher<MsgT>("/input", rclcpp::SystemDefaultsQoS());

  for (auto [message_data, write_func, read_func, validation_func] : this->test_cases_) {
    message_data = write_func(message_data);
    auto message = std::get<1>(message_data);
    publisher->publish(message);
    rclcpp::spin_some(this->interface_->get_node()->get_node_base_interface());
    auto input = this->interface_->template read_input<DataT>("input");
    ASSERT_TRUE(input);
    EXPECT_TRUE(validation_func(message_data, std::make_tuple(*input, message)));
    std::this_thread::sleep_for(100ms);
    ASSERT_FALSE(this->interface_->template read_input<DataT>("input"));
  }
}

TYPED_TEST_P(ControllerInterfaceTest, OutputTest) {
  using DataT = typename std::tuple_element<0, TypeParam>::type;
  using MsgT = typename std::tuple_element<1, TypeParam>::type;

  this->init();
  this->interface_->template add_output<DataT>("output", "/output");
  auto node_state = this->interface_->get_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = this->interface_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto test_node = SubscriptionNode<MsgT>("/output");
  for (auto [message_data, write_func, read_func, validation_func] : this->test_cases_) {
    auto data = std::get<0>(message_data);
    this->interface_->template write_output<DataT>("output", data);
    // rclcpp::spin_some(this->interface_->get_node()->get_node_base_interface());
    auto return_code =
        rclcpp::spin_until_future_complete(test_node.get_node_base_interface(), test_node.get_sub_future(), 200ms);
    ASSERT_EQ(return_code, rclcpp::FutureReturnCode::SUCCESS);
    auto received_message_data = read_func(std::make_tuple(data, test_node.message));
    ASSERT_TRUE(validation_func(message_data, received_message_data));
  }
}

REGISTER_TYPED_TEST_CASE_P(ControllerInterfaceTest, ConfigureErrorTest, InputTest, OutputTest);

typedef ::testing::Types<BoolT, DoubleT, DoubleVecT, IntT, StringT, CartesianStateT, JointStateT> SignalTypes;
INSTANTIATE_TYPED_TEST_CASE_P(TestPrefix, ControllerInterfaceTest, SignalTypes);
