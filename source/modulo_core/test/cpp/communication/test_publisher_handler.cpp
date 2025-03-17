#include <gtest/gtest.h>

#include <rclcpp/node.hpp>

#include "modulo_core/communication/MessagePair.hpp"
#include "modulo_core/communication/PublisherHandler.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace modulo_core::communication;

template<typename MsgT, typename DataT>
static void test_publisher_interface(const std::shared_ptr<rclcpp::Node>& node, const DataT& value) {
  // create message pair
  auto data = std::make_shared<DataT>(value);
  auto message_pair = std::make_shared<MessagePair<MsgT, DataT>>(data, node->get_clock());

  // create publisher handler
  auto publisher = node->create_publisher<MsgT>("topic", 10);
  auto publisher_handler =
      std::make_shared<PublisherHandler<rclcpp::Publisher<MsgT>, MsgT>>(PublisherType::PUBLISHER, publisher);

  // use in publisher interface
  std::shared_ptr<PublisherInterface> publisher_interface(publisher_handler);
  EXPECT_THROW(publisher_interface->publish(), modulo_core::exceptions::CoreException);
  publisher_interface->set_message_pair(message_pair);
  EXPECT_NO_THROW(publisher_interface->publish());

  auto publisher_interface_ptr = publisher_handler->create_publisher_interface(message_pair);
  EXPECT_NO_THROW(publisher_interface->publish());
}

class PublisherTest : public ::testing::Test {
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

protected:
  void SetUp() { node = std::make_shared<rclcpp::Node>("test_node"); }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(PublisherTest, BasicTypes) {
  test_publisher_interface<std_msgs::msg::Bool, bool>(node, true);
  test_publisher_interface<std_msgs::msg::Float64, double>(node, 0.1);
  test_publisher_interface<std_msgs::msg::Float64MultiArray, std::vector<double>>(node, {0.1, 0.2, 0.3});
  test_publisher_interface<std_msgs::msg::Int32, int>(node, 1);
  test_publisher_interface<std_msgs::msg::String, std::string>(node, "this");
}

TEST_F(PublisherTest, EncodedState) {
  // create message pair
  auto data =
      std::make_shared<state_representation::CartesianState>(state_representation::CartesianState::Random("test"));
  auto message_pair =
      std::make_shared<MessagePair<modulo_core::EncodedState, state_representation::State>>(data, node->get_clock());

  // create publisher handler
  auto publisher = node->create_publisher<modulo_core::EncodedState>("topic", 10);
  auto publisher_handler =
      std::make_shared<PublisherHandler<rclcpp::Publisher<modulo_core::EncodedState>, modulo_core::EncodedState>>(
          PublisherType::PUBLISHER, publisher);

  // use in publisher interface
  std::shared_ptr<PublisherInterface> publisher_interface(publisher_handler);
  EXPECT_THROW(publisher_interface->publish(), modulo_core::exceptions::CoreException);
  publisher_interface->set_message_pair(message_pair);
  EXPECT_NO_THROW(publisher_interface->publish());

  publisher_interface = publisher_handler->create_publisher_interface(message_pair);
  EXPECT_NO_THROW(publisher_interface->publish());
}

TEST_F(PublisherTest, CustomTypes) {
  test_publisher_interface<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(node, sensor_msgs::msg::Image());
  test_publisher_interface<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>(node, sensor_msgs::msg::Imu());
  test_publisher_interface<std_msgs::msg::Float64MultiArray, std_msgs::msg::Float64MultiArray>(
      node, std_msgs::msg::Float64MultiArray());
}
