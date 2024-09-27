#include <gtest/gtest.h>

#include "test_modulo_core/communication_nodes.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace modulo_core::communication;

class CommunicationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    clock_ = std::make_shared<rclcpp::Clock>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  template<typename MsgT>
  void add_nodes(
      const std::string& topic_name, const std::shared_ptr<MessagePairInterface>& pub_message,
      const std::shared_ptr<MessagePairInterface>& sub_message) {
    pub_node_ = std::make_shared<MinimalPublisher<MsgT>>(topic_name, pub_message);
    sub_node_ = std::make_shared<MinimalSubscriber<MsgT>>(topic_name, sub_message);
    exec_->add_node(pub_node_);
    exec_->add_node(sub_node_);
  }

  void clear_nodes() {
    exec_->remove_node(pub_node_);
    exec_->remove_node(sub_node_);
    pub_node_.reset();
    sub_node_.reset();
  }

  template<typename MsgT, typename DataT>
  void communicate(const DataT& initial_value, const DataT& new_value) {
    auto pub_data = std::make_shared<DataT>(new_value);
    auto pub_message = make_shared_message_pair(pub_data, this->clock_);
    auto sub_data = std::make_shared<DataT>(initial_value);
    auto sub_message = make_shared_message_pair(sub_data, this->clock_);
    this->add_nodes<MsgT>("/test_topic", pub_message, sub_message);

    this->exec_->template spin_until_future_complete(
        std::dynamic_pointer_cast<MinimalSubscriber<MsgT>>(this->sub_node_)->received_future, 500ms);

    EXPECT_EQ(*pub_data, *sub_data);
    this->clear_nodes();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  std::shared_ptr<rclcpp::Node> sub_node_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(CommunicationTest, BasicTypes) {
  this->communicate<std_msgs::msg::Bool, bool>(false, true);
  this->communicate<std_msgs::msg::Float64, double>(1.0, 2.0);
  this->communicate<std_msgs::msg::Float64MultiArray, std::vector<double>>({1.0, 2.0}, {3.0, 4.0});
  this->communicate<std_msgs::msg::Int32, int>(1, 2);
  this->communicate<std_msgs::msg::String, std::string>("this", "that");
}

// TEST_F(CommunicationTest, CustomTypes) { // TODO: uncomment when Subscription handler (needed by MinimalSubscriber) supports generic types
//   sensor_msgs::msg::Image initial_image;
//   initial_image.height = 480;
//   sensor_msgs::msg::Image new_image;
//   new_image.height = 320;
//   this->communicate<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(initial_image, new_image);

//   sensor_msgs::msg::Imu initial_imu;
//   initial_imu.linear_acceleration.x = 1.0;
//   sensor_msgs::msg::Imu new_imu;
//   new_imu.linear_acceleration.x = 0.5;
//   this->communicate<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu>(initial_imu, new_imu);
// }
