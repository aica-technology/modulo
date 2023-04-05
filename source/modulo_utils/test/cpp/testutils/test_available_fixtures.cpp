#include <gtest/gtest.h>

#include <chrono>

#include <std_srvs/srv/empty.hpp>

#include "modulo_utils/testutils/PredicatesListener.hpp"
#include "modulo_utils/testutils/ServiceClient.hpp"

namespace modulo_utils {

using namespace std::chrono_literals;

class FixturesTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(FixturesTest, ServiceClient) {
  EXPECT_THROW(std::make_shared<testutils::ServiceClient<std_srvs::srv::Empty>>(
      rclcpp::NodeOptions(), "/bool"), std::runtime_error);
}

TEST_F(FixturesTest, PredicatesListener) {
  auto listener = std::make_shared<testutils::PredicatesListener>(
      rclcpp::NodeOptions(), "test", std::vector<std::string>{"in_error_state"});
  exec_->add_node(listener->get_node_base_interface());
  auto result = exec_->spin_until_future_complete(listener->get_predicate_future(), 1s);
  EXPECT_EQ(result, rclcpp::FutureReturnCode::TIMEOUT);
}
}// namespace modulo_utils