#pragma once

#include <chrono>
#include <exception>

#include <rclcpp/rclcpp.hpp>

namespace modulo_utils::testutils {

using namespace std::chrono_literals;

template<typename SrvT>
class ServiceClient : public rclcpp::Node {
public:
  ServiceClient(const rclcpp::NodeOptions& options, const std::string& service) : rclcpp::Node("client_node", options) {
    this->client_ = this->create_client<SrvT>(service);
    if (!this->client_->wait_for_service(1s)) {
      throw std::runtime_error("Service not available");
    }
  }

  typename rclcpp::Client<SrvT>::FutureAndRequestId call_async(const std::shared_ptr<typename SrvT::Request>& request) {
    return this->client_->async_send_request(request);
  }

  std::shared_ptr<rclcpp::Client<SrvT>> client_;
};
}// namespace modulo_utils::testutils
