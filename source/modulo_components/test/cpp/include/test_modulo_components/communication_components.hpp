#pragma once

#include <rclcpp/rclcpp.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include "modulo_components/LifecycleComponent.hpp"

using namespace state_representation;
using namespace std::chrono_literals;

namespace modulo_components {

inline void add_configure_activate(
    const std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>& exec,
    const std::shared_ptr<LifecycleComponent>& component
) {
  exec->add_node(component->get_node_base_interface());
  component->configure();
  component->activate();
}

template<class ComponentT>
class MinimalCartesianOutput : public ComponentT {
public:
  MinimalCartesianOutput(
      const rclcpp::NodeOptions& node_options, const std::string& topic, const CartesianState& cartesian_state,
      bool publish_manually
  ) : ComponentT(node_options, "minimal_cartesian_output"), output_(std::make_shared<CartesianState>(cartesian_state)) {
    this->add_output("cartesian_state", this->output_, topic, true, publish_manually);
  }

  void publish() {
    this->publish_output("cartesian_state");
  }

private:
  std::shared_ptr<CartesianState> output_;
};

template<class ComponentT>
class MinimalCartesianInput : public ComponentT {
public:
  MinimalCartesianInput(const rclcpp::NodeOptions& node_options, const std::string& topic) :
      ComponentT(node_options, "minimal_cartesian_input"), input(std::make_shared<CartesianState>()) {
    this->received_future = this->received_.get_future();
    this->add_input("cartesian_state", this->input, [this]() { this->received_.set_value(); }, topic);
  }

  std::shared_ptr<CartesianState> input;
  std::shared_future<void> received_future;

private:
  std::promise<void> received_;
};
}// namespace modulo_components
