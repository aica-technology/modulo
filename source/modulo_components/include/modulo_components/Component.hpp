#pragma once

#include <thread>

#include <rclcpp/node.hpp>

#include "modulo_components/ComponentInterface.hpp"
#include "modulo_core/concepts.hpp"

namespace modulo_components {

/**
 * @class Component
 * @brief A wrapper for rclcpp::Node to simplify application composition through unified component interfaces.
 * @details This class is intended for direct inheritance to implement custom components that perform one-shot or
 * externally triggered operations. Examples of triggered behavior include providing a service, processing signals
 * or publishing outputs on a periodic timer. One-shot behaviors may include interacting with the filesystem or
 * publishing a predefined sequence of outputs.
 * Developers should override on_validate_parameter_callback() if any parameters are added and on_execute_callback()
 * to implement any one-shot behavior. In the latter case, execute() should be invoked at the end of the derived
 * constructor.
 * @see LifecycleComponent for a state-based composition alternative
 */
class Component : public rclcpp::Node, public ComponentInterface {
public:
  friend class ComponentPublicInterface;

  /**
   * @brief Constructor from node options.
   * @param node_options Node options as used in ROS2 Node
   * @param fallback_name The name of the component if it was not provided through the node options
   */
  explicit Component(const rclcpp::NodeOptions& node_options, const std::string& fallback_name = "Component");

  /**
   * @brief Virtual default destructor.
   */
  virtual ~Component() = default;

protected:
  /**
   * @copydoc ComponentInterface::get_parameter
   */
  [[nodiscard]] std::shared_ptr<state_representation::ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Start the execution thread.
   */
  void execute();

  /**
   * @brief Execute the component logic. To be redefined in derived classes.
   * @return True, if the execution was successful, false otherwise
   */
  virtual bool on_execute_callback();

  /**
   * @brief Add and configure an output signal of the component.
   * @tparam DataT Type of the data pointer
   * @param signal_name Name of the output signal
   * @param data Data to transmit on the output signal
   * @param default_topic If set, the default value for the topic name to use
   * @param fixed_topic If true, the topic name of the output signal is fixed
   * @param publish_on_step If true, the output is published periodically on step
   */
  template<typename DataT>
  void add_output(
      const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::string& default_topic = "",
      bool fixed_topic = false, bool publish_on_step = true);

private:
  /**
   * @brief Step function that is called periodically and publishes predicates, outputs, and evaluates daemon callbacks.
   */
  void step() override;

  /**
   * @brief Run the execution function in a try catch block and set the predicates according to the outcome of the
   * execution.
   */
  void on_execute();

  // TODO hide ROS methods
  using ComponentInterface::create_output;
  using ComponentInterface::evaluate_periodic_callbacks;
  using ComponentInterface::get_parameter;
  using ComponentInterface::inputs_;
  using ComponentInterface::outputs_;
  using ComponentInterface::periodic_outputs_;
  using ComponentInterface::publish_outputs;
  using ComponentInterface::publish_predicates;
  using rclcpp::Node::get_parameter;

  std::thread execute_thread_;///< The execution thread of the component
  bool started_;              ///< Flag that indicates if execution has started or not
};

template<typename DataT>
inline void Component::add_output(
    const std::string& signal_name, const std::shared_ptr<DataT>& data, const std::string& default_topic,
    bool fixed_topic, bool publish_on_step) {
  using namespace modulo_core::communication;
  try {
    auto parsed_signal_name =
        this->create_output(PublisherType::PUBLISHER, signal_name, data, default_topic, fixed_topic, publish_on_step);
    auto topic_name = this->get_parameter_value<std::string>(parsed_signal_name + "_topic");
    RCLCPP_DEBUG_STREAM(
        this->get_logger(), "Adding output '" << parsed_signal_name << "' with topic name '" << topic_name << "'.");
    auto message_pair = this->outputs_.at(parsed_signal_name)->get_message_pair();
    switch (message_pair->get_type()) {
      case MessageType::ENCODED_STATE: {
        auto publisher = this->create_publisher<modulo_core::EncodedState>(topic_name, this->get_qos());
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<modulo_core::EncodedState>, modulo_core::EncodedState>>(
                PublisherType::PUBLISHER, publisher)
                ->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::BOOL: {
        auto publisher = this->create_publisher<std_msgs::msg::Bool>(topic_name, this->get_qos());
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Bool>, std_msgs::msg::Bool>>(
                PublisherType::PUBLISHER, publisher)
                ->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::FLOAT64: {
        auto publisher = this->create_publisher<std_msgs::msg::Float64>(topic_name, this->get_qos());
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Float64>, std_msgs::msg::Float64>>(
                PublisherType::PUBLISHER, publisher)
                ->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::FLOAT64_MULTI_ARRAY: {
        auto publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, this->get_qos());
        this->outputs_.at(parsed_signal_name) = std::make_shared<PublisherHandler<
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>, std_msgs::msg::Float64MultiArray>>(
                                                    PublisherType::PUBLISHER, publisher)
                                                    ->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::INT32: {
        auto publisher = this->create_publisher<std_msgs::msg::Int32>(topic_name, this->get_qos());
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::Int32>, std_msgs::msg::Int32>>(
                PublisherType::PUBLISHER, publisher)
                ->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::STRING: {
        auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, this->get_qos());
        this->outputs_.at(parsed_signal_name) =
            std::make_shared<PublisherHandler<rclcpp::Publisher<std_msgs::msg::String>, std_msgs::msg::String>>(
                PublisherType::PUBLISHER, publisher)
                ->create_publisher_interface(message_pair);
        break;
      }
      case MessageType::CUSTOM_MESSAGE: {
        if constexpr (modulo_core::concepts::CustomT<DataT>) {
          auto publisher = this->create_publisher<DataT>(topic_name, this->get_qos());
          this->outputs_.at(parsed_signal_name) =
              std::make_shared<PublisherHandler<rclcpp::Publisher<DataT>, DataT>>(PublisherType::PUBLISHER, publisher)
                  ->create_publisher_interface(message_pair);
        }
        break;
      }
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to add output '" << signal_name << "': " << ex.what());
  }
}
}// namespace modulo_components
