# Modulo Core

Modulo Core is an interface package to support the interoperability of the [Robot Operating System (ROS)](https://www.ros.org)
with [AICA control libraries](https://github.com/aica-technology/control-libraries) by providing communication and
translation utilities.

This package is specifically designed for ROS2 and was developed on Galactic Geochelone.

## Communication

In ROS, applications communicate through publishers and subscribers. Communication is handled in a particular format
(a ROS message), which includes a serialized representation of the data as well as a message header.
For example, data of type `bool` maps to the standard message `std_msgs::msg::Bool`.

If application A wishes to send data to application B, a publisher of A and a subscriber of B must be configured
to the same ROS message type. The data is written into the message format and published by A. B receives the message
and can read the data back out.

### MessagePair

The communication module simplifies the process of sending and receiving data by binding data types to their
respective message types in a `MessagePair`. `MessagePair` objects are templated containers holding a pointer to the
data. With a `MessagePair` instance, it is possible to set and get the data or read and write the corresponding
message with simple access methods. The conversion between the data and the message when reading or writing is
handled by the [translators](#translators) module.

### MessageType

The supported groupings of data types and message types is defined by the `MessageType` enumeration.
See the [`MessageType` header file](./include/modulo_core/communication/MessageType.h).

| MessageType           | C++ data type                 | ROS message type                       |
|-----------------------|-------------------------------|----------------------------------------|
| `BOOL`                | `bool`                        | `std_msgs::msgs::Bool`                 |
| `INT32`               | `int`                         | `std_msgs::msgs::Int32`                |
| `FLOAT64`             | `double`                      | `std_msgs::msgs::FLOAT64`              |
| `FLOAT64_MUlTI_ARRAY` | `std::vector<double>`         | `std_msgs::msgs::FLOAT64MultiArray`    |
| `STRING`              | `std::string`                 | `std_msgs::msgs::String`               |
| `ENCODED_STATE`       | `state_representation::State` | `modulo_interfaces::msg::EncodedState` |

### Encoded State (TODO)

The `ENCODED_STATE` type supports all classes in the family of `state_representation::State`. It uses the
`clproto` serialization library from control_libraries to encode `State` type messages to and from a binary format.
The message type `modulo_core::EncodedState` is equivalent to the ROS `std_msgs::msg::UInt8MultiArray`.

The helper function `state_representation::make_shared_state(state_instance)` can be used to inject any `State`-derived
instance in a `std::shared_ptr<state_representation::State>` pointer for the MessagePair.

### MessagePairInterface

As `MessagePair` instances need to be templated by the data type and message type, they can be somewhat verbose to
manage directly. For example, storing multiple `MessagePair` instances in a container or passing them as function
arguments would require additional templating.

The `MessagePairInterface` is the non-templated base class of the `MessagePair`. By injecting a reference to a
`MessagePair` into a `MessagePairInterface` pointer, it is possible to manage any `MessagePair` in a generic,
type-agnostic way.

The `MessageType` property of the `MessagePairInterface` allows introspection of the contained `MessagePair` type,
and, through dynamic down-casting, both the `MessagePair` instance and contained data may be retrieved.

### Publisher and Subscription Interfaces

To send and receive ROS messages, applications must create publishers and subscribers templated to the corresponding
ROS message type. A ROS publisher provides a `publish` method which takes a ROS message as the argument.
A ROS subscription forwards an incoming ROS message to a callback function.

By virtue of the `MessagePair` design pattern, the underlying ROS message types can be fully encapsulated and hidden
from the user, allowing applications to simply publish and subscribe to supported data types.

#### Publisher

The `PublisherHandler` is a templated wrapper for a ROS publisher that additionally holds a `MessagePair` reference.
This allows the user to invoke a `publish()` method without having to pass a ROS message as an argument. Instead,
the data referenced by the MessagePair is automatically translated into a ROS message and published.

The `PublisherHandler` supports multiple publisher types, namely for standard or lifecycle publishers.

The intended pattern for the developer is shown below. There are a few more setup steps compared to creating a
standard ROS publisher. However, once the publisher interface has been established, modifying and publishing
the referenced data is greatly simplified.

```c++
using namespace modulo_core::communication;
typedef std_msgs::msg::FLOAT64 MsgT;
auto node = std::make_shared<rclcpp::Node>("example");

// hold a pointer reference to some data that should be published
auto data = std::make_shared<double>(1.0);

// make a MessagePair to bind the data pointer to the corresponding ROS message
auto message_pair = std::make_shared<MessagePair<MsgT, double>>(data, rclcpp::Clock());

// create the ROS publisher
auto publisher = node->create_publisher<MsgT>("topic", 10);

// create the PublisherHandler from the ROS publisher
auto publisher_handler =
  std::make_shared<PublisherHandler<rclcpp::Publisher<MsgT>, MsgT>>(PublisherType::PUBLISHER, publisher);

// encapsulate the PublisherHandler in a PublisherInterface
std::shared_ptr<PublisherInterface> publisher_interface(publisher_handler);

// pass the MessagePair to the PublisherInterface 
publisher_interface->set_message_pair(message_pair);

// now, the data can be published, with automatic translation of the data value into a ROS message
publisher_interface->publish();

// because the PublisherInterface holds a reference to the MessagePair, which in turn references the original data,
// any changes to the data value will be observed when publishing. 
*data = 2.0;
publisher_interface->publish();
```

#### Subscription

The `SubscriptionHandler` is a templated wrapper for a ROS subscription that additionally holds a `MessagePair`
reference. When the subscription receives a message, the `SubscriptionHandler` callback automatically translates
the ROS message and updates the value of the data referenced by the `MessagePair`.

```c++
using namespace modulo_core::communication;
typedef std_msgs::msg::FLOAT64 MsgT;
auto node = std::make_shared<rclcpp::Node>("example");

// hold a pointer reference to some data that should be published
auto data = std::make_shared<double>(1.0);

// make a MessagePair to bind the data pointer to the corresponding ROS message
auto message_pair = std::make_shared<MessagePair<MsgT, double>>(data, rclcpp::Clock());

// create the SubscriptionHandler
auto subscription_handler = std::make_shared<SubscriptionHandler<modulo_core::EncodedState>>(message_pair);

// create the ROS subscription and associate the SubscriptionHandler callback
auto subscription = node->create_subscription<modulo_core::EncodedState>(
    "topic", 10, subscription_handler->get_callback());

// encapsulate the SubscriptionHandler in a SubscriptionInterface
auto subscription_interface = subscription_handler->create_subscription_interface(subscription);

// because the SubscriptionInterface holds a reference to the MessagePair, which in turn references the original data,
// any received ROS message will be translated to update the referenced data value
wait_until_some_subscription_received();
assert(*data == 2.0);
```

## Translators

The translation module provides functions to convert between ROS2 and state_representation data types.

### Message Translators

As described in the [Communication](#communication) section, the ROS framework uses specific message formats to
send data between applications. Wrapping and unwrapping the data values to and from the ROS message is handled by
the message translators.

The message readers set the value of a particular data type from the corresponding ROS message instance.

The message writers format the value of a data type into the corresponding ROS message instance.

### Parameter Translators

Conceptually, parameters are a way to label and transfer data with a name-value relationship.
A parameter has a name and a data value of a particular data type. On an interface level, this allows parameter
values to be retrieved or written by name in order to determine or configure the behaviour of particular elements.

The ROS `Parameter` is a specific implementation of the parameter concept which, together with the `ParameterMessage`,
can be used to read and write named parameters on application nodes through the ROS interface. The ROS `Parameter`
supports only simple types (atomic types, strings and arrays).

The control libraries [`state_representation::Parameter`](https://aica-technology.github.io/control-libraries/versions/main/classstate__representation_1_1_parameter_interface.html)
is another implementation that supports more data types, including `State`-derived objects and matrices.

The parameter translator utilities in `modulo_core::translators` convert between ROS and `state_representation`
parameter formats, so that parameters can be sent on the ROS interface but consumed as `state_representation` objects locally.