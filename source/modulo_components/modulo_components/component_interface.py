import copy
import inspect
import sys
from functools import partial
from threading import Lock
from typing import Callable, Dict, Iterable, List, Optional, TypeVar, Union

import clproto
import modulo_core.translators.message_readers as modulo_readers
import modulo_core.translators.message_writers as modulo_writers
import state_representation as sr
from geometry_msgs.msg import TransformStamped
from modulo_interfaces.msg import Predicate as PredicateMsg
from modulo_interfaces.msg import PredicateCollection
from modulo_interfaces.srv import EmptyTrigger, StringTrigger
from modulo_utils.parsing import parse_topic_name, topic_validation_warning
from modulo_core import EncodedState, Predicate
from modulo_core.exceptions import *
from modulo_core.translators.parameter_translators import get_ros_parameter_type, read_parameter_const, write_parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.service import Service
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray, String
from tf2_py import TransformException
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

MsgT = TypeVar('MsgT')
T = TypeVar('T')
NODE_KWARGS = ["context", "cli_args", "namespace", "use_global_arguments", "enable_rosout",
               "start_parameter_services", "parameter_overrides", "allow_undeclared_parameters",
               "automatically_declare_parameters_from_overrides"]


class ComponentInterface(Node):
    """
    Base interface class for modulo components to wrap a ROS Node with custom behaviour,
    following the same logic pattern as the C++ modulo_components::ComponentInterface class.
    """

    def __init__(self, node_name: str, *args, **kwargs):
        node_kwargs = {key: value for key, value in kwargs.items() if key in NODE_KWARGS}
        super().__init__(node_name, *args, **node_kwargs)
        self.__step_lock = Lock()
        self.__parameter_dict: Dict[str, Union[str, sr.Parameter]] = {}
        self.__predicates: Dict[str, Predicate] = {}
        self.__triggers: List[str] = []
        self.__periodic_callbacks: Dict[str, Callable[[], None]] = {}
        self.__inputs = {}
        self.__outputs = {}
        self.__periodic_outputs: Dict[str, bool] = {}
        self.__services_dict: Dict[str, Service] = {}
        self.__tf_buffer: Optional[Buffer] = None
        self.__tf_listener: Optional[TransformListener] = None
        self.__tf_broadcaster: Optional[TransformBroadcaster] = None
        self.__static_tf_broadcaster: Optional[StaticTransformBroadcaster] = None

        self.__qos = QoSProfile(depth=10)

        self.add_on_set_parameters_callback(self.__on_set_parameters_callback)
        self.add_parameter(sr.Parameter("rate", 10.0, sr.ParameterType.DOUBLE),
                           "The rate in Hertz for all periodic callbacks")

        self.__predicate_publisher = self.create_publisher(PredicateCollection, "/predicates", self.__qos)
        self.__predicate_message = PredicateCollection()
        self.__predicate_message.node = self.get_fully_qualified_name()
        self.__predicate_message.type = PredicateCollection.COMPONENT

        self.__rate = self.get_parameter_value("rate")
        self.__period = 1.0 / self.__rate
        self.__step_timer = self.create_timer(self.__period, self.__step_with_mutex)

    def __del__(self):
        self.__step_lock.acquire()

    def get_rate(self) -> float:
        """
        Get the component rate in Hertz.

        :return: The component rate
        """
        return self.__rate

    def get_period(self) -> float:
        """
        Get the component period in seconds.

        :return: The component period
        """
        return self.__period

    def __step_with_mutex(self) -> None:
        if self.__step_lock.acquire(blocking=False):
            self._step()
            self.__step_lock.release()

    def _step(self) -> None:
        """
        Step function that is called periodically.
        """
        pass

    def on_step_callback(self) -> None:
        """
        Steps to execute periodically. To be redefined by derived classes.
        """
        pass

    def add_parameter(self, parameter: Union[str, sr.Parameter], description: str, read_only=False) -> None:
        """
        Add a parameter. This method stores either the name of the attribute corresponding to the parameter object or
        a parameter object directly in the local parameter dictionary and declares the equivalent ROS parameter on the
        ROS interface. If an attribute name is provided, changing the ROS parameter will also update the provided
        attribute and vice versa. If the provided argument is not an attribute name or a Parameter object, nothing
        happens.

        :param parameter: Either the name of the parameter attribute or the parameter itself
        :param description: The parameter description
        :param read_only: If True, the value of the parameter cannot be changed after declaration
        :raises ParameterError: if the parameter could not be added
        """
        try:
            if isinstance(parameter, sr.Parameter):
                sr_parameter = parameter
            elif isinstance(parameter, str):
                attr = self.__getattribute__(parameter)
                if isinstance(attr, sr.Parameter):
                    sr_parameter = attr
                else:
                    raise TypeError(
                        f"The attribute with the provided name '{parameter}' does not contain a Parameter object.")
            else:
                raise TypeError("Provide either a state_representation.Parameter object or a string "
                                "containing the name of the attribute that refers to the parameter to add.")
            ros_param = write_parameter(sr_parameter)
        except (TypeError, ParameterTranslationError) as e:
            raise ParameterError(f"Failed to add parameter: {e}")
        if not self.has_parameter(sr_parameter.get_name()):
            self.get_logger().debug(f"Adding parameter '{sr_parameter.get_name()}'.")
            self.__parameter_dict[sr_parameter.get_name()] = parameter
            try:
                descriptor = ParameterDescriptor(description=description, read_only=read_only)
                if sr_parameter.is_empty():
                    descriptor.dynamic_typing = True
                    descriptor.type = get_ros_parameter_type(sr_parameter.get_parameter_type()).value
                    self.declare_parameter(ros_param.name, None, descriptor=descriptor)
                else:
                    self.declare_parameter(ros_param.name, ros_param.value, descriptor=descriptor)
            except Exception as e:
                del self.__parameter_dict[sr_parameter.get_name()]
                raise ParameterError(f"Failed to add parameter: {e}")
        else:
            self.get_logger().debug(f"Parameter '{sr_parameter.get_name()}' already exists.")

    def get_parameter(self, name: str) -> Union[sr.Parameter, Parameter]:
        """
        Get a parameter by name. If this method is called from a file that contains 'rclpy' in its path, the
        rclpy.node.Node.get_parameter method will be invoked, otherwise return the parameter from the local parameter
        dictionary.

        :param name: The name of the parameter
        :raises ParameterError: if the parameter does not exist
        :return: The requested parameter
        """
        try:
            co_filename = sys._getframe().f_back.f_code.co_filename
            self.get_logger().debug(f"get_parameter called from {co_filename}")
            if "rclpy" in co_filename:
                return Node.get_parameter(self, name)
            else:
                return self.__get_component_parameter(name)
        except Exception as e:
            raise ParameterError(f"Failed to get parameter '{name}': {e}")

    def __get_component_parameter(self, name: str) -> sr.Parameter:
        """
        Get the parameter from the parameter dictionary by its name.

        :param name: The name of the parameter
        :raises ParameterError: if the parameter does not exist
        :return: The parameter, if it exists
        """
        if name not in self.__parameter_dict.keys():
            raise ParameterError(f"Parameter '{name}' is not in the dict of parameters")
        try:
            if isinstance(self.__parameter_dict[name], str):
                return self.__getattribute__(self.__parameter_dict[name])
            else:
                return self.__parameter_dict[name]
        except AttributeError as e:
            raise ParameterError(f"{e}")

    def get_parameter_value(self, name: str) -> T:
        """
        Get the parameter value from the parameter dictionary by its name.

        :param name: The name of the parameter
        :raises ParameterError: if the parameter does not exist
        :return: The value of the parameter, if the parameter exists
        """
        return self.__get_component_parameter(name).get_value()

    def set_parameter_value(self, name: str, value: T, parameter_type: sr.ParameterType) -> None:
        """
        Set the value of a parameter. The parameter must have been previously declared. If the parameter is an
        attribute, the attribute is updated.

        :param name: The name of the parameter
        :param value: The value of the parameter
        :param parameter_type: The type of the parameter
        """
        try:
            ros_param = write_parameter(sr.Parameter(name, value, parameter_type))
            result = self.set_parameters([ros_param])[0]
            if not result.successful:
                self.get_logger().error(f"Failed to set parameter value of parameter '{name}': {result.reason}",
                                        throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f"Failed to set parameter value of parameter '{name}': {e}",
                                    throttle_duration_sec=1.0)

    def __validate_parameter(self, parameter: sr.Parameter) -> bool:
        """
        Parameter validation wrapper that validates the period and calls the on_validate_parameter_callback function of
        the derived Component classes.

        :param parameter: The parameter to be validated
        :return: The validation result
        """
        if parameter.get_name() == "rate":
            value = parameter.get_value()
            if value <= 0 or value > sys.float_info.max:
                self.get_logger().error("Value for parameter 'rate' has to be a positive finite number.")
                return False
        try:
            result = self.on_validate_parameter_callback(parameter)
        except AttributeError:
            self.get_logger().error("Attribute error during 'on_validate_parameter_callback'. Declare necessary "
                                    "attributes before __init__ of the parent class")
            raise
        if result is None:
            self.get_logger().error("Expected a return value from 'on_validate_parameter_callback'. "
                                    "Parameter change will be rejected.")
            return False
        return result

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        """
        Parameter validation function to be redefined by derived Component classes. This method is automatically invoked
        whenever the ROS interface tried to modify a parameter. If the validation returns True, the updated parameter
        value (including any modifications) is applied. If the validation returns False, any changes to the parameter
        are discarded and the parameter value is not changed.

        :param parameter: The parameter to be validated
        :return: The validation result
        """
        return True

    def __on_set_parameters_callback(self, ros_parameters: List[Parameter]) -> SetParametersResult:
        """
        Callback function to validate and update parameters on change.

        :param ros_parameters: The new parameter objects provided by the ROS interface
        :return: The result of the validation
        """
        result = SetParametersResult(successful=True)
        for ros_param in ros_parameters:
            try:
                parameter = self.__get_component_parameter(ros_param.name)
                new_parameter = read_parameter_const(ros_param, parameter)
                if not self.__validate_parameter(new_parameter):
                    result.successful = False
                    result.reason = f"Validation of parameter '{ros_param.name}' returned false!"
                else:
                    if isinstance(self.__parameter_dict[ros_param.name], str):
                        self.__setattr__(self.__parameter_dict[ros_param.name], new_parameter)
                    else:
                        self.__parameter_dict[ros_param.name] = new_parameter
            except Exception as e:
                result.successful = False
                result.reason += str(e)
        return result

    def add_predicate(self, name: str, predicate: Union[bool, Callable[[], bool]]) -> None:
        """
        Add a predicate to the map of predicates.

        :param name: The name of the predicate
        :param predicate: The value of the predicate as a bool or a callable function
        """
        if not name:
            self.get_logger().error("Failed to add predicate: Provide a non empty string as a name.")
        if name in self.__predicates.keys():
            self.get_logger().warn(f"Predicate with name '{name}' already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding predicate '{name}'.")
        try:
            if callable(predicate):
                self.__predicates[name] = Predicate(predicate)
            else:
                self.__predicates[name] = Predicate(lambda: predicate)
        except Exception as e:
            self.get_logger().error(f"Failed to add predicate '{name}': {e}")

    def get_predicate(self, name: str) -> bool:
        """
        Get the value of the predicate given as parameter. If the predicate is not found or the callable function fails,
        this method returns False.

        :param name: The name of the predicate to retrieve from the map of predicates
        :return: The value of the predicate as a boolean
        """
        if name not in self.__predicates.keys():
            self.get_logger().error(
                f"Failed to get predicate '{name} ': Predicate does not exist, returning false.",
                throttle_duration_sec=1.0)
            return False
        try:
            return self.__predicates[name].get_value()
        except Exception as e:
            self.get_logger().error(f"""Failed to evaluate callback of predicate '{
                name}', returning false: {e}""", throttle_duration_sec=1.0)
        return False

    def set_predicate(self, name: str, predicate: Union[bool, Callable[[], bool]]) -> None:
        """
        Set the value of the predicate given as parameter, if the predicate is not found does not do anything. Even
        though the predicates are published periodically, the new value of this predicate will be published once
        immediately after setting it.

        :param name: The name of the predicate to retrieve from the map of predicates
        :param predicate: The new value of the predicate as a bool or a callable function
        """
        if name not in self.__predicates.keys():
            self.get_logger().error(f"""Failed to set predicate '{
                name}': Predicate does not exist.""", throttle_duration_sec=1.0)
            return
        try:
            if callable(predicate):
                self.__predicates[name].set_predicate(predicate)
            else:
                self.__predicates[name].set_predicate(lambda: predicate)
        except Exception as e:
            self.get_logger().error(f"Failed to set predicate '{name}': {e}", throttle_duration_sec=1.0)
            return
        new_value = self.__predicates[name].query()
        if new_value is not None:
            self.__publish_predicate(name, new_value)

    def add_trigger(self, trigger_name: str) -> None:
        """
        Add a trigger to the component. Triggers are predicates that are always false except when it's triggered in
        which case it is set back to false immediately after it is read.

        :param trigger_name: The name of the trigger
        """
        if not trigger_name:
            self.get_logger().error("Failed to add trigger: Provide a non empty string as a name.")
            return
        if trigger_name in self.__triggers:
            self.get_logger().error(f"Failed to add trigger: there is already a trigger with name '{trigger_name}'.")
            return
        if trigger_name in self.__predicates.keys():
            self.get_logger().error(f"Failed to add trigger: there is already a predicate with name '{trigger_name}'.")
            return
        self.__triggers.append(trigger_name)
        self.add_predicate(trigger_name, False)

    def trigger(self, trigger_name: str) -> None:
        """
        Latch the trigger with the provided name.

        :param trigger_name: The name of the trigger
        """
        if trigger_name not in self.__triggers:
            self.get_logger().error(f"Failed to trigger: could not find trigger with name '{trigger_name}'.")
            return
        self.set_predicate(trigger_name, True)
        # reset the trigger to be published on the next step
        self.__predicates[trigger_name].set_predicate(lambda: False)

    def remove_output(self, signal_name) -> None:
        if signal_name not in self.__outputs.keys():
            parsed_signal_name = parse_topic_name(signal_name)
            if parsed_signal_name not in self.__outputs.keys():
                self.get_logger().debug(f"Unknown output '{signal_name}' (parsed name was '{parsed_signal_name}').")
                return
            signal_name = parsed_signal_name
        if "publisher" in self.__outputs[signal_name].keys():
            self.destroy_publisher(self.__outputs[signal_name]["publisher"])
        self.__outputs.pop(signal_name)
        self.get_logger().debug(f"Removing signal '{signal_name}'.")

    def __create_output(self, signal_name: str, data: str, message_type: MsgT,
                        clproto_message_type: Union[clproto.MessageType, None], default_topic: str, fixed_topic: bool,
                        publish_on_step: bool) -> str:
        """
        Helper function to parse the signal name and add an output without Publisher to the dict of outputs.

        :param signal_name: Name of the output signal
        :param data: Name of the attribute to transmit over the channel
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :param publish_on_step: If true, the output is published periodically on step
        :raises AddSignalError: if there is a problem adding the output
        :return: The parsed signal name
        """
        try:
            if message_type == Bool or message_type == Float64 or \
                    message_type == Float64MultiArray or message_type == Int32 or message_type == String:
                translator = modulo_writers.write_std_message
            elif message_type == EncodedState:
                cl_msg_type = clproto_message_type if clproto_message_type else modulo_writers.get_clproto_msg_type(
                    self.__getattribute__(data))
                if cl_msg_type == clproto.MessageType.UNKNOWN_MESSAGE:
                    raise AddSignalError(f"""Provide a valid clproto message type for output '{
                                         signal_name}' of type EncodedState.""")
                translator = partial(modulo_writers.write_clproto_message, clproto_message_type=cl_msg_type)
            elif hasattr(message_type, 'get_fields_and_field_types'):
                def write_ros_msg(message, data):
                    for field in message.get_fields_and_field_types().keys():
                        setattr(message, field, getattr(data, field))
                translator = write_ros_msg
            else:
                raise AddSignalError(
                    f"The provided message type is not supported to create component output '{signal_name}'.")
            self.declare_output(signal_name, default_topic, fixed_topic)
            parsed_signal_name = parse_topic_name(signal_name)
            self.__outputs[parsed_signal_name] = {"attribute": data, "message_type": message_type,
                                                  "translator": translator}
            self.__periodic_outputs[parsed_signal_name] = publish_on_step
            return parsed_signal_name
        except AddSignalError:
            raise
        except Exception as e:
            raise AddSignalError(f"{e}")

    def __set_output_publisher(self, output_name: str, publisher) -> None:
        """
        Set the publisher object for a component output.

        :param output_name: The name of the output
        :param publisher: The publisher object for the output
        """
        try:
            self.__outputs[output_name]["publisher"] = publisher
        except Exception as e:
            self.get_logger().warn(f"Failed to set output publisher: {e}")

    def remove_input(self, signal_name: str) -> None:
        if not self.destroy_subscription(self.__inputs.pop(signal_name, None)):
            parsed_signal_name = parse_topic_name(signal_name)
            if not self.destroy_subscription(self.__inputs.pop(parsed_signal_name, None)):
                self.get_logger().debug(f"Unknown input '{signal_name}' (parsed name was '{parsed_signal_name}').")
                return
            self.get_logger().debug(f"Removing signal '{parsed_signal_name}'.")
            return
        self.get_logger().debug(f"Removing signal '{signal_name}'.")

    def __read_translated_message(self, message: MsgT, attribute_name: str, reader: Callable) -> None:
        obj_type = type(self.__getattribute__(attribute_name))
        decoded_message = reader(message)
        self.__setattr__(attribute_name, obj_type(decoded_message))

    def __read_custom_message(self, message: MsgT, attribute_name: str) -> None:
        for field in message.get_fields_and_field_types().keys():
            setattr(self.__getattribute__(attribute_name), field, getattr(message, field))

    def __subscription_callback(
            self, message: MsgT, attribute_name: str, read_message: Callable, user_callback: Callable) -> None:
        """
        Subscription callback for the ROS subscriptions.

        :param message: The message from the ROS network
        :param attribute_name: The name of the attribute that is updated by the subscription
        :param reader: A callable that can read the ROS message and translate to the desired type
        """
        try:
            read_message(message, attribute_name)
        except (AttributeError, MessageTranslationError, TypeError) as e:
            self.get_logger().warn(f"Failed to read message for attribute {attribute_name}: {e}",
                                   throttle_duration_sec=1.0)
            return
        try:
            user_callback()
        except Exception as e:
            self.get_logger().error(f"""Failed to execute user callback in subscription for attribute
                                    '{attribute_name}': {e}""", throttle_duration_sec=1.0)

    def __safe_callback(self, message: MsgT, signal_name: str, callback: Callable) -> None:
        try:
            callback(message)
        except Exception as e:
            self.get_logger().warn(f"""Unhandled exception in callback of input '{
                signal_name}': {e}""", throttle_duration_sec=1.0)

    def __declare_signal(self, signal_name: str, signal_type: str, default_topic="", fixed_topic=False) -> None:
        """
        Declare an input to create the topic parameter without adding it to the map of inputs yet.

        :param signal_name: The signal name of the input
        :param signal_type: The type of the signal (input or output)
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the signal is fixed
        :raises AddSignalError: if the signal could not be declared (empty name or already created)
        """
        parsed_signal_name = parse_topic_name(signal_name)
        if not parsed_signal_name:
            raise AddSignalError(topic_validation_warning(signal_name, signal_type))
        if signal_name != parsed_signal_name:
            self.get_logger().warn(
                f"""The parsed name for {signal_type} '{signal_name}' is '{parsed_signal_name}'."
                "Use the parsed name to refer to this {signal_type}.""")
        if parsed_signal_name in self.__inputs.keys():
            raise AddSignalError(f"Signal with name '{parsed_signal_name}' already exists as input.")
        if parsed_signal_name in self.__outputs.keys():
            raise AddSignalError(f"Signal with name '{parsed_signal_name}' already exists as output.")
        topic_name = default_topic if default_topic else "~/" + parsed_signal_name
        parameter_name = parsed_signal_name + "_topic"
        if self.has_parameter(parameter_name) and self.get_parameter(parameter_name).is_empty():
            self.set_parameter_value(parameter_name, topic_name, sr.ParameterType.STRING)
        else:
            self.add_parameter(sr.Parameter(parameter_name, topic_name, sr.ParameterType.STRING),
                               f"Signal topic name of {signal_type} '{parsed_signal_name}'", fixed_topic)
        self.get_logger().debug(
            f"Declared signal '{parsed_signal_name}' and parameter '{parameter_name}' with value '{topic_name}'.")

    def declare_input(self, signal_name: str, default_topic="", fixed_topic=False) -> None:
        """
        Declare an input to create the topic parameter without adding it to the map of inputs yet.

        :param signal_name: The signal name of the input
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the signal is fixed
        :raises AddSignalError: if the input could not be declared (empty name or already created)
        """
        self.__declare_signal(signal_name, "input", default_topic, fixed_topic)

    def declare_output(self, signal_name: str, default_topic="", fixed_topic=False) -> None:
        """
        Declare an output to create the topic parameter without adding it to the map of outputs yet.

        :param signal_name: The signal name of the output
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the signal is fixed
        :raises AddSignalError: if the output could not be declared (empty name or already created)
        """
        self.__declare_signal(signal_name, "output", default_topic, fixed_topic)

    def add_input(self, signal_name: str, subscription: Union[str, Callable], message_type: MsgT, default_topic="",
                  fixed_topic=False, user_callback: Callable = None) -> None:
        """
        Add and configure an input signal of the component.

        :param signal_name: Name of the output signal
        :param subscription: Name of the attribute to receive messages for or the callback to use for the subscription
        :param message_type: ROS message type of the subscription
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :param user_callback: Callback function to trigger after receiving the input signal
        :raises AddSignalError: if there is a problem adding the input
        """
        try:
            self.declare_input(signal_name, default_topic, fixed_topic)
            parsed_signal_name = parse_topic_name(signal_name)
            topic_name = self.get_parameter_value(parsed_signal_name + "_topic")
            self.get_logger().debug(f"Adding input '{parsed_signal_name}' with topic name '{topic_name}'.")
            if isinstance(subscription, Callable):
                if user_callback:
                    self.get_logger().warn("Providing a callable for arguments 'subscription' and 'user_callback' is"
                                           "not supported. The user callback will be ignored.")
                self.__inputs[parsed_signal_name] = self.create_subscription(message_type, topic_name, partial(
                    self.__safe_callback, signal_name=parsed_signal_name, callback=subscription), self.__qos)
            elif isinstance(subscription, str):
                if callable(user_callback):
                    signature = inspect.signature(user_callback)
                    if len(signature.parameters) != 0:
                        raise AddSignalError("Provide a user callback that has no input arguments.")
                else:
                    if user_callback:
                        self.get_logger().warn("Provided user callback is not a callable, ignoring it.")

                    def default_callback():
                        return None

                    user_callback = default_callback
                if message_type == Bool or message_type == Float64 or \
                        message_type == Float64MultiArray or message_type == Int32 or message_type == String:
                    read_message = partial(self.__read_translated_message,
                                           reader=modulo_readers.read_std_message)
                    self.__inputs[parsed_signal_name] = \
                        self.create_subscription(message_type, topic_name,
                                                 partial(self.__subscription_callback,
                                                         attribute_name=subscription,
                                                         read_message=read_message,
                                                         user_callback=user_callback),
                                                 self.__qos)
                elif message_type == EncodedState:
                    read_message = partial(self.__read_translated_message,
                                           reader=modulo_readers.read_clproto_message)
                    self.__inputs[parsed_signal_name] = \
                        self.create_subscription(message_type, topic_name,
                                                 partial(self.__subscription_callback,
                                                         attribute_name=subscription,
                                                         read_message=read_message,
                                                         user_callback=user_callback),
                                                 self.__qos)
                elif hasattr(message_type, 'get_fields_and_field_types'):
                    self.__inputs[parsed_signal_name] = \
                        self.create_subscription(message_type, topic_name,
                                                 partial(self.__subscription_callback,
                                                         attribute_name=subscription,
                                                         read_message=self.__read_custom_message,
                                                         user_callback=user_callback),
                                                 self.__qos)
                else:
                    raise TypeError("The provided message type is not supported to create a component input.")
            else:
                raise TypeError("Provide either a string containing the name of an attribute or a callable.")
        except Exception as e:
            self.get_logger().error(f"Failed to add input '{signal_name}': {e}")

    def add_service(self, service_name: str, callback: Union[Callable[[], dict], Callable[[str], dict]]) -> None:
        """
        Add a service to trigger a callback function.
        The callback should take either no arguments (empty service) or a single string argument (string service).
        The string payload can have an arbitrary format to parameterize and control the callback behaviour as desired.
        It is the responsibility of the service callback to parse the string according to some payload format.
        When adding a service with a string payload, be sure to document the payload format appropriately.

        :param service_name: The name of the service
        :param callback: The callback function to execute
        :return: A dict with the outcome of the service call, containing "success" and "message" fields in the format
            {"success": [True | False], "message": "..."}
        """

        def callback_wrapper(request, response, cb):
            try:
                if hasattr(request, "payload"):
                    ret = cb(request.payload)
                else:
                    ret = cb()

                # if the return does not contain a success field or bool result,
                # but the callback completes without error, assume it was successful
                response.success = True
                if isinstance(ret, dict):
                    if "success" in ret.keys():
                        response.success = ret["success"]
                    if "message" in ret.keys():
                        response.message = ret["message"]
                elif isinstance(ret, bool):
                    response.success = ret
            except Exception as e:
                response.success = False
                response.message = f"{e}"
            return response

        try:
            parsed_service_name = parse_topic_name(service_name)
            if not parsed_service_name:
                raise AddServiceError(topic_validation_warning(service_name, "service"))
            if service_name != parsed_service_name:
                self.get_logger().warn(
                    f"The parsed name for service '{service_name}' is '{parsed_service_name}'."
                    "Use the parsed name to refer to this service.")
            if parsed_service_name in self.__services_dict.keys():
                raise AddServiceError(f"Service with name '{parsed_service_name}' already exists.")
            signature = inspect.signature(callback)
            if len(signature.parameters) == 0:
                self.get_logger().debug(f"Adding empty service '{parsed_service_name}'.")
                service_type = EmptyTrigger
            else:
                self.get_logger().debug(f"Adding string service '{parsed_service_name}'.")
                service_type = StringTrigger
            self.__services_dict[parsed_service_name] = \
                self.create_service(service_type, "~/" + parsed_service_name,
                                    lambda request, response: callback_wrapper(request, response, callback))
        except Exception as e:
            self.get_logger().error(f"Failed to add service '{service_name}': {e}")

    def add_tf_broadcaster(self) -> None:
        """
        Configure a transform broadcaster.
        """
        if not self.__tf_broadcaster:
            self.get_logger().debug("Adding TF broadcaster.")
            self.__tf_broadcaster = TransformBroadcaster(self)
        else:
            self.get_logger().error("TF broadcaster already exists.")

    def add_static_tf_broadcaster(self) -> None:
        """
        Configure a static transform broadcaster.
        """
        if not self.__static_tf_broadcaster:
            self.get_logger().debug("Adding static TF broadcaster.")
            self.__static_tf_broadcaster = StaticTransformBroadcaster(self)
        else:
            self.get_logger().error("TF broadcaster already exists.")

    def add_tf_listener(self) -> None:
        """
        Configure a transform buffer and listener.
        """
        if not self.__tf_buffer or not self.__tf_listener:
            self.get_logger().debug("Adding TF buffer and listener.")
            self.__tf_buffer = Buffer()
            self.__tf_listener = TransformListener(self.__tf_buffer, self)
        else:
            self.get_logger().error("TF buffer and listener already exist.")

    def send_transforms(self, transforms: Iterable[sr.CartesianPose]) -> None:
        """
        Send a list of transforms to TF.

        :param transforms: The list of transforms to send
        """
        self.__publish_transforms(transforms)

    def send_transform(self, transform: sr.CartesianPose):
        """
        Send a transform to TF.

        :param transform: The transform to send
        """
        self.send_transforms([transform])

    def send_static_transforms(self, transforms: Iterable[sr.CartesianPose]) -> None:
        """
        Send a list of static transforms to TF.

        :param transforms: The list of transforms to send
        """
        self.__publish_transforms(transforms, static=True)

    def send_static_transform(self, transform: sr.CartesianPose):
        """
        Send a static transform to TF.

        :param transform: The transform to send
        """
        self.send_static_transforms([transform])

    def lookup_transform(self, frame: str, reference_frame="world", validity_period=-1.0, time_point=Time(),
                         duration=Duration(nanoseconds=1e4)) -> sr.CartesianPose:
        """
        Look up a transform from TF.

        :param frame: The desired frame of the transform
        :param reference_frame: The desired reference frame of the transform
        :param validity_period: The validity period of the latest transform from the time of lookup in seconds
        :param time_point: The time at which the value of the transform is desired (default: 0, will get the latest)
        :param duration: How long to block the lookup call before failing
        :return: If it exists and is still valid, the requested transform
        :raises LookupTransformError if TF buffer/listener are unconfigured if the lookupTransform call failed,
        or if the transform is too old
        """
        if not self.__tf_buffer or not self.__tf_listener:
            raise LookupTransformError("Failed to lookup transform: To TF buffer / listener configured.")
        try:
            transform = self.__tf_buffer.lookup_transform(reference_frame, frame, time_point, duration)
        except TransformException as e:
            raise LookupTransformError(f"Failed to lookup transform: {e}")
        if 0.0 < validity_period < (self.get_clock().now() - Time().from_msg(transform.header.stamp)).nanoseconds / 1e9:
            raise LookupTransformError("Failed to lookup transform: Latest transform is too old!")
        result = sr.CartesianPose(frame, reference_frame)
        modulo_readers.read_stamped_message(result, transform)
        return result

    def get_qos(self) -> QoSProfile:
        """
        Getter of the Quality of Service attribute.
        """
        return self.__qos

    def set_qos(self, qos: QoSProfile) -> None:
        """
        Setter of the Quality of Service for ROS publishers and subscribers.

        :param qos: The desired Quality of Service
        """
        self.__qos = qos

    def add_periodic_callback(self, name: str, callback: Callable[[], None]) -> None:
        """
        Add a periodic callback function. The provided function is evaluated periodically at the component step period.

        :param name: The name of the callback
        :param callback: The callback function that is evaluated periodically
        """
        if not name:
            self.get_logger().error("Failed to add periodic function: Provide a non empty string as a name.")
            return
        if name in self.__periodic_callbacks.keys():
            self.get_logger().warn(f"Periodic function '{name}' already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding periodic function '{name}'.")
        self.__periodic_callbacks[name] = callback

    def __get_predicate_message(self, name: str, value: bool) -> Predicate:
        """
        Populate a Predicate messag with the name and the value of a predicate.

        :param name: The name of the predicate
        :param value: The value of the predicate
        """
        message = PredicateMsg()
        message.predicate = name
        message.value = value
        return message

    def __publish_predicate(self, name: str, value: bool) -> None:
        """
        Helper function to publish a predicate.

        :param name: The name of the predicate to publish
        :param value: The value of the predicate
        """
        message = copy.copy(self.__predicate_message)
        message.predicates = [self.__get_predicate_message(name, value)]
        self.__predicate_publisher.publish(message)

    def __publish_predicates(self) -> None:
        """
        Helper function to publish all predicates.
        """
        message = copy.deepcopy(self.__predicate_message)
        for name in self.__predicates.keys():
            new_value = self.__predicates[name].query()
            if new_value is not None:
                message.predicates.append(self.__get_predicate_message(name, new_value))
        if len(message.predicates):
            self.__predicate_publisher.publish(message)

    def __translate_and_publish(self, output_name: str) -> None:
        """
        Translate and publish a message of an output

        :param output_name: The name of the output
        """
        message = self.__outputs[output_name]["message_type"]()
        data = self.__getattribute__(self.__outputs[output_name]["attribute"])
        # only publish if the data is not empty
        if not getattr(data, "is_empty", lambda: False)():
            self.__outputs[output_name]["translator"](message, data)
            self.__outputs[output_name]["publisher"].publish(message)

    def publish_output(self, signal_name: str) -> None:
        """
        Trigger the publishing of an output

        :param signal_name: The name of the output signal
        :raises CoreError: if the output is being published periodically or if the signal name could not be found
        """
        parsed_signal_name = parse_topic_name(signal_name)
        if parsed_signal_name not in self.__outputs.keys():
            raise CoreError(f"Output with name '{signal_name}' doesn't exist")
        if self.__periodic_outputs[parsed_signal_name]:
            raise CoreError("An output that is published periodically cannot be triggered manually")
        try:
            self.__translate_and_publish(parsed_signal_name)
        except Exception as e:
            self.get_logger().error(f"Failed to publish output '{parsed_signal_name}': {e}")

    def __publish_outputs(self) -> None:
        """
        Helper function to publish all outputs.
        """
        for output in self.__outputs.keys():
            try:
                if self.__periodic_outputs[output]:
                    self.__translate_and_publish(output)
            except Exception as e:
                self.get_logger().error(f"{e}")

    def __evaluate_periodic_callbacks(self) -> None:
        """
        Helper function to evaluate all periodic function callbacks.
        """
        for name, callback in self.__periodic_callbacks.items():
            try:
                callback()
            except Exception as e:
                self.get_logger().error(f"Failed to evaluate periodic function callback '{name}': {e}",
                                        throttle_duration_sec=1.0)

    def __publish_transforms(self, transforms: Iterable[sr.CartesianPose], static=False) -> None:
        """
        Send a list of transforms to TF using the normal or static tf broadcaster

        :param transforms: The list of transforms to send
        :param static: If true, use the static tf broadcaster instead of the normal one
        """
        tf_broadcaster = self.__static_tf_broadcaster if static else self.__tf_broadcaster
        modifier = 'static ' if static else ''
        if not tf_broadcaster:
            self.get_logger().error(f"Failed to send {modifier}transform: No {modifier}TF broadcaster configured.",
                                    throttle_duration_sec=1.0)
            return
        try:
            transform_messages = []
            for tf in transforms:
                transform_message = TransformStamped()
                modulo_writers.write_stamped_message(transform_message, tf, self.get_clock().now())
                transform_messages.append(transform_message)
            tf_broadcaster.sendTransform(transform_messages)
        except (MessageTranslationError, TransformException) as e:
            self.get_logger().error(f"Failed to send {modifier}transform: {e}", throttle_duration_sec=1.0)

    def raise_error(self) -> None:
        """
        Notify an error in the component.
        """
        self.get_logger().error("An error was raised in the component.")

    def __finalize_interfaces(self) -> None:
        """
        Finalize all interfaces.
        """
        self.__inputs = {}
        self.__outputs = {}
        self.__services_dict = {}
        self.__tf_buffer = None
        self.__tf_listener = None
        self.__tf_broadcaster = None
        self.__static_tf_broadcaster = None
        self.__predicate_publisher = None
        if self.__step_timer:
            self.__step_timer.cancel()
        self.__step_timer = None
