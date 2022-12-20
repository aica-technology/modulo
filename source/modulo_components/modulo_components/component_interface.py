import inspect
import sys
from functools import partial
from typing import Callable, Dict, List, Optional, TypeVar, Union, Iterable

import clproto
import modulo_core.translators.message_readers as modulo_readers
import modulo_core.translators.message_writers as modulo_writers
import state_representation as sr
from geometry_msgs.msg import TransformStamped
from modulo_component_interfaces.srv import EmptyTrigger, StringTrigger
from modulo_components.exceptions import AddServiceError, AddSignalError, ComponentParameterError, LookupTransformError
from modulo_components.utilities import generate_predicate_topic, parse_topic_name
from modulo_core import EncodedState
from modulo_core.exceptions import MessageTranslationError, ParameterTranslationError
from modulo_core.translators.parameter_translators import get_ros_parameter_type, read_parameter_const, write_parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
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
        self._parameter_dict: Dict[str, Union[str, sr.Parameter]] = {}
        self._predicates: Dict[str, Union[bool, Callable[[], bool]]] = {}
        self._predicate_publishers: Dict[str, Publisher] = {}
        self._triggers: Dict[str, bool] = {}
        self._periodic_callbacks: Dict[str, Callable[[], None]] = {}
        self._inputs = {}
        self._outputs = {}
        self._services_dict: Dict[str, Service] = {}
        self.__tf_buffer: Optional[Buffer] = None
        self.__tf_listener: Optional[TransformListener] = None
        self.__tf_broadcaster: Optional[TransformBroadcaster] = None
        self.__static_tf_broadcaster: Optional[StaticTransformBroadcaster] = None

        self._qos = QoSProfile(depth=10)

        self.add_on_set_parameters_callback(self.__on_set_parameters_callback)
        self.add_parameter(sr.Parameter("period", 0.1, sr.ParameterType.DOUBLE),
                           "Period (in s) between step function calls.")

        self.add_predicate("in_error_state", False)

        self.create_timer(self.get_parameter_value("period"), self._step)

    def _step(self) -> None:
        """
        Step function that is called periodically.
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
        :raises ComponentParameterError: if the parameter could not be added
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
            raise ComponentParameterError(f"Failed to add parameter: {e}")
        if not self.has_parameter(sr_parameter.get_name()):
            self.get_logger().debug(f"Adding parameter '{sr_parameter.get_name()}'.")
            self._parameter_dict[sr_parameter.get_name()] = parameter
            try:
                descriptor = ParameterDescriptor(description=description, read_only=read_only)
                if sr_parameter.is_empty():
                    descriptor.dynamic_typing = True
                    descriptor.type = get_ros_parameter_type(sr_parameter.get_parameter_type()).value
                    self.declare_parameter(ros_param.name, None, descriptor=descriptor)
                else:
                    self.declare_parameter(ros_param.name, ros_param.value, descriptor=descriptor)
            except Exception as e:
                del self._parameter_dict[sr_parameter.get_name()]
                raise ComponentParameterError(f"Failed to add parameter: {e}")
        else:
            self.get_logger().debug(f"Parameter '{sr_parameter.get_name()}' already exists.")

    def get_parameter(self, name: str) -> Union[sr.Parameter, Parameter]:
        """
        Get a parameter by name. If this method is called from a file that contains 'rclpy' in its path, the
        rclpy.node.Node.get_parameter method will be invoked, otherwise return the parameter from the local parameter
        dictionary.

        :param name: The name of the parameter
        :raises ComponentParameterError: if the parameter does not exist
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
            raise ComponentParameterError(f"Failed to get parameter '{name}': {e}")

    def __get_component_parameter(self, name: str) -> sr.Parameter:
        """
        Get the parameter from the parameter dictionary by its name.

        :param name: The name of the parameter
        :raises ComponentParameterError: if the parameter does not exist
        :return: The parameter, if it exists
        """
        if name not in self._parameter_dict.keys():
            raise ComponentParameterError(f"Parameter '{name}' is not in the dict of parameters")
        try:
            if isinstance(self._parameter_dict[name], str):
                return self.__getattribute__(self._parameter_dict[name])
            else:
                return self._parameter_dict[name]
        except AttributeError as e:
            raise ComponentParameterError(f"{e}")

    def get_parameter_value(self, name: str) -> T:
        """
        Get the parameter value from the parameter dictionary by its name.

        :param name: The name of the parameter
        :raises ComponentParameterError: if the parameter does not exist
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
        if parameter.get_name() == "period":
            value = parameter.get_value()
            if value <= 0.0 or value > sys.float_info.max:
                self.get_logger().error("Value for parameter 'period' has to be a positive finite number.")
                return False
        return self.on_validate_parameter_callback(parameter)

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
                    if isinstance(self._parameter_dict[ros_param.name], str):
                        self.__setattr__(self._parameter_dict[ros_param.name], new_parameter)
                    else:
                        self._parameter_dict[ros_param.name] = new_parameter
            except Exception as e:
                result.successful = False
                result.reason += str(e)
        return result

    def add_predicate(self, name: str, value: Union[bool, Callable[[], bool]]):
        """
        Add a predicate to the map of predicates.

        :param name: The name of the associated predicate
        :param value: The value of the predicate as a bool or a callable function
        """
        if not name:
            self.get_logger().error("Failed to add predicate: Provide a non empty string as a name.")
        if name in self._predicates.keys():
            self.get_logger().warn(f"Predicate with name '{name}' already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding predicate '{name}'.")
            self._predicate_publishers[name] = self.create_publisher(Bool,
                                                                     generate_predicate_topic(self.get_name(), name),
                                                                     10)
        self._predicates[name] = value

    def get_predicate(self, name: str) -> bool:
        """
        Get the value of the predicate given as parameter. If the predicate is not found or the callable function fails,
        this method returns False.

        :param name: The name of the predicate to retrieve from the map of predicates
        :return: The value of the predicate as a boolean
        """
        if name not in self._predicates.keys():
            self.get_logger().error(f"Predicate {name} does not exist, returning false.",
                                    throttle_duration_sec=1.0)
            return False
        value = self._predicates[name]
        if callable(value):
            bool_value = False
            try:
                bool_value = value()
            except Exception as e:
                self.get_logger().error(f"Error while evaluating the callback function: {e}",
                                        throttle_duration_sec=1.0)
            return bool_value
        return value

    def set_predicate(self, name: str, value: Union[bool, Callable[[], bool]]):
        """
        Set the value of the predicate given as parameter, if the predicate is not found does not do anything. Even
        though the predicates are published periodically, the new value of this predicate will be published once
        immediately after setting it.

        :param name: The name of the predicate to retrieve from the map of predicates
        :param value: The new value of the predicate as a bool or a callable function
        """
        if name not in self._predicates.keys():
            self.get_logger().error(
                f"Cannot set predicate {name} with a new value because it does not exist.",
                throttle_duration_sec=1.0)
            return
        self._predicates[name] = value
        self._publish_predicate(name)

    def add_trigger(self, trigger_name: str):
        """
        Add a trigger to the component. Triggers are predicates that are always false except when it's triggered in
        which case it is set back to false immediately after it is read.

        :param trigger_name: The name of the trigger
        """
        if not trigger_name:
            self.get_logger().error("Failed to add trigger: Provide a non empty string as a name.")
            return
        if trigger_name in self._triggers.keys() or trigger_name in self._predicates.keys():
            self.get_logger().error(
                f"Failed to add trigger: there is already a trigger or predicate with name '{trigger_name}'.")
            return
        self._triggers[trigger_name] = False
        self.add_predicate(trigger_name, partial(self.__get_trigger_value, trigger_name=trigger_name))

    def __get_trigger_value(self, trigger_name: str) -> bool:
        """
        Get the trigger value and set it to false independent of the previous value.

        :param trigger_name: The name of the trigger
        :return: The value of the trigger
        """
        value = self._triggers[trigger_name]
        self._triggers[trigger_name] = False
        return value

    def trigger(self, trigger_name: str):
        """
        Latch the trigger with the provided name.

        :param trigger_name: The name of the trigger
        """
        if trigger_name not in self._triggers.keys():
            self.get_logger().error(f"Failed to trigger: could not find trigger with name '{trigger_name}'.")
            return
        self._triggers[trigger_name] = True
        self._publish_predicate(trigger_name)

    def remove_output(self, signal_name):
        if signal_name not in self._outputs.keys():
            parsed_signal_name = parse_topic_name(signal_name)
            if parsed_signal_name not in self._outputs.keys():
                self.get_logger().debug(f"Unknown output '{signal_name}' (parsed name was '{parsed_signal_name}').")
                return
            signal_name = parsed_signal_name
        if "publisher" in self._outputs[signal_name].keys():
            self.destroy_publisher(self._outputs[signal_name]["publisher"])
        self._outputs.pop(signal_name)
        self.get_logger().debug(f"Removing signal '{signal_name}'.")

    def _create_output(self, signal_name: str, data: str, message_type: MsgT, clproto_message_type: clproto.MessageType,
                       default_topic: str, fixed_topic: bool) -> str:
        """
        Helper function to parse the signal name and add an output without Publisher to the dict of outputs.

        :param signal_name: Name of the output signal
        :param data: Name of the attribute to transmit over the channel
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :raises AddSignalError: if there is a problem adding the output
        :return: The parsed signal name
        """
        try:
            if message_type == EncodedState and clproto_message_type == clproto.MessageType.UNKNOWN_MESSAGE:
                raise AddSignalError(f"Provide a valid clproto message type for outputs of type EncodedState.")
            parsed_signal_name = parse_topic_name(signal_name)
            self.declare_output(parsed_signal_name, default_topic, fixed_topic)
            if message_type == Bool or message_type == Float64 or \
                    message_type == Float64MultiArray or message_type == Int32 or message_type == String:
                translator = modulo_writers.write_std_message
            elif message_type == EncodedState:
                translator = partial(modulo_writers.write_clproto_message,
                                     clproto_message_type=clproto_message_type)
            else:
                raise AddSignalError("The provided message type is not supported to create a component output.")
            self._outputs[parsed_signal_name] = {"attribute": data, "message_type": message_type,
                                                 "translator": translator}
            return parsed_signal_name
        except AddSignalError:
            raise
        except Exception as e:
            raise AddSignalError(f"{e}")

    def remove_input(self, signal_name: str):
        if not self.destroy_subscription(self._inputs.pop(signal_name, None)):
            parsed_signal_name = parse_topic_name(signal_name)
            if not self.destroy_subscription(self._inputs.pop(parsed_signal_name, None)):
                self.get_logger().debug(f"Unknown input '{signal_name}' (parsed name was '{parsed_signal_name}').")
                return
            self.get_logger().debug(f"Removing signal '{parsed_signal_name}'.")
            return
        self.get_logger().debug(f"Removing signal '{signal_name}'.")

    def __subscription_callback(self, message: MsgT, attribute_name: str, reader: Callable, user_callback: Callable):
        """
        Subscription callback for the ROS subscriptions.

        :param message: The message from the ROS network
        :param attribute_name: The name of the attribute that is updated by the subscription
        :param reader: A callable that can read the ROS message and translate to the desired type
        """
        try:
            self.__setattr__(attribute_name, reader(message))
        except (AttributeError, MessageTranslationError) as e:
            self.get_logger().warn(f"Failed to read message for attribute {attribute_name}: {e}",
                                   throttle_duration_sec=1.0)
            return
        try:
            user_callback()
        except Exception as e:
            self.get_logger().error(f"Failed to execute user callback in subscription for attribute"
                                    f" '{attribute_name}': {e}", throttle_duration_sec=1.0)

    def declare_signal(self, signal_name: str, signal_type: str, default_topic="", fixed_topic=False):
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
            raise AddSignalError(f"The parsed signal name for {signal_type} '{signal_name}' is empty. Provide a "
                                 f"string with valid characters for the signal name ([a-zA-Z0-9_]).")
        if parsed_signal_name in self._inputs.keys():
            raise AddSignalError(f"Signal with name '{parsed_signal_name}' already exists as input.")
        if parsed_signal_name in self._outputs.keys():
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

    def declare_input(self, signal_name: str, default_topic="", fixed_topic=False):
        """
        Declare an input to create the topic parameter without adding it to the map of inputs yet.

        :param signal_name: The signal name of the input
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the signal is fixed
        :raises AddSignalError: if the input could not be declared (empty name or already created)
        """
        self.declare_signal(signal_name, "input", default_topic, fixed_topic)

    def declare_output(self, signal_name: str, default_topic="", fixed_topic=False):
        """
        Declare an output to create the topic parameter without adding it to the map of outputs yet.

        :param signal_name: The signal name of the output
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the signal is fixed
        :raises AddSignalError: if the output could not be declared (empty name or already created)
        """
        self.declare_signal(signal_name, "output", default_topic, fixed_topic)

    def add_input(self, signal_name: str, subscription: Union[str, Callable], message_type: MsgT, default_topic="",
                  fixed_topic=False, user_callback: Callable = None):
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
            parsed_signal_name = parse_topic_name(signal_name)
            self.declare_input(parsed_signal_name, default_topic, fixed_topic)
            topic_name = self.get_parameter_value(parsed_signal_name + "_topic")
            self.get_logger().debug(f"Adding input '{parsed_signal_name}' with topic name '{topic_name}'.")
            if isinstance(subscription, Callable):
                if user_callback:
                    self.get_logger().warn("Providing a callable for arguments 'subscription' and 'user_callback' is"
                                           "not supported. The user callback will be ignored.")
                self._inputs[parsed_signal_name] = self.create_subscription(message_type, topic_name, subscription,
                                                                            self._qos)
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
                    self._inputs[parsed_signal_name] = \
                        self.create_subscription(message_type, topic_name,
                                                 partial(self.__subscription_callback,
                                                         attribute_name=subscription,
                                                         reader=modulo_readers.read_std_message,
                                                         user_callback=user_callback),
                                                 self._qos)
                elif message_type == EncodedState:
                    self._inputs[parsed_signal_name] = \
                        self.create_subscription(message_type, topic_name,
                                                 partial(self.__subscription_callback,
                                                         attribute_name=subscription,
                                                         reader=modulo_readers.read_clproto_message,
                                                         user_callback=user_callback),
                                                 self._qos)
                else:
                    raise TypeError("The provided message type is not supported to create a component input.")
            else:
                raise TypeError("Provide either a string containing the name of an attribute or a callable.")
        except Exception as e:
            self.get_logger().error(f"Failed to add input '{signal_name}': {e}")

    def add_service(self, service_name: str, callback: Union[Callable[[], dict], Callable[[str], dict]]):
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
                raise AddServiceError(f"The parsed signal name for service {service_name} is empty. Provide a "
                                      f"string with valid characters for the service name ([a-zA-Z0-9_]).")
            if parsed_service_name in self._services_dict.keys():
                raise AddServiceError(f"Service with name '{parsed_service_name}' already exists.")
            signature = inspect.signature(callback)
            if len(signature.parameters) == 0:
                self.get_logger().debug(f"Adding empty service '{parsed_service_name}'.")
                service_type = EmptyTrigger
            else:
                self.get_logger().debug(f"Adding string service '{parsed_service_name}'.")
                service_type = StringTrigger
            self._services_dict[parsed_service_name] = \
                self.create_service(service_type, "~/" + parsed_service_name,
                                    lambda request, response: callback_wrapper(request, response, callback))
        except Exception as e:
            self.get_logger().error(f"Failed to add service '{service_name}': {e}")

    def add_tf_broadcaster(self):
        """
        Configure a transform broadcaster.
        """
        if not self.__tf_broadcaster:
            self.get_logger().debug("Adding TF broadcaster.")
            self.__tf_broadcaster = TransformBroadcaster(self)
        else:
            self.get_logger().error("TF broadcaster already exists.")

    def add_static_tf_broadcaster(self):
        """
        Configure a static transform broadcaster.
        """
        if not self.__static_tf_broadcaster:
            self.get_logger().debug("Adding static TF broadcaster.")
            self.__static_tf_broadcaster = StaticTransformBroadcaster(self)
        else:
            self.get_logger().error("TF broadcaster already exists.")

    def add_tf_listener(self):
        """
        Configure a transform buffer and listener.
        """
        if not self.__tf_buffer or not self.__tf_listener:
            self.get_logger().debug("Adding TF buffer and listener.")
            self.__tf_buffer = Buffer()
            self.__tf_listener = TransformListener(self.__tf_buffer, self)
        else:
            self.get_logger().error("TF buffer and listener already exist.")

    def send_transforms(self, transforms: Iterable[sr.CartesianPose]):
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

    def send_static_transforms(self, transforms: Iterable[sr.CartesianPose]):
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
        :param validity_period: The validity period of the latest transform from the time of lookup
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
        return self._qos

    def set_qos(self, qos: QoSProfile):
        """
        Setter of the Quality of Service for ROS publishers and subscribers.

        :param qos: The desired Quality of Service
        """
        self._qos = qos

    def add_periodic_callback(self, name: str, callback: Callable[[], None]):
        """
        Add a periodic callback function. The provided function is evaluated periodically at the component step period.

        :param name: The name of the callback
        :param callback: The callback function that is evaluated periodically
        """
        if not name:
            self.get_logger().error("Failed to add periodic function: Provide a non empty string as a name.")
            return
        if name in self._periodic_callbacks.keys():
            self.get_logger().warn(f"Periodic function '{name}' already exists, overwriting.")
        else:
            self.get_logger().debug(f"Adding periodic function '{name}'.")
        self._periodic_callbacks[name] = callback

    def _publish_predicate(self, name):
        """
        Helper function to publish a predicate.

        :param name: The name of the predicate to publish
        """
        message = Bool()
        value = self.get_predicate(name)
        try:
            message.data = value
        except AssertionError:
            self.get_logger().error(f"Predicate '{name}' has invalid type: expected 'bool', got '{type(value)}'.",
                                    throttle_duration_sec=1.0)
            return
        if name not in self._predicate_publishers.keys():
            self.get_logger().error(f"No publisher for predicate '{name}' found.", throttle_duration_sec=1.0)
            return
        self._predicate_publishers[name].publish(message)

    def _publish_predicates(self):
        """
        Helper function to publish all predicates.
        """
        for name in self._predicates.keys():
            self._publish_predicate(name)

    def _publish_outputs(self):
        """
        Helper function to publish all outputs.
        """
        for signal, output_dict in self._outputs.items():
            try:
                message = output_dict["message_type"]()
                data = self.__getattribute__(output_dict["attribute"])
                # only publish if the data is not empty
                if not getattr(data, "is_empty", lambda: False)():
                    output_dict["translator"](message, data)
                    output_dict["publisher"].publish(message)
            except Exception as e:
                self.get_logger().error(f"{e}")

    def _evaluate_periodic_callbacks(self):
        """
        Helper function to evaluate all periodic function callbacks.
        """
        for name, callback in self._periodic_callbacks.items():
            try:
                callback()
            except Exception as e:
                self.get_logger().error(f"Failed to evaluate periodic function callback '{name}': {e}",
                                        throttle_duration_sec=1.0)

    def __publish_transforms(self, transforms: Iterable[sr.CartesianPose], static=False):
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

    def raise_error(self):
        """
        Put the component in error state by setting the 'in_error_state' predicate to true.
        """
        self.set_predicate("in_error_state", True)
