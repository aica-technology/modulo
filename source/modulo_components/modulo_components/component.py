from threading import Thread
from typing import TypeVar

import clproto
from modulo_components.component_interface import ComponentInterface

MsgT = TypeVar('MsgT')


class Component(ComponentInterface):
    """
    Class to represent a Component in python, following the same logic pattern
    as the C++ modulo_components::Component class.
    """

    def __init__(self, node_name: str, *args, **kwargs):
        """
        Constructs all the necessary attributes and declare all the parameters.

        :param node_name: The name of the node to be passed to the base Node class
        """
        super().__init__(node_name, *args, **kwargs)
        self.__started = False
        self.__execute_thread = None
        self.add_predicate("is_finished", False)

    def _step(self):
        """
        Step function that is called periodically.
        """
        try:
            self._evaluate_periodic_callbacks()
            self._publish_outputs()
            self._publish_predicates()
        except Exception as e:
            self.get_logger().error(f"Failed to execute step function: {e}", throttle_duration_sec=1.0)

    def execute(self):
        """
        Start the execution thread.
        """
        if self.__started:
            self.get_logger().error("Failed to start execution thread: Thread has already been started.")
            return
        self.__started = True
        self.__execute_thread = Thread(target=self.__on_execute)
        self.__execute_thread.start()

    def __on_execute(self):
        """
        Run the execution function in a try catch block and set the predicates according to the outcome of the
        execution.
        """
        try:
            if not self.on_execute_callback():
                self.raise_error()
                return
        except Exception as e:
            self.get_logger().error(f"Failed to run the execute function: {e}")
            self.raise_error()
            return
        self.get_logger().debug("Execution finished, setting 'is_finished' predicate to true.")
        self.set_predicate("is_finished", True)

    def on_execute_callback(self) -> bool:
        """
        Execute the component logic. To be redefined in the derived classes.

        :return: True, if the execution was successful, false otherwise
        """
        return True

    def add_output(self, signal_name: str, data: str, message_type: MsgT,
                   clproto_message_type=clproto.MessageType.UNKNOWN_MESSAGE, default_topic="", fixed_topic=False):
        """
        Add and configure an output signal of the component.

        :param signal_name: Name of the output signal
        :param data: Name of the attribute to transmit over the channel
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the output signal is fixed
        """
        try:
            parsed_signal_name = self._create_output(signal_name, data, message_type, clproto_message_type,
                                                     default_topic, fixed_topic)
            topic_name = self.get_parameter_value(parsed_signal_name + "_topic")
            self.get_logger().debug(f"Adding output '{parsed_signal_name}' with topic name '{topic_name}'.")
            publisher = self.create_publisher(message_type, topic_name, self._qos)
            self._outputs[parsed_signal_name]["publisher"] = publisher
        except Exception as e:
            self.get_logger().error(f"Failed to add output '{signal_name}': {e}")
