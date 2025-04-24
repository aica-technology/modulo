from typing import Optional, TypeVar

import clproto
from lifecycle_msgs.msg import State
from modulo_components.component_interface import ComponentInterface
from modulo_core.exceptions import AddSignalError
from rclpy.callback_groups import CallbackGroup
from rclpy.lifecycle import LifecycleNodeMixin, LifecycleState
from rclpy.lifecycle.node import TransitionCallbackReturn

MsgT = TypeVar('MsgT')
LIFECYCLE_NODE_MIXIN_KWARGS = ["enable_communication_interface", "callback_group"]


class _LifecycleNodeMixin(LifecycleNodeMixin):
    def __init__(self, enable_communication_interface: bool = True, callback_group: Optional[CallbackGroup] = None):
        super().__init__(enable_communication_interface=enable_communication_interface, callback_group=callback_group)

    def destroy(self):
        self._state_machine = None
        self._callbacks = {}


class LifecycleComponent(ComponentInterface, _LifecycleNodeMixin):
    """
    Class to represent a LifecycleComponent in python, following the same logic pattern
    as the C++ modulo_components::LifecycleComponent class.
    """

    def __init__(self, node_name: str, *args, **kwargs):
        """
        Constructs all the necessary attributes and declare all the parameters.

        :param node_name: The name of the node to be passed to the base Node class
        """
        lifecycle_node_kwargs = {key: value for key, value in kwargs.items() if key in LIFECYCLE_NODE_MIXIN_KWARGS}
        ComponentInterface.__init__(self, node_name, *args, **kwargs)
        _LifecycleNodeMixin.__init__(self, **lifecycle_node_kwargs)
        self.__has_error = False

    def destroy_node(self) -> None:
        """
        Cleanly destroy the node by cleaning up the Mixin class.
        """
        _LifecycleNodeMixin.destroy(self)
        ComponentInterface.destroy_node(self)

    def get_lifecycle_state(self) -> LifecycleState:
        """
        Get the current state of the component.

        :return: The current state
        """
        return LifecycleState(self._state_machine.current_state[1], self._state_machine.current_state[0])

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'Configuring'.

        on_configure callback is called when the lifecycle component enters the 'Configuring' transition state.
        The component must be in the 'Unconfigured' state.
        Depending on the return value of this function, the component may either transition to the 'Inactive' state
        via the 'configure' transition, stay 'Unconfigured' or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'
        TRANSITION_CALLBACK_FAILURE transitions to 'Unconfigured'
        TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().debug(f"on_configure called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().warn(f"Invalid transition 'configure' from state {previous_state.label}")
            return TransitionCallbackReturn.FAILURE
        if not self.__handle_configure():
            self.get_logger().warn("Configuration failed! Reverting to the unconfigured state.")
            if self.__handle_cleanup():
                return TransitionCallbackReturn.FAILURE
            else:
                self.get_logger().error(
                    "Could not revert to the unconfigured state! Entering into the error processing transition state.")
                return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_configure(self) -> bool:
        """
        Handle the configure transition by related transition steps and invoking the user callback.

        :return: True if configuration is successful, false otherwise
        """
        try:
            result = self.on_configure_callback()
            if result is None:
                self.get_logger().error("'on_configure_callback' doesn't return a value, transition will be rejected.")
                return False
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False
        return result and self.__configure_outputs()

    def on_configure_callback(self) -> bool:
        """
        Steps to execute when configuring the component. This method can be overridden by derived Component classes.
        Configuration generally involves reading parameters and adding inputs and outputs.

        :return: True if configuration is successful, false otherwise
        """
        return True

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'CleaningUp'.

        on_cleanup callback is called when the lifecycle component enters the 'CleaningUp' transition state.
        The component must be in the 'Inactive' state.
        Depending on the return value of this function, the component may either transition to the 'Unconfigured' state
        via the 'cleanup' transition or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Unconfigured'
        TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().debug(f"on_cleanup called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_INACTIVE:
            self.get_logger().warn(f"Invalid transition 'cleanup' from state {previous_state.label}")
        if not self.__handle_cleanup():
            self.get_logger().warn("Cleanup failed! Entering into the error processing transition state.")
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_cleanup(self) -> bool:
        """
        Handle the cleanup transition by related transition steps and invoking the user callback.

        :return: True if cleanup is successful, false otherwise
        """
        try:
            result = self.on_cleanup_callback()
            if result is None:
                self.get_logger().error("'on_cleanup_callback' doesn't return a value, transition will be rejected.")
                return False
            return result
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False

    def on_cleanup_callback(self) -> bool:
        """
        Steps to execute when cleaning up the component. This method can be overridden by derived Component classes.
        Cleanup generally involves resetting the properties and states to initial conditions.

        :return: True if cleanup is successful, false otherwise
        """
        return True

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'Activating'.

        on_activate callback is called when the lifecycle component enters the 'Activating' transition state.
        The component must be in the 'Inactive' state.
        Depending on the return value of this function, the component may either transition to the 'Active' state
        via the 'activate' transition, stay 'Inactive' or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Active'
        TRANSITION_CALLBACK_FAILURE transitions to 'Inactive'
        TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().debug(f"on_activate called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_INACTIVE:
            self.get_logger().warn(f"Invalid transition 'activate' from state {previous_state.label}")
            return TransitionCallbackReturn.FAILURE
        if not self.__handle_activate():
            self.get_logger().warn("Activation failed! Reverting to the inactive state.")
            # perform deactivation actions to ensure the component is inactive
            if self.__handle_deactivate():
                return TransitionCallbackReturn.FAILURE
            else:
                self.get_logger().error(
                    "Could not revert to the inactive state! Entering into the error processing transition state.")
                return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_activate(self) -> bool:
        """
        Handle the activate transition by related transition steps and invoking the user callback.

        :return: True if activation is successful, false otherwise
        """
        try:
            result = self.on_activate_callback()
            if result is None:
                self.get_logger().error("'on_configure_callback' doesn't return a value, transition will be rejected.")
                return False
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False
        return result and self.__activate_outputs()

    def on_activate_callback(self) -> bool:
        """
        Steps to execute when activating the component. This method can be overridden by derived Component classes.
        Activation generally involves final setup steps before the on_step callback is periodically evaluated.

        :return: True if activation is successful, false otherwise
        """
        return True

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'Deactivating'.

        on_deactivate callback is called when the lifecycle component enters the 'Deactivating' transition state.
        The component must be in the 'Active' state.
        Depending on the return value of this function, the component may either transition to the 'Inactive' state
        via the 'deactivate' transition or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Inactive'
        TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """
        self.get_logger().debug(f"on_deactivate called from previous state {previous_state.label}.")
        if previous_state.state_id != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().warn(f"Invalid transition 'deactivate' from state {previous_state.label}")
            return TransitionCallbackReturn.FAILURE
        if not self.__handle_deactivate():
            self.get_logger().warn("Deactivation failed! Reverting to the inactive state.")
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def __handle_deactivate(self) -> bool:
        """
        Handle the deactivate transition by related transition steps and invoking the user callback.

        :return: True if deactivation is successful, false otherwise
        """
        result = self.__deactivate_outputs()
        try:
            cb_result = self.on_deactivate_callback()
            if cb_result is None:
                self.get_logger().error("'on_deactivate_callback' doesn't return a value, transition will be rejected.")
                return False
            return result and cb_result
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False

    def on_deactivate_callback(self) -> bool:
        """
        Steps to execute when deactivating the component. This method can be overridden by derived Component classes.
        Deactivation generally involves any steps to reset the component to an inactive state.

        :return: True if deactivation is successful, false otherwise
        """
        return True

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'ShuttingDown'.

        on_shutdown callback is called when the lifecycle component enters the 'ShuttingDown' transition state.
        The component must be in the 'Unconfigured', 'Inactive' and 'Active' states.
        Depending on the return value of this function, the component may either transition to the 'Finalized' state
        via the 'shutdown' transition or go to 'ErrorProcessing'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Finalized'
        TRANSITION_CALLBACK_FAILURE, TRANSITION_CALLBACK_ERROR or any uncaught exceptions to 'ErrorProcessing'
        """

        def error_processing(self):
            self.get_logger().error("Entering into the error processing transition state.")
            return TransitionCallbackReturn.ERROR

        self.get_logger().debug(f"on_shutdown called from previous state {previous_state.label}.")
        if not self.__has_error:
            if previous_state.state_id == State.PRIMARY_STATE_FINALIZED:
                return TransitionCallbackReturn.SUCCESS
            if previous_state.state_id == State.PRIMARY_STATE_ACTIVE:
                if not self.__handle_deactivate():
                    self.get_logger().debug("Shutdown failed during intermediate deactivation!")
                    return error_processing(self)
                if not self.__handle_cleanup():
                    self.get_logger().debug("Shutdown failed during intermediate cleanup!")
                    return error_processing(self)
                if not self.__handle_shutdown():
                    return error_processing(self)
                self._finalize_component_interfaces()
                return TransitionCallbackReturn.SUCCESS
            if previous_state.state_id == State.PRIMARY_STATE_INACTIVE:
                if not self.__handle_cleanup():
                    self.get_logger().debug("Shutdown failed during intermediate cleanup!")
                    return error_processing(self)
                if not self.__handle_shutdown():
                    return error_processing(self)
                self._finalize_component_interfaces()
                return TransitionCallbackReturn.SUCCESS
            if previous_state.state_id == State.PRIMARY_STATE_UNCONFIGURED:
                if not self.__handle_shutdown():
                    return error_processing(self)
                self._finalize_component_interfaces()
                return TransitionCallbackReturn.SUCCESS
            self.get_logger().warn(f"Invalid transition 'shutdown' from state {previous_state.label}.")
        return error_processing(self)

    def __handle_shutdown(self) -> bool:
        """
        Handle the shutdown transition by related transition steps and invoking the user callback.

        :return: True if shutdown is successful, false otherwise
        """
        try:
            result = self.on_shutdown_callback()
            if result is None:
                self.get_logger().error("'on_shutdown_callback' doesn't return a value, transition will be rejected.")
                return False
            return result
        except Exception as e:
            self.get_logger().error(f"{e}")
            return False

    def on_shutdown_callback(self) -> bool:
        """
        Steps to execute when shutting down the component. This method can be overridden by derived Component classes.
        Shutdown generally involves the destruction of any threads or properties not handled by the base class.

        :return: True if shutdown is successful, false otherwise
        """
        return True

    def on_error(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        """
        Transition callback for state 'ErrorProcessing'.

        on_shutdown callback is called when the lifecycle component enters the 'ErrorProcessing' transition state.
        This transition can originate from any step.
        Depending on the return value of this function, the component may either transition to the 'Unconfigured' state
        or go to 'Finalized'.
        TRANSITION_CALLBACK_SUCCESS transitions to 'Unconfigured'
        TRANSITION_CALLBACK_FAILURE transitions to 'Finalized'
        TRANSITION_CALLBACK_ERROR should not be returned, and any exceptions should be caught and returned as a failure
        """
        self.get_logger().debug(f"on_error called from previous state {previous_state.label}.")
        error_handled = False
        try:
            error_handled = self.__handle_error()
        except Exception as e:
            self.get_logger().debug(f"Exception caught during on_error handling: {e}")
            error_handled = False
        if not error_handled:
            self.get_logger().error("Error processing failed! Entering into the finalized state.")
            self._ComponentInterface__finalize_component_interfaces()
            return TransitionCallbackReturn.ERROR
        self.__has_error = False
        return TransitionCallbackReturn.SUCCESS

    def __handle_error(self) -> bool:
        """
        Handle the error transition by related transition steps and invoking the user callback.

        :return: True if error handling is successful, false otherwise
        """
        result = self.on_error_callback()
        if result is None:
            self.get_logger().error("'on_error_callback' doesn't return a value, transition will be rejected.")
            return False
        return result

    def on_error_callback(self) -> bool:
        """
        Steps to execute when handling errors. This method can be overridden by derived Component classes.
        Error handling generally involves recovering and resetting the component to an unconfigured state.

        :return: True if error handling is successful, false otherwise
        """
        return False

    def _step(self) -> None:
        """
        Step function that is called periodically and publishes predicates, outputs, evaluates daemon callbacks, and
        calls the on_step function.
        """
        try:
            if self.get_lifecycle_state().state_id == State.PRIMARY_STATE_ACTIVE:
                self._ComponentInterface__evaluate_periodic_callbacks()
                self.on_step_callback()
                self._ComponentInterface__publish_outputs()
            self._ComponentInterface__publish_predicates()
        except Exception as e:
            self.get_logger().error(f"Failed to execute step function: {e}")
            self.raise_error()

    def __configure_outputs(self) -> bool:
        """
        Configure all outputs.

        :return: True if configuration was successful
        """
        success = True
        for signal_name, output_dict in self._ComponentInterface__outputs.items():
            try:
                topic_name = self.get_parameter_value(signal_name + "_topic")
                self.get_logger().debug(f"Configuring output '{signal_name}' with topic name '{topic_name}'.")
                publisher = self.create_lifecycle_publisher(msg_type=output_dict["message_type"], topic=topic_name,
                                                            qos_profile=self.get_qos())
                self._ComponentInterface__set_output_publisher(signal_name, publisher)
            except Exception as e:
                success = False
                self.get_logger().debug(f"Failed to configure output '{signal_name}': {e}")
        return success

    def add_output(self, signal_name: str, data: str, message_type: MsgT,
                   clproto_message_type: Optional[clproto.MessageType] = None, default_topic="", fixed_topic=False,
                   publish_on_step=True) -> None:
        """
        Add an output signal of the component.

        :param signal_name: Name of the output signal
        :param data: Name of the attribute to transmit over the channel
        :param message_type: The ROS message type of the output
        :param clproto_message_type: The clproto message type, if applicable
        :param default_topic: If set, the default value for the topic name to use
        :param fixed_topic: If true, the topic name of the output signal is fixed
        :param publish_on_step: If true, the output is published periodically on step
        """
        if self.get_lifecycle_state().state_id not in [State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE]:
            self.get_logger().warn(f"Adding output in state {self.get_lifecycle_state().label} is not allowed.",
                                   throttle_duration_sec=1.0)
            return
        try:
            parsed_signal_name = self._ComponentInterface__create_output(
                signal_name, data, message_type, clproto_message_type, default_topic, fixed_topic, publish_on_step)
            topic_name = self.get_parameter_value(parsed_signal_name + "_topic")
            self.get_logger().debug(f"Adding output '{parsed_signal_name}' with topic name '{topic_name}'.")
        except AddSignalError as e:
            self.get_logger().error(f"Failed to add output '{signal_name}': {e}")

    def __activate_outputs(self) -> None:
        success = True
        state = self.get_lifecycle_state().state_id
        for signal_name, output_dict in self._ComponentInterface__outputs.items():
            try:
                output_dict["publisher"].on_activate(state)
            except Exception as e:
                success = False
                self.get_logger().error(f"Failed to activate output '{signal_name}': {e}")
        self.get_logger().debug("All outputs activated.")
        return success

    def __deactivate_outputs(self) -> None:
        success = True
        state = self.get_lifecycle_state().state_id
        for signal_name, output_dict in self._ComponentInterface__outputs.items():
            try:
                output_dict["publisher"].on_deactivate(state)
            except Exception as e:
                success = False
                self.get_logger().error(f"Failed to deactivate output '{signal_name}': {e}")
        self.get_logger().debug("All outputs deactivated.")
        return success

    def raise_error(self) -> None:
        """
        Trigger the shutdown and error transitions.
        """
        super().raise_error()
        self.__has_error = True
        self.trigger_shutdown()
