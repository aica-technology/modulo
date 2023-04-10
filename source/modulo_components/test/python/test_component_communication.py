import pytest
from modulo_components.component import Component


class Trigger(Component):
    def __init__(self):
        super().__init__("trigger")
        self.add_trigger("test")

    def trigger(self):
        super().trigger("test")


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[Component, "/topic"]], indirect=True)
def test_input_output(ros_exec, random_pose, minimal_cartesian_output, minimal_cartesian_input):
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    ros_exec.remove_node(minimal_cartesian_input)
    ros_exec.remove_node(minimal_cartesian_output)
    assert minimal_cartesian_input.received_future.result()
    assert random_pose.get_name() == minimal_cartesian_input.input.get_name()
    assert random_pose.dist(minimal_cartesian_input.input) < 1e-3


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
def test_input_output_invalid(ros_exec, make_minimal_invalid_encoded_state_publisher, minimal_cartesian_input):
    invalid_publisher = make_minimal_invalid_encoded_state_publisher("/topic")
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(invalid_publisher)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.result()


def test_trigger(ros_exec, make_predicates_listener):
    trigger = Trigger()
    listener = make_predicates_listener("trigger", ["test"])
    ros_exec.add_node(listener)
    ros_exec.add_node(trigger)
    ros_exec.spin_until_future_complete(listener.predicates_future, timeout_sec=0.5)
    assert not listener.predicates_future.done()
    assert not listener.predicate_values["test"]
    trigger.trigger()
    ros_exec.spin_until_future_complete(listener.predicates_future, timeout_sec=0.5)
    assert listener.predicates_future.done()
    assert listener.predicate_values["test"]
