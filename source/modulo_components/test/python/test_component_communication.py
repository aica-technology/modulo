import pytest
from modulo_components.component import Component
from modulo_core.exceptions import CoreError


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[Component, "/topic", True]], indirect=True)
def test_input_output(ros_exec, random_pose, minimal_cartesian_output, minimal_cartesian_input):
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert minimal_cartesian_input.received_future.done() and minimal_cartesian_input.received_future.result()
    assert random_pose.get_name() == minimal_cartesian_input.input.get_name()
    assert random_pose.dist(minimal_cartesian_input.input) < 1e-3
    with pytest.raises(CoreError):
        minimal_cartesian_output.publish()


@pytest.mark.parametrize("exception_cartesian_input", [[Component, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[Component, "/topic", True]], indirect=True)
def test_exception_input_output(ros_exec, minimal_cartesian_output, exception_cartesian_input):
    ros_exec.add_node(exception_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    ros_exec.spin_until_future_complete(exception_cartesian_input.received_future, timeout_sec=0.5)
    assert exception_cartesian_input.received_future.done() and exception_cartesian_input.received_future.result()


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[Component, "/topic", False]], indirect=True)
def test_input_output_manual(ros_exec, random_pose, minimal_cartesian_output, minimal_cartesian_input):
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.done()
    minimal_cartesian_output.publish()
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert minimal_cartesian_input.received_future.done() and minimal_cartesian_input.received_future.result()
    assert random_pose.get_name() == minimal_cartesian_input.input.get_name()
    assert random_pose.dist(minimal_cartesian_input.input) < 1e-3


@pytest.mark.parametrize("minimal_sensor_input", [[Component, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_sensor_output", [[Component, "/topic", False]], indirect=True)
def test_input_output_manual_sensor(ros_exec, random_sensor, minimal_sensor_output, minimal_sensor_input):
    ros_exec.add_node(minimal_sensor_input)
    ros_exec.add_node(minimal_sensor_output)
    ros_exec.spin_until_future_complete(minimal_sensor_input.received_future, timeout_sec=0.5)
    assert not minimal_sensor_input.received_future.done()
    minimal_sensor_output.publish()
    ros_exec.spin_until_future_complete(minimal_sensor_input.received_future, timeout_sec=0.5)
    assert minimal_sensor_input.received_future.done() and minimal_sensor_input.received_future.result()
    for key in random_sensor.get_fields_and_field_types().keys():
        assert getattr(random_sensor, key) == getattr(minimal_sensor_input.input, key)


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_joint_output", [[Component, "/topic", True]], indirect=True)
def test_input_output_invalid_type(ros_exec, minimal_joint_output, minimal_cartesian_input):
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_joint_output)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.done()


@pytest.mark.parametrize("minimal_cartesian_input", [[Component, "/topic"]], indirect=True)
def test_input_output_invalid_msg(ros_exec, make_minimal_invalid_encoded_state_publisher, minimal_cartesian_input):
    invalid_publisher = make_minimal_invalid_encoded_state_publisher("/topic")
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(invalid_publisher)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.result()


def test_trigger(ros_exec, make_predicates_listener, make_minimal_trigger):
    trigger = make_minimal_trigger(Component)
    listener = make_predicates_listener("/trigger", ["test"])
    ros_exec.add_node(listener)
    ros_exec.add_node(trigger)
    ros_exec.spin_until_future_complete(listener.predicates_future, timeout_sec=0.5)
    assert not listener.predicates_future.done()
    assert not listener.predicate_values["test"]
    trigger.trigger("test")
    ros_exec.spin_until_future_complete(listener.predicates_future, timeout_sec=0.5)
    assert listener.predicates_future.done()
    assert listener.predicate_values["test"]
    assert not trigger.get_predicate("trigger")
