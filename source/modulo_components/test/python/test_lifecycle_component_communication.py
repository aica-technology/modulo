import pytest
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_utils.exceptions import ModuloError


class Trigger(LifecycleComponent):
    def __init__(self):
        super().__init__("trigger")

    def on_configure_callback(self) -> bool:
        self.add_trigger("test")
        return True

    def on_activate_callback(self) -> bool:
        self.trigger("test")
        return True


@pytest.mark.parametrize("minimal_cartesian_input", [[LifecycleComponent, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[LifecycleComponent, "/topic", True]], indirect=True)
def test_input_output(ros_exec, make_lifecycle_change_client, random_pose, minimal_cartesian_output,
                      minimal_cartesian_input):
    input_change_client = make_lifecycle_change_client("minimal_cartesian_input")
    output_change_client = make_lifecycle_change_client("minimal_cartesian_output")
    ros_exec.add_node(input_change_client)
    ros_exec.add_node(output_change_client)
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    input_change_client.configure(ros_exec)
    output_change_client.configure(ros_exec)
    input_change_client.activate(ros_exec)
    output_change_client.activate(ros_exec)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert minimal_cartesian_input.received_future.done() and minimal_cartesian_input.received_future.result()
    assert random_pose.get_name() == minimal_cartesian_input.input.get_name()
    assert random_pose.dist(minimal_cartesian_input.input) < 1e-3
    with pytest.raises(ModuloError):
        minimal_cartesian_output.publish()


@pytest.mark.parametrize("minimal_cartesian_input", [[LifecycleComponent, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[LifecycleComponent, "/topic", False]], indirect=True)
def test_input_output_manual(ros_exec, make_lifecycle_change_client, random_pose, minimal_cartesian_output,
                             minimal_cartesian_input):
    input_change_client = make_lifecycle_change_client("minimal_cartesian_input")
    output_change_client = make_lifecycle_change_client("minimal_cartesian_output")
    ros_exec.add_node(input_change_client)
    ros_exec.add_node(output_change_client)
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_cartesian_output)
    input_change_client.configure(ros_exec)
    output_change_client.configure(ros_exec)
    input_change_client.activate(ros_exec)
    output_change_client.activate(ros_exec)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.done()
    minimal_cartesian_output.publish()
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert minimal_cartesian_input.received_future.done() and minimal_cartesian_input.received_future.result()
    assert random_pose.get_name() == minimal_cartesian_input.input.get_name()
    assert random_pose.dist(minimal_cartesian_input.input) < 1e-3


@pytest.mark.parametrize("minimal_cartesian_input", [[LifecycleComponent, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_joint_output", [[LifecycleComponent, "/topic", True]], indirect=True)
def test_input_output_invalid_type(ros_exec, minimal_joint_output, minimal_cartesian_input):
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(minimal_joint_output)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.done()


@pytest.mark.parametrize("minimal_cartesian_input", [[LifecycleComponent, "/topic"]], indirect=True)
def test_input_output_invalid_msg(ros_exec, make_lifecycle_change_client, make_minimal_invalid_encoded_state_publisher,
                              minimal_cartesian_input):
    input_change_client = make_lifecycle_change_client("minimal_cartesian_input")
    invalid_publisher = make_minimal_invalid_encoded_state_publisher("/topic")
    ros_exec.add_node(input_change_client)
    ros_exec.add_node(minimal_cartesian_input)
    ros_exec.add_node(invalid_publisher)
    input_change_client.configure(ros_exec)
    input_change_client.activate(ros_exec)
    ros_exec.spin_until_future_complete(minimal_cartesian_input.received_future, timeout_sec=0.5)
    assert not minimal_cartesian_input.received_future.result()


def test_trigger(ros_exec, make_lifecycle_change_client, make_predicates_listener):
    trigger = Trigger()
    listener = make_predicates_listener("/trigger", ["test"])
    client = make_lifecycle_change_client("trigger")
    ros_exec.add_node(trigger)
    ros_exec.add_node(listener)
    ros_exec.add_node(client)
    client.configure(ros_exec)
    ros_exec.spin_until_future_complete(listener.predicates_future, timeout_sec=0.5)
    assert not listener.predicates_future.done()
    assert not listener.predicate_values["test"]
    client.activate(ros_exec)
    ros_exec.spin_until_future_complete(listener.predicates_future, timeout_sec=0.5)
    assert listener.predicates_future.done()
    assert listener.predicate_values["test"]
