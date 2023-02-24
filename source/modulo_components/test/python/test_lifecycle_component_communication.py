import pytest
from modulo_components.lifecycle_component import LifecycleComponent


@pytest.mark.parametrize("minimal_cartesian_input", [[LifecycleComponent, "/topic"]], indirect=True)
@pytest.mark.parametrize("minimal_cartesian_output", [[LifecycleComponent, "/topic"]], indirect=True)
def test_input_output(ros_exec, make_lifecycle_change_client, random_state, minimal_cartesian_output,
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
    assert minimal_cartesian_input.received_future.result()
    assert random_state.get_name() == minimal_cartesian_input.input.get_name()
    assert random_state.dist(minimal_cartesian_input.input) < 1e-3


@pytest.mark.parametrize("minimal_cartesian_input", [[LifecycleComponent, "/topic"]], indirect=True)
def test_input_output_invalid(ros_exec, make_lifecycle_change_client, make_minimal_invalid_encoded_state_publisher,
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
