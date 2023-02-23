import pytest

from modulo_component_interfaces.srv import EmptyTrigger


def test_ros_context(ros_context):
    assert True


def test_ros_exec(ros_exec):
    assert True


def test_lifecycle_change_client(ros_context, make_lifecycle_change_client):
    with pytest.raises(RuntimeError):
        make_lifecycle_change_client("/test")


def test_predicates_listener(ros_exec, make_predicates_listener):
    listener = make_predicates_listener("test", ["in_error_state"])
    ros_exec.spin_until_future_complete(listener.predicates_future, 0.1)
    assert not listener.predicates_future.done()

def test_service_client(ros_context, make_service_client):
    with pytest.raises(RuntimeError):
        make_service_client({"/test": EmptyTrigger})
