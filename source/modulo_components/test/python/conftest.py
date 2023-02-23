import clproto
import pytest
import state_representation as sr
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from modulo_components.component import Component
from modulo_core import EncodedState
from rclpy.node import Node
from rclpy.task import Future

pytest_plugins = ["modulo_utils.testutils.ros"]


@pytest.fixture
def random_state():
    return sr.CartesianPose.Random("test")


@pytest.fixture
def minimal_cartesian_output(request, random_state):
    def _make_minimal_cartesian_output(component_type, topic):
        component = component_type("minimal_cartesian_output")
        component._output = random_state
        component.add_output("cartesian_state", "_output", EncodedState, clproto.MessageType.CARTESIAN_STATE_MESSAGE,
                             topic)
        return component

    yield _make_minimal_cartesian_output(request.param[0], request.param[1])


class MinimalInvalidEncodedStatePublisher(Component):
    def __init__(self, topic, *args, **kwargs):
        super().__init__("minimal_invalid_encoded_state_publisher", *args, **kwargs)
        self.publisher = self.create_publisher(EncodedState, topic, self.get_qos())
        self.add_periodic_callback("publish", self.__publish)

    def __publish(self):
        msg = EncodedState()
        data = "hello"
        msg.data = bytes(data.encode())
        self.publisher.publish(msg)


@pytest.fixture
def make_minimal_invalid_encoded_state_publisher():
    def _make_minimal_invalid_encoded_state_publisher(topic):
        return MinimalInvalidEncodedStatePublisher(topic)

    yield _make_minimal_invalid_encoded_state_publisher


@pytest.fixture
def minimal_cartesian_input(request):
    def _make_minimal_cartesian_input(component_type, topic):
        component = component_type("minimal_cartesian_input")
        component.received_future = Future()
        component.input = sr.CartesianState()
        component.add_input("cartesian_state", "input", EncodedState, topic,
                            user_callback=lambda: component.received_future.set_result(True))
        return component

    yield _make_minimal_cartesian_input(request.param[0], request.param[1])


class LifecycleServiceClient(Node):
    def __init__(self, namespace):
        super().__init__("lifecycle_service_node")
        self.client = self.create_client(ChangeState, namespace + "/change_state")


class Helpers:
    @staticmethod
    def trigger_lifecycle_transition(ros_exec, lifecycle_client, transition_id):
        future = lifecycle_client.client.call_async(ChangeState.Request(transition=Transition(id=transition_id)))
        ros_exec.spin_until_future_complete(future, timeout_sec=0.5)
        assert future.result().success

    @staticmethod
    def configure(ros_exec, lifecycle_client):
        Helpers.trigger_lifecycle_transition(ros_exec, lifecycle_client, 1)

    @staticmethod
    def activate(ros_exec, lifecycle_client):
        Helpers.trigger_lifecycle_transition(ros_exec, lifecycle_client, 3)

    @staticmethod
    def deactivate(ros_exec, lifecycle_client):
        Helpers.trigger_lifecycle_transition(ros_exec, lifecycle_client, 4)


@pytest.fixture
def helpers():
    return Helpers


@pytest.fixture
def make_lifecycle_service_client():
    def _make_lifecycle_service_client(namespace):
        return LifecycleServiceClient(namespace)

    yield _make_lifecycle_service_client
