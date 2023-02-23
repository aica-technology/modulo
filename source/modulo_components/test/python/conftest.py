import clproto
import pytest
import state_representation as sr
from modulo_components.component import Component
from modulo_core import EncodedState
from rclpy.task import Future

pytest_plugins = ["modulo_utils.testutils.ros", "modulo_utils.testutils.lifecycle_change_client"]


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
