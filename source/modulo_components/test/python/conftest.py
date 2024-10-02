import clproto
import pytest
import state_representation as sr
from modulo_components.component import Component
from modulo_core import EncodedState
from rclpy.task import Future
from sensor_msgs.msg import JointState

pytest_plugins = ["modulo_utils.testutils.ros", "modulo_utils.testutils.lifecycle_change_client",
                  "modulo_utils.testutils.service_client", "modulo_utils.testutils.predicates_listener"]


@pytest.fixture
def random_pose():
    return sr.CartesianPose().Random("test")


@pytest.fixture
def random_joint():
    return sr.JointState().Random("test", 3)


@pytest.fixture
def random_sensor():
    random = sr.JointState().Random("test", 3)
    msg = JointState()
    msg.name = random.get_names()
    msg.position = random.get_positions().tolist()
    msg.velocity = random.get_velocities().tolist()
    msg.effort = random.get_torques().tolist()
    return msg


@pytest.fixture
def minimal_cartesian_output(request, random_pose):
    def _make_minimal_cartesian_output(component_type, topic, publish_on_step):
        def publish(self):
            self.publish_output("cartesian_pose")

        component = component_type("minimal_cartesian_output")
        component._output = random_pose
        component.add_output("cartesian_pose", "_output", EncodedState, clproto.MessageType.CARTESIAN_STATE_MESSAGE,
                             topic, publish_on_step=publish_on_step)
        component.publish = publish.__get__(component)
        return component

    yield _make_minimal_cartesian_output(request.param[0], request.param[1], request.param[2])


@pytest.fixture
def minimal_joint_output(request, random_joint):
    def _make_minimal_cartesian_output(component_type, topic, publish_on_step):
        def publish(self):
            self.publish_output("joint_state")

        component = component_type("minimal_joint_output")
        component._output = random_joint
        component.add_output("joint_state", "_output", EncodedState, clproto.MessageType.JOINT_STATE_MESSAGE,
                             topic, publish_on_step=publish_on_step)
        component.publish = publish.__get__(component)
        return component

    yield _make_minimal_cartesian_output(request.param[0], request.param[1], request.param[2])


@pytest.fixture
def minimal_sensor_output(request, random_sensor):
    def _make_minimal_sensor_output(component_type, topic, publish_on_step):
        def publish(self):
            self.publish_output("sensor_state")

        component = component_type("minimal_sensor_output")
        component._output = random_sensor
        component.add_output("sensor_state", "_output", JointState, default_topic=topic, publish_on_step=publish_on_step)
        component.publish = publish.__get__(component)
        return component

    yield _make_minimal_sensor_output(request.param[0], request.param[1], request.param[2])


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
        component.add_input("cartesian_pose", "input", EncodedState, topic,
                            user_callback=lambda: component.received_future.set_result(True))
        return component

    yield _make_minimal_cartesian_input(request.param[0], request.param[1])


@pytest.fixture
def minimal_sensor_input(request):
    def _make_minimal_sensor_input(component_type, topic):
        component = component_type("minimal_sensor_input")
        component.received_future = Future()
        component.input = JointState()
        component.add_input("sensor_state", "input", JointState, topic,
                            user_callback=lambda: component.received_future.set_result(True))
        return component

    yield _make_minimal_sensor_input(request.param[0], request.param[1])
