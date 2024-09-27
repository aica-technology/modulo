import pytest
import rclpy.executors
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


class LifecycleChangeClient(Node):
    def __init__(self, namespace: str, timeout_sec=0.5):
        """
        Client node to trigger a lifecycle change of another component

        :param namespace: Namespace of the target component (usually '<component_name>')
        :param timeout_sec: The timeout for waiting for the service
        :raise RuntimeError if the service is not available within the specified timeout
        """
        super().__init__("lifecycle_change_client")
        topic = namespace + "/change_state"
        self._client = self.create_client(ChangeState, topic)

        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"Service 'ChangeState' at {topic} not available")

    def trigger_lifecycle_transition(
            self,
            ros_exec: rclpy.executors.Executor,
            transition_id: int,
            timeout_sec=0.5):
        """
        Trigger the specified transition on the target component

        :param ros_exec: ROS executor object to for spinning
        :param transition_id: The desired transition
        :param timeout_sec: The timeout for completion of the transition
        """
        future = self._client.call_async(
            ChangeState.Request(
                transition=Transition(
                    id=transition_id)))
        ros_exec.spin_until_future_complete(future, timeout_sec=timeout_sec)
        assert future.done() and future.result().success

    def configure(self, ros_exec: rclpy.executors.Executor, timeout_sec=0.5):
        """
        Trigger the 'configure' transition on the target component

        :param ros_exec: ROS executor object to for spinning
        :param timeout_sec: The timeout for completion of the transition
        """
        self.trigger_lifecycle_transition(ros_exec, 1, timeout_sec)

    def activate(self, ros_exec: rclpy.executors.Executor, timeout_sec=0.5):
        """
        Trigger the 'activate' transition on the target component

        :param ros_exec: ROS executor object to for spinning
        :param timeout_sec: The timeout for completion of the transition
        """
        self.trigger_lifecycle_transition(ros_exec, 3, timeout_sec)

    def deactivate(self, ros_exec: rclpy.executors.Executor, timeout_sec=0.5):
        """
        Trigger the 'deactivate' transition on the target component

        :param ros_exec: ROS executor object to for spinning
        :param timeout_sec: The timeout for completion of the transition
        """
        self.trigger_lifecycle_transition(ros_exec, 4, timeout_sec)


@pytest.fixture
def make_lifecycle_change_client():
    """
    Create client node to trigger a lifecycle change of another component. Provide\n
    namespace (str): Namespace of the target component (usually just the target component name)\n
    timeout_sec (float): Optional timeout for waiting for the service

    :raise RuntimeError: if the service is not available within the specified timeout
    """

    def _make_lifecycle_change_client(namespace: str, timeout_sec=0.5):
        return LifecycleChangeClient(namespace, timeout_sec)

    yield _make_lifecycle_change_client
