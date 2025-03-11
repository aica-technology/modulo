from threading import Thread
from typing import List, Optional, Union

from modulo_core.exceptions import LookupJointPositionsException
from modulo_interfaces.msg import JointPositions, JointPositionsCollection
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile


class JointPositionsListener:
    """
    The JointPositionsBroadcaster is a TF2 style class that listens to the fixed /joint_positions topic and allows to
    lookup messages from a buffer.
    """

    def __init__(
            self, node: Node, spin_thread: bool = False, qos: Optional[Union[QoSProfile, int]] = None):
        """
        Constructor of the JointPositionsListener.

        :param node: The ROS2 node
        :param spin_thread: Whether to create a dedicated thread to spin this node
        :param qos: A QoSProfile or a history depth to apply to the subscriber
        """
        if qos is None:
            qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._node = node
        # Default callback group is mutually exclusive, which would prevent waiting for joint positions
        # from another callback in the same group.
        self._group = ReentrantCallbackGroup()
        self._buffer = []

        self._subscription = node.create_subscription(
            JointPositionsCollection, "/joint_positions", self._callback, qos, callback_group=self._group)

        if spin_thread:
            self._executor = SingleThreadedExecutor()

            def run_func():
                self._executor.add_node(self._node)
                self._executor.spin()
                self._executor.remove_node(self._node)

            self._dedicated_listener_thread = Thread(target=run_func)
            self._dedicated_listener_thread.start()

    def __del__(self):
        if hasattr(self, 'dedicated_listener_thread') and hasattr(self, 'executor'):
            self._executor.shutdown()
            self._dedicated_listener_thread.join()
        self._node.destroy_subscription(self._subscription)

    def _callback(self, message: JointPositionsCollection):
        for msg in message.joint_positions:
            self._buffer.append(msg)

    def lookup(self, name: str) -> JointPositions:
        """
        Look up JointPositions object by it's name.

        :param name: The name of the JointPositions object to lookup
        """
        for joint_positions in self._buffer:
            if joint_positions.header.frame_id == name:
                return joint_positions
        raise LookupJointPositionsException(f"Joint positions {name} is not available")

    @property
    def buffer(self) -> List[JointPositions]:
        """
        Get a vector of all buffered JointPositions objects.
        """
        return self._buffer
