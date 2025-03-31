from typing import List, Optional, Union

from modulo_interfaces.msg import JointPositions, JointPositionsCollection
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile


class JointPositionsBroadcaster:
    """
    The JointPositionsBroadcaster is a TF2 style class that publishes a collection of JointPositions messages to 
    the fixed /joint_positions topic.
    """

    def __init__(self, node: Node, qos: Optional[Union[QoSProfile, int]] = None):
        """
        Constructor of the JointPositionsBroadcaster.

        :param node: The ROS2 node
        :param qos: A QoSProfile or a history depth to apply to the subscriber
        """
        if qos is None:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self._publisher = node.create_publisher(JointPositionsCollection, '/joint_positions', qos)
        self._net_message = JointPositionsCollection()
        self._names = set()
        self._logger = node.get_logger()

    def send(self, joint_positions: Union[JointPositions, List[JointPositions]]):
        """
        Send a JointPositions object or a list of JointPositions.

        :param joint_positions: A JointPositions object or list of JointPositions to send.
        """
        def update_net_message(jp_in: JointPositions):
            if not isinstance(jp_in, JointPositions):
                self._logger.warn("Sending joint positions failed, incorrect argument type")
                return
            if jp_in.header.frame_id not in self._names:
                self._names.add(jp_in.header.frame_id)
                self._net_message.joint_positions.append(jp_in)
    
        if hasattr(joint_positions, '__iter__'):
            [update_net_message(jp) for jp in joint_positions]
        else:
            update_net_message(joint_positions)
    
        self._publisher.publish(self._net_message)
