import rclpy
from modulo_core import JointPositionsBroadcaster, JointPositionsListener
from modulo_interfaces.msg import JointPositions


def test_send_lookup(ros_context):
    node = rclpy.create_node("test_joint_positions")

    listener = JointPositionsListener(node)
    broadcaster = JointPositionsBroadcaster(node)

    joint_pos = JointPositions()
    joint_pos.header.frame_id = "foo"
    joint_pos.joint_names = ["one", "two"]
    joint_pos.positions = [1.0, 2.0]
    broadcaster.send(joint_pos)

    rclpy.spin_once(node)
    
    joint_positions = listener.lookup("foo")
    assert joint_positions.header.frame_id == joint_pos.header.frame_id
    assert joint_positions.joint_names == joint_pos.joint_names
    assert joint_positions.positions == joint_pos.positions
