import clproto
import geometry_msgs.msg
import modulo_core.translators.message_readers as modulo_readers
import modulo_core.translators.message_writers as modulo_writers
import numpy as np
import pytest
import state_representation as sr
from modulo_core import EncodedState
from modulo_core.exceptions import MessageTranslationError
from rclpy.clock import Clock
from sensor_msgs.msg import JointState


def read_xyz(message):
    return [message.x, message.y, message.z]


def read_quaternion(message):
    return [message.w, message.x, message.y, message.z]


def assert_np_array_equal(a: np.array, b: np.array, places=3):
    try:
        np.testing.assert_almost_equal(a, b, decimal=places)
    except AssertionError as e:
        pytest.fail(f'{e}')


def test_accel(cart_state: sr.CartesianState, clock: Clock):
    message = geometry_msgs.msg.Accel()
    modulo_writers.write_message(message, cart_state)
    assert_np_array_equal(read_xyz(message.linear), cart_state.get_linear_acceleration())
    assert_np_array_equal(read_xyz(message.angular), cart_state.get_angular_acceleration())

    new_state = sr.CartesianState("new")
    with pytest.raises(MessageTranslationError):
        modulo_writers.write_message(new_state, message)
    modulo_readers.read_message(new_state, message)
    assert_np_array_equal(cart_state.get_acceleration(), new_state.get_acceleration())

    message_stamped = geometry_msgs.msg.AccelStamped()
    modulo_writers.write_stamped_message(message_stamped, cart_state, clock.now())
    assert message_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(message.linear), cart_state.get_linear_acceleration())
    assert_np_array_equal(read_xyz(message.angular), cart_state.get_angular_acceleration())
    new_state = sr.CartesianState("new")
    modulo_readers.read_stamped_message(new_state, message_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_acceleration(), new_state.get_acceleration())


def test_pose(cart_state: sr.CartesianState, clock: Clock):
    message = geometry_msgs.msg.Pose()
    modulo_writers.write_message(message, cart_state)
    assert_np_array_equal(read_xyz(message.position), cart_state.get_position())
    assert_np_array_equal(read_quaternion(message.orientation), cart_state.get_orientation_coefficients())

    new_state = sr.CartesianState("new")
    with pytest.raises(MessageTranslationError):
        modulo_writers.write_message(new_state, message)
    modulo_readers.read_message(new_state, message)
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())

    message_stamped = geometry_msgs.msg.PoseStamped()
    modulo_writers.write_stamped_message(message_stamped, cart_state, clock.now())
    assert message_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(message.position), cart_state.get_position())
    assert_np_array_equal(read_quaternion(message.orientation), cart_state.get_orientation_coefficients())
    new_state = sr.CartesianState("new")
    modulo_readers.read_stamped_message(new_state, message_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())


def test_transform(cart_state: sr.CartesianState, clock: Clock):
    message = geometry_msgs.msg.Transform()
    modulo_writers.write_message(message, cart_state)
    assert_np_array_equal(read_xyz(message.translation), cart_state.get_position())
    assert_np_array_equal(read_quaternion(message.rotation), cart_state.get_orientation_coefficients())

    new_state = sr.CartesianState("new")
    with pytest.raises(MessageTranslationError):
        modulo_writers.write_message(new_state, message)
    modulo_readers.read_message(new_state, message)
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())

    message_stamped = geometry_msgs.msg.TransformStamped()
    modulo_writers.write_stamped_message(message_stamped, cart_state, clock.now())
    assert message_stamped.header.frame_id == cart_state.get_reference_frame()
    assert message_stamped.child_frame_id == cart_state.get_name()
    assert_np_array_equal(read_xyz(message.translation), cart_state.get_position())
    assert_np_array_equal(read_quaternion(message.rotation), cart_state.get_orientation_coefficients())
    new_state = sr.CartesianState("new")
    modulo_readers.read_stamped_message(new_state, message_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert cart_state.get_name() == new_state.get_name()
    assert_np_array_equal(cart_state.get_pose(), new_state.get_pose())


def test_twist(cart_state: sr.CartesianState, clock: Clock):
    message = geometry_msgs.msg.Twist()
    modulo_writers.write_message(message, cart_state)
    assert_np_array_equal(read_xyz(message.linear), cart_state.get_linear_velocity())
    assert_np_array_equal(read_xyz(message.angular), cart_state.get_angular_velocity())

    new_state = sr.CartesianState("new")
    with pytest.raises(MessageTranslationError):
        modulo_writers.write_message(new_state, message)
    modulo_readers.read_message(new_state, message)
    assert_np_array_equal(cart_state.get_twist(), new_state.get_twist())

    message_stamped = geometry_msgs.msg.TwistStamped()
    modulo_writers.write_stamped_message(message_stamped, cart_state, clock.now())
    assert message_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(message.linear), cart_state.get_linear_velocity())
    assert_np_array_equal(read_xyz(message.angular), cart_state.get_angular_velocity())
    new_state = sr.CartesianState("new")
    modulo_readers.read_stamped_message(new_state, message_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_twist(), new_state.get_twist())


def test_wrench(cart_state: sr.CartesianState, clock: Clock):
    message = geometry_msgs.msg.Wrench()
    modulo_writers.write_message(message, cart_state)
    assert_np_array_equal(read_xyz(message.force), cart_state.get_force())
    assert_np_array_equal(read_xyz(message.torque), cart_state.get_torque())

    new_state = sr.CartesianState("new")
    with pytest.raises(MessageTranslationError):
        modulo_writers.write_message(new_state, message)
    modulo_readers.read_message(new_state, message)
    assert_np_array_equal(cart_state.get_wrench(), new_state.get_wrench())

    message_stamped = geometry_msgs.msg.WrenchStamped()
    modulo_writers.write_stamped_message(message_stamped, cart_state, clock.now())
    assert message_stamped.header.frame_id == cart_state.get_reference_frame()
    assert_np_array_equal(read_xyz(message.force), cart_state.get_force())
    assert_np_array_equal(read_xyz(message.torque), cart_state.get_torque())
    new_state = sr.CartesianState("new")
    modulo_readers.read_stamped_message(new_state, message_stamped)
    assert cart_state.get_reference_frame() == new_state.get_reference_frame()
    assert_np_array_equal(cart_state.get_wrench(), new_state.get_wrench())


def test_joint_state(joint_state: sr.JointState):
    message = JointState()
    modulo_writers.write_message(message, joint_state)
    for i in range(joint_state.get_size()):
        assert message.name[i] == joint_state.get_names()[i]
    assert_np_array_equal(message.position, joint_state.get_positions())
    assert_np_array_equal(message.velocity, joint_state.get_velocities())
    assert_np_array_equal(message.effort, joint_state.get_torques())

    new_state = sr.JointState("test", ["1", "2", "3"])
    with pytest.raises(MessageTranslationError):
        modulo_writers.write_message(new_state, message)
    modulo_readers.read_message(new_state, message)
    for i in range(joint_state.get_size()):
        assert new_state.get_names()[i] == joint_state.get_names()[i]
    assert_np_array_equal(new_state.get_positions(), joint_state.get_positions())
    assert_np_array_equal(new_state.get_velocities(), joint_state.get_velocities())
    assert_np_array_equal(new_state.get_torques(), joint_state.get_torques())


def test_encoded_state(cart_state: sr.CartesianState):
    message = EncodedState()
    modulo_writers.write_clproto_message(message, cart_state, clproto.MessageType.CARTESIAN_STATE_MESSAGE)

    new_state = modulo_readers.read_clproto_message(message)
    assert_np_array_equal(new_state.data(), cart_state.data())
    assert new_state.get_name() == cart_state.get_name()
    assert new_state.get_reference_frame() == cart_state.get_reference_frame()
