from typing import TypeVar, Union

import clproto
import geometry_msgs.msg as geometry
import numpy as np
import rclpy.time
import state_representation as sr
from modulo_core import EncodedState
from modulo_core.exceptions import MessageTranslationError
from sensor_msgs.msg import JointState

DataT = TypeVar('DataT')
MsgT = TypeVar('MsgT')
StateT = TypeVar('StateT')


def write_xyz(message: Union[geometry.Point, geometry.Vector3], vector: np.array):
    """
    Helper function to write a vector to a Point or Vector3 message.

    :param message: The message to populate
    :param vector: The vector to read from
    :raises MessageTranslationError if the message could not be written
    """
    if vector.size != 3:
        raise MessageTranslationError("Provide a vector of size 3 to transform to a Point or Vector3 message.")
    message.x = vector[0]
    message.y = vector[1]
    message.z = vector[2]


def write_quaternion(message: geometry.Quaternion, quat: np.array):
    """
    Helper function to write a vector of quaternion coefficients (w,x,y,z) to a Quaternion message.

    :param message: The message to populate
    :param quat: The vector to read from
    :raises MessageTranslationError if the message could not be written
    """
    if quat.size != 4:
        raise MessageTranslationError("Provide a vector of size 4 to transform to a Quaternion message.")
    message.w = quat[0]
    message.x = quat[1]
    message.y = quat[2]
    message.z = quat[3]


def write_message(message: MsgT, state: StateT):
    """
    Convert a state_representation State type to a ROS message.

    :param message: The ROS message to populate
    :param state: The state to read from
    :raises MessageTranslationError if the message could not be written
    """
    try:
        if not isinstance(state, sr.State):
            raise MessageTranslationError("This state type is not supported.")
        if not state:
            raise MessageTranslationError(f"{state.get_name()} state is empty while attempting to write it to message.")
        if isinstance(state, sr.CartesianState):
            if isinstance(message, geometry.Accel):
                write_xyz(message.linear, state.get_linear_acceleration())
                write_xyz(message.angular, state.get_angular_acceleration())
            elif isinstance(message, geometry.Pose):
                write_xyz(message.position, state.get_position())
                write_quaternion(message.orientation, state.get_orientation_coefficients())
            elif isinstance(message, geometry.Transform):
                write_xyz(message.translation, state.get_position())
                write_quaternion(message.rotation, state.get_orientation_coefficients())
            elif isinstance(message, geometry.Twist):
                write_xyz(message.linear, state.get_linear_velocity())
                write_xyz(message.angular, state.get_angular_velocity())
            elif isinstance(message, geometry.Wrench):
                write_xyz(message.force, state.get_force())
                write_xyz(message.torque, state.get_torque())
            else:
                raise MessageTranslationError("The provided combination of state type and message type is not supported")
        elif isinstance(message, JointState) and isinstance(state, sr.JointState):
            message.name = state.get_names()
            message.position = state.get_positions().tolist()
            message.velocity = state.get_velocities().tolist()
            message.effort = state.get_torques().tolist()
        else:
            raise MessageTranslationError("The provided combination of state type and message type is not supported")
    except MessageTranslationError:
        raise
    except Exception as e:
        raise MessageTranslationError(f"{e}")


def write_stamped_message(message: MsgT, state: StateT, time: rclpy.time.Time):
    """
    Convert a state_representation State type to a stamped ROS message.

    :param message: The ROS message to populate
    :param state: The state to read from
    :param time: The time of the message
    :raises MessageTranslationError if the message could not be written
    """
    try:
        if isinstance(message, geometry.AccelStamped):
            write_message(message.accel, state)
        elif isinstance(message, geometry.PoseStamped):
            write_message(message.pose, state)
        elif isinstance(message, geometry.TransformStamped):
            write_message(message.transform, state)
            message.child_frame_id = state.get_name()
        elif isinstance(message, geometry.TwistStamped):
            write_message(message.twist, state)
        elif isinstance(message, geometry.WrenchStamped):
            write_message(message.wrench, state)
        else:
            raise MessageTranslationError("The provided combination of state type and message type is not supported")
        message.header.stamp = time.to_msg()
        message.header.frame_id = state.get_reference_frame()
    except MessageTranslationError:
        raise
    except Exception as e:
        raise MessageTranslationError(f"{e}")


def write_std_message(message: MsgT, data: DataT):
    """
    Convert to a std_msgs.msg message.

    :param message: The std_msgs.msg message to populate
    :param data: The data to read from
    """
    message.data = data


def write_clproto_message(message: EncodedState, state: StateT, clproto_message_type: clproto.MessageType):
    """
    Convert a state_representation State to an EncodedState message.

    :param message: The EncodedState message to populate
    :param state: The state to read from
    :param clproto_message_type: The clproto message type to encode the state
    :raises MessageTranslationError if the message could not be written
    """
    if not isinstance(state, sr.State):
        raise MessageTranslationError("This state type is not supported.")
    try:
        message.data = clproto.encode(state, clproto_message_type)
    except MessageTranslationError:
        raise
    except Exception as e:
        raise MessageTranslationError(f"{e}")
