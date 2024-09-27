from typing import Optional

import clproto
import numpy as np
import state_representation as sr
from modulo_core.exceptions import ParameterTranslationError
from rclpy import Parameter


def write_parameter(parameter: sr.Parameter) -> Parameter:
    """
    Write a ROS Parameter from a state_representation Parameter.

    :param parameter: The state_representation parameter to read from
    :raises ParameterTranslationError if the parameter could not be written
    :return: The resulting ROS parameter
    """
    if not parameter:
        return Parameter(
            parameter.get_name(),
            value=None,
            type_=Parameter.Type.NOT_SET)
    elif parameter.get_parameter_type() == sr.ParameterType.BOOL or \
            parameter.get_parameter_type() == sr.ParameterType.INT or \
            parameter.get_parameter_type() == sr.ParameterType.DOUBLE or \
            parameter.get_parameter_type() == sr.ParameterType.STRING:
        return Parameter(parameter.get_name(), value=parameter.get_value())
    elif parameter.get_parameter_type() == sr.ParameterType.BOOL_ARRAY:
        return Parameter(
            parameter.get_name(),
            value=parameter.get_value(),
            type_=Parameter.Type.BOOL_ARRAY)
    elif parameter.get_parameter_type() == sr.ParameterType.INT_ARRAY:
        return Parameter(
            parameter.get_name(),
            value=parameter.get_value(),
            type_=Parameter.Type.INTEGER_ARRAY)
    elif parameter.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY:
        return Parameter(
            parameter.get_name(),
            value=parameter.get_value(),
            type_=Parameter.Type.DOUBLE_ARRAY)
    elif parameter.get_parameter_type() == sr.ParameterType.STRING_ARRAY:
        return Parameter(
            parameter.get_name(),
            value=parameter.get_value(),
            type_=Parameter.Type.STRING_ARRAY)
    elif parameter.get_parameter_type() == sr.ParameterType.STATE:
        if parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_STATE:
            return Parameter(
                parameter.get_name(),
                Parameter.Type.STRING,
                clproto.to_json(
                    clproto.encode(
                        parameter.get_value(),
                        clproto.MessageType.CARTESIAN_STATE_MESSAGE)))
        elif parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_POSE:
            return Parameter(
                parameter.get_name(),
                Parameter.Type.STRING,
                clproto.to_json(
                    clproto.encode(
                        parameter.get_value(),
                        clproto.MessageType.CARTESIAN_POSE_MESSAGE)))
        elif parameter.get_parameter_state_type() == sr.StateType.JOINT_STATE:
            return Parameter(
                parameter.get_name(),
                Parameter.Type.STRING,
                clproto.to_json(
                    clproto.encode(
                        parameter.get_value(),
                        clproto.MessageType.JOINT_STATE_MESSAGE)))
        if parameter.get_parameter_state_type() == sr.StateType.JOINT_POSITIONS:
            return Parameter(
                parameter.get_name(),
                Parameter.Type.STRING,
                clproto.to_json(
                    clproto.encode(
                        parameter.get_value(),
                        clproto.MessageType.JOINT_POSITIONS_MESSAGE)))
        else:
            raise ParameterTranslationError(
                f"Parameter {parameter.get_name()} could not be written!")
    elif parameter.get_parameter_type() == sr.ParameterType.VECTOR or \
            parameter.get_parameter_type() == sr.ParameterType.MATRIX:
        return Parameter(
            parameter.get_name(),
            value=parameter.get_value().flatten().tolist(),
            type_=Parameter.Type.DOUBLE_ARRAY)
    else:
        raise ParameterTranslationError(
            f"Parameter {parameter.get_name()} could not be written!")


def copy_parameter_value(
        source_parameter: sr.Parameter,
        parameter: sr.Parameter):
    """
    Copy the value of one state_representation Parameter into another. This helper function calls the getters and
    setters of the appropriate parameter type to modify the value of the parameter instance while preserving the
    reference to the original parameter.

    :param source_parameter: The parameter with a value to copy
    :param parameter: The destination parameter to be updated
    :raises ParameterTranslationError if the copying failed
    """
    if source_parameter.get_parameter_type() != parameter.get_parameter_type():
        raise ParameterTranslationError(
            f"Source parameter {source_parameter.get_name()} to be copied does not have the same type "
            f"as destination parameter {parameter.get_name()}")
    if source_parameter.get_parameter_type() == sr.ParameterType.BOOL or \
            source_parameter.get_parameter_type() == sr.ParameterType.BOOL_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.INT or \
            source_parameter.get_parameter_type() == sr.ParameterType.INT_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.DOUBLE or \
            source_parameter.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.STRING or \
            source_parameter.get_parameter_type() == sr.ParameterType.STRING_ARRAY or \
            source_parameter.get_parameter_type() == sr.ParameterType.VECTOR or \
            source_parameter.get_parameter_type() == sr.ParameterType.MATRIX:
        parameter.set_value(source_parameter.get_value())
    elif source_parameter.get_parameter_type() == sr.ParameterType.STATE:
        if source_parameter.get_parameter_state_type(
        ) != parameter.get_parameter_state_type():
            raise ParameterTranslationError(
                f"Source parameter {source_parameter.get_name()} to be copied does not have the same state type "
                f"as destination parameter {parameter.get_name()}")
        if source_parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_STATE or \
                source_parameter.get_parameter_state_type() == sr.StateType.CARTESIAN_POSE or \
                source_parameter.get_parameter_state_type() == sr.StateType.JOINT_STATE or \
                source_parameter.get_parameter_state_type() == sr.StateType.JOINT_POSITIONS:
            parameter.set_value(source_parameter.get_value())
        else:
            raise ParameterTranslationError(
                f"Could not copy the value from source parameter " f"{
                    source_parameter.get_name()} into parameter {
                    parameter.get_name()}")
    else:
        raise ParameterTranslationError(
            f"Could not copy the value from source parameter {
                source_parameter.get_name()} " f"into parameter {
                parameter.get_name()}")


def read_parameter(ros_parameter: Parameter,
                   parameter: Optional[sr.Parameter] = None) -> sr.Parameter:
    """
    Update the parameter value of a state_representation Parameter from a ROS Parameter object.

    :param ros_parameter: The ROS Parameter to read from
    :param parameter: The state_representation Parameter to populate
    :raises ParameterTranslationError if the ROS Parameter could not be read
    :return: The resulting state_representation Parameter
    """

    def read_new_parameter(ros_param: Parameter) -> sr.Parameter:
        """
        Read a ROS Parameter object and translate to a state_representation Parameter object.

        :param ros_param: The ROS Parameter to read from
        :raises ParameterTranslationError if the ROS Parameter could not be read
        :return: The resulting state_representation Parameter
        """
        if ros_param.type_ == Parameter.Type.BOOL:
            return sr.Parameter(
                ros_param.name,
                ros_param.get_parameter_value().bool_value,
                sr.ParameterType.BOOL)
        elif ros_param.type_ == Parameter.Type.BOOL_ARRAY:
            return sr.Parameter(
                ros_param.name,
                ros_param.get_parameter_value().bool_array_value,
                sr.ParameterType.BOOL_ARRAY)
        elif ros_param.type_ == Parameter.Type.INTEGER:
            return sr.Parameter(
                ros_param.name,
                ros_param.get_parameter_value().integer_value,
                sr.ParameterType.INT)
        elif ros_param.type_ == Parameter.Type.INTEGER_ARRAY:
            return sr.Parameter(
                ros_param.name,
                ros_param.get_parameter_value().integer_array_value,
                sr.ParameterType.INT_ARRAY)
        elif ros_param.type_ == Parameter.Type.DOUBLE:
            return sr.Parameter(
                ros_param.name,
                ros_param.get_parameter_value().double_value,
                sr.ParameterType.DOUBLE)
        elif ros_param.type_ == Parameter.Type.DOUBLE_ARRAY:
            return sr.Parameter(
                ros_param.name,
                ros_param.get_parameter_value().double_array_value,
                sr.ParameterType.DOUBLE_ARRAY)
        elif ros_param.type_ == Parameter.Type.STRING_ARRAY:
            return sr.Parameter(
                ros_param.name,
                ros_param.get_parameter_value().string_array_value,
                sr.ParameterType.STRING_ARRAY)
        elif ros_param.type_ == Parameter.Type.STRING:
            encoding = ""
            try:
                encoding = clproto.from_json(
                    ros_param.get_parameter_value().string_value)
            except Exception:
                if not clproto.is_valid(encoding):
                    return sr.Parameter(
                        ros_param.name,
                        ros_param.get_parameter_value().string_value,
                        sr.ParameterType.STRING)
            message_type = clproto.check_message_type(encoding)
            if message_type == clproto.MessageType.CARTESIAN_STATE_MESSAGE:
                return sr.Parameter(
                    ros_param.name,
                    clproto.decode(encoding),
                    sr.ParameterType.STATE,
                    sr.StateType.CARTESIAN_STATE)
            elif message_type == clproto.MessageType.CARTESIAN_POSE_MESSAGE:
                return sr.Parameter(
                    ros_param.name,
                    clproto.decode(encoding),
                    sr.ParameterType.STATE,
                    sr.StateType.CARTESIAN_POSE)
            elif message_type == clproto.MessageType.JOINT_STATE_MESSAGE:
                return sr.Parameter(
                    ros_param.name,
                    clproto.decode(encoding),
                    sr.ParameterType.STATE,
                    sr.StateType.JOINT_STATE)
            elif message_type == clproto.MessageType.JOINT_POSITIONS_MESSAGE:
                return sr.Parameter(
                    ros_param.name,
                    clproto.decode(encoding),
                    sr.ParameterType.STATE,
                    sr.StateType.JOINT_POSITIONS)
            else:
                raise ParameterTranslationError(
                    f"Parameter {
                        ros_param.name} has an unsupported encoded message type!")
        elif ros_param.type_ == Parameter.Type.BYTE_ARRAY:
            raise ParameterTranslationError(
                "Parameter byte arrays are not currently supported.")
        else:
            raise ParameterTranslationError(
                f"Parameter {ros_param.name} could not be read!")

    new_parameter = read_new_parameter(ros_parameter)
    if parameter is None:
        return new_parameter
    else:
        copy_parameter_value(new_parameter, parameter)
        return parameter


def read_parameter_const(
        ros_parameter: Parameter,
        parameter: sr.Parameter) -> sr.Parameter:
    """
    Update the parameter value of a state_representation Parameter from a ROS Parameter object only if the value of
    the ROS Parameter can be interpreted as the value of the original state_representation Parameter.

    :param ros_parameter: The ROS Parameter to read from
    :param parameter: The state_representation Parameter to update
    :raises ParameterTranslationError if the ROS Parameter could not be read
    :return: The resulting state_representation Parameter
    """
    if ros_parameter.name != parameter.get_name():
        raise ParameterTranslationError(
            f"The ROS parameter {
                ros_parameter.name} to be read does not have " f"the same name as the reference parameter {
                parameter.get_name()}")
    if ros_parameter.type_ == Parameter.Type.NOT_SET:
        return sr.Parameter(
            parameter.get_name(),
            parameter.get_parameter_type(),
            parameter.get_parameter_state_type())
    new_parameter = read_parameter(ros_parameter)
    if new_parameter.get_parameter_type() == parameter.get_parameter_type():
        if new_parameter.get_parameter_state_type() != parameter.get_parameter_state_type():
            raise ParameterTranslationError(
                f"The received state parameter {
                    ros_parameter.name} does not have " f"the same state type as the reference parameter")
        return new_parameter
    elif new_parameter.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY:
        value = new_parameter.get_value()
        if parameter.get_parameter_type() == sr.ParameterType.VECTOR:
            vector = np.array(value)
            return sr.Parameter(
                parameter.get_name(),
                vector,
                sr.ParameterType.VECTOR)
        elif parameter.get_parameter_type() == sr.ParameterType.MATRIX:
            matrix = parameter.get_value()
            if matrix.size != len(value):
                raise ParameterTranslationError(
                    f"The ROS parameter {
                        ros_parameter.name} with type double array has size " f"{
                        len(value)} while the reference parameter matrix {
                        parameter.get_name()} " f"has size {
                        len(matrix)}")
            matrix = np.array(value).reshape(matrix.shape)
            return sr.Parameter(
                parameter.get_name(),
                matrix,
                sr.ParameterType.MATRIX)
        else:
            raise ParameterTranslationError(
                f"The ROS parameter {
                    ros_parameter.name} with type double array cannot be interpreted " f"by reference parameter {
                    parameter.get_name()} (type code {
                    parameter.get_parameter_type()}")
    else:
        raise ParameterTranslationError(
            f"The ROS parameter has an incompatible type for component parameter '{
                parameter.get_name()}': " f"expected {
                parameter.get_parameter_type().name}, got {
                ros_parameter.type_.name}")


def get_ros_parameter_type(parameter_type: sr.ParameterType) -> Parameter.Type:
    """
    Given a state representation parameter type, get the corresponding ROS parameter type.

    :param parameter_type: The state representation parameter type
    :return: The corresponding ROS parameter type
    """
    if parameter_type == sr.ParameterType.BOOL:
        return Parameter.Type.BOOL
    elif parameter_type == sr.ParameterType.BOOL_ARRAY:
        return Parameter.Type.BOOL_ARRAY
    elif parameter_type == sr.ParameterType.INT:
        return Parameter.Type.INTEGER
    elif parameter_type == sr.ParameterType.INT_ARRAY:
        return Parameter.Type.INTEGER_ARRAY
    elif parameter_type == sr.ParameterType.DOUBLE:
        return Parameter.Type.DOUBLE
    elif parameter_type in [sr.ParameterType.DOUBLE_ARRAY, sr.ParameterType.VECTOR, sr.ParameterType.MATRIX]:
        return Parameter.Type.DOUBLE_ARRAY
    elif parameter_type in [sr.ParameterType.STRING, sr.ParameterType.STATE]:
        return Parameter.Type.STRING
    elif parameter_type == sr.ParameterType.STRING_ARRAY:
        return Parameter.Type.STRING_ARRAY
    else:
        return Parameter.Type.NOT_SET
