import clproto
import numpy as np
import pytest
import state_representation as sr
from modulo_core.translators.parameter_translators import write_parameter, read_parameter, read_parameter_const
from modulo_core.exceptions import ParameterTranslationError
from rclpy import Parameter


def assert_np_array_equal(a: np.array, b: np.array, places=3):
    try:
        np.testing.assert_almost_equal(a, b, decimal=places)
    except AssertionError as e:
        pytest.fail(f'{e}')


def test_parameter_write(parameters):
    for name, value_types in parameters.items():
        param = sr.Parameter(name, value_types[1])
        ros_param = write_parameter(param)
        assert ros_param.type_ == Parameter.Type.NOT_SET
        param = sr.Parameter(name, value_types[0], value_types[1])
        ros_param = write_parameter(param)
        assert ros_param.type_ == value_types[3]
        assert ros_param.to_parameter_msg() == Parameter(name, value=value_types[0]).to_parameter_msg()


def test_parameter_read_write(parameters):
    # A ROS parameter with type NOT_SET cannot be interpreted as sr.Parameter
    ros_param = Parameter("name", type_=Parameter.Type.NOT_SET)
    with pytest.raises(ParameterTranslationError):
        read_parameter(ros_param)
    for name, value_types in parameters.items():
        ros_param = Parameter(name, value=value_types[0])
        param = read_parameter(ros_param)
        assert ros_param.name == param.get_name()
        assert param.get_parameter_type() == value_types[1]
        assert param.get_value() == value_types[0]

        new_ros_param = write_parameter(param)
        assert ros_param.to_parameter_msg() == new_ros_param.to_parameter_msg()


def test_parameter_const_read(parameters):
    for name, value_types in parameters.items():
        param = sr.Parameter(name, value_types[0], value_types[1])
        ros_param = Parameter(name, value=value_types[0])

        new_param = read_parameter_const(ros_param, param)
        assert new_param.get_name() == param.get_name()
        assert new_param.get_parameter_type() == param.get_parameter_type()
        assert new_param.get_value() == param.get_value()

        new_param.set_name("other")
        assert new_param.get_name() != param.get_name()


def test_parameter_non_const_read(parameters):
    for name, value_types in parameters.items():
        param = sr.Parameter(name, value_types[2], value_types[1])
        ros_param = Parameter(name, value=value_types[0])

        param_ref = param
        new_param = read_parameter(ros_param, param)
        assert new_param.get_name() == ros_param.name
        assert new_param.get_parameter_type() == value_types[1]
        assert new_param.get_value() == value_types[0]

        assert param_ref.get_name() == ros_param.name
        assert param_ref.get_parameter_type() == value_types[1]
        assert param_ref.get_value() == value_types[0]


def test_state_parameter_write(state_parameters):
    for name, value_types in state_parameters.items():
        param = sr.Parameter(name, sr.ParameterType.STATE, value_types[1])
        ros_param = write_parameter(param)
        assert ros_param.type_ == Parameter.Type.NOT_SET
        param = sr.Parameter(name, value_types[0], sr.ParameterType.STATE, value_types[1])
        ros_param = write_parameter(param)
        assert ros_param.type_ == Parameter.Type.STRING
        assert ros_param.name == param.get_name()
        state = clproto.decode(clproto.from_json(ros_param.value))
        assert state.get_name() == value_types[0].get_name()
        assert_np_array_equal(state.data(), value_types[0].data())


def test_state_parameter_read_write(state_parameters):
    for name, value_types in state_parameters.items():
        json = clproto.to_json(clproto.encode(value_types[0], value_types[2]))
        ros_param = Parameter(name, value=json)
        param = read_parameter(ros_param)
        assert param.get_name() == ros_param.name
        assert param.get_parameter_type() == sr.ParameterType.STATE
        assert param.get_parameter_state_type() == value_types[1]

        assert_np_array_equal(param.get_value().data(), value_types[0].data())
        assert param.get_value().get_name() == value_types[0].get_name()

        new_ros_param = write_parameter(param)
        assert ros_param.name == new_ros_param.name
        assert ros_param.type_ == new_ros_param.type_
        assert_np_array_equal(
            clproto.decode(clproto.from_json(new_ros_param.get_parameter_value().string_value)).data(),
            value_types[0].data())


def test_vector_parameter_read_write():
    vec = np.random.rand(3)
    param = sr.Parameter("vector", sr.ParameterType.VECTOR)
    ros_param = write_parameter(param)
    assert ros_param.type_ == Parameter.Type.NOT_SET
    param = sr.Parameter("vector", vec, sr.ParameterType.VECTOR)
    ros_param = write_parameter(param)
    assert ros_param.type_ == Parameter.Type.DOUBLE_ARRAY
    assert ros_param.name == param.get_name()
    ros_vec = ros_param.get_parameter_value().double_array_value
    assert vec.tolist() == ros_vec.tolist()

    new_param = read_parameter(ros_param)
    assert new_param.get_name() == ros_param.name
    assert new_param.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY
    assert new_param.get_value() == ros_vec.tolist()

    new_param = read_parameter_const(ros_param, param)
    assert new_param.get_name() == ros_param.name
    assert new_param.get_parameter_type() == sr.ParameterType.VECTOR
    assert_np_array_equal(new_param.get_value(), vec)


def test_matrix_parameter_read_write():
    mat = np.random.rand(3, 4)
    param = sr.Parameter("vector", sr.ParameterType.MATRIX)
    ros_param = write_parameter(param)
    assert ros_param.type_ == Parameter.Type.NOT_SET
    param = sr.Parameter("matrix", mat, sr.ParameterType.MATRIX)
    ros_param = write_parameter(param)
    assert ros_param.type_ == Parameter.Type.DOUBLE_ARRAY
    assert ros_param.name == param.get_name()
    ros_mat = ros_param.get_parameter_value().double_array_value
    assert mat.flatten().tolist() == ros_mat.tolist()

    new_param = read_parameter(ros_param)
    assert new_param.get_name() == ros_param.name
    assert new_param.get_parameter_type() == sr.ParameterType.DOUBLE_ARRAY
    assert new_param.get_value() == ros_mat.tolist()

    new_param = read_parameter_const(ros_param, param)
    assert new_param.get_name() == ros_param.name
    assert new_param.get_parameter_type() == sr.ParameterType.MATRIX
    assert_np_array_equal(new_param.get_value(), mat)
    assert new_param.get_value().shape == mat.shape


def test_state_parameter_const_read_invalid():
    param = sr.Parameter("test", sr.CartesianState("test"), sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE)
    ros_param = write_parameter(param)

    expected_param = sr.Parameter("test", sr.ParameterType.STATE, sr.StateType.JOINT_STATE)
    with pytest.raises(ParameterTranslationError):
        read_parameter_const(ros_param, expected_param)
