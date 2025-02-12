import clproto
import pytest
import rclpy.clock
import state_representation as sr
import datetime
from rclpy import Parameter


@pytest.fixture
def cart_state():
    return sr.CartesianState().Random("test", "ref")


@pytest.fixture
def joint_state():
    return sr.JointState().Random("robot", 3)


@pytest.fixture
def clock():
    return rclpy.clock.Clock()


@pytest.fixture
def cartesian_trajectory():
    trajectory = sr.CartesianTrajectory("test", "ref")
    for i in range(10):
        trajectory.add_point(sr.CartesianState().Random("test", "ref"), datetime.timedelta(seconds=(i+1)*10))
    return trajectory


@pytest.fixture
def joint_trajectory():
    trajectory = sr.JointTrajectory("test")
    trajectory.set_joint_names(["joint1", "joint2", "joint3"])
    for i in range(10):
        trajectory.add_point(
            sr.JointState().Random("test", trajectory.get_joint_names()),
            datetime.timedelta(seconds=(i + 1.05) * 10))
    return trajectory


@pytest.fixture
def parameters():
    return {"bool": [True, sr.ParameterType.BOOL, False, Parameter.Type.BOOL],
            "bool_array": [[True, False], sr.ParameterType.BOOL_ARRAY, [False], Parameter.Type.BOOL_ARRAY],
            "int": [1, sr.ParameterType.INT, 2, Parameter.Type.INTEGER],
            "int_array": [[1, 2], sr.ParameterType.INT_ARRAY, [2], Parameter.Type.INTEGER_ARRAY],
            "double": [1.0, sr.ParameterType.DOUBLE, 2.0, Parameter.Type.DOUBLE],
            "double_array": [[1.0, 2.0], sr.ParameterType.DOUBLE_ARRAY, [2.0], Parameter.Type.DOUBLE_ARRAY],
            "string": ["1", sr.ParameterType.STRING, "2", Parameter.Type.STRING],
            "string_array": [["1", "2"], sr.ParameterType.STRING_ARRAY, ["2"], Parameter.Type.STRING_ARRAY]
            }


@pytest.fixture
def state_parameters():
    return {"cartesian_state": [sr.CartesianState().Random("test"), sr.StateType.CARTESIAN_STATE,
                                clproto.CARTESIAN_STATE_MESSAGE],
            "cartesian_pose": [sr.CartesianPose().Random("test"), sr.StateType.CARTESIAN_POSE,
                               clproto.CARTESIAN_POSE_MESSAGE],
            "joint_state": [sr.JointState().Random("test", 3), sr.StateType.JOINT_STATE,
                            clproto.JOINT_STATE_MESSAGE],
            "joint_positions": [sr.JointPositions().Random("test", 3), sr.StateType.JOINT_POSITIONS,
                                clproto.JOINT_POSITIONS_MESSAGE]
            }
