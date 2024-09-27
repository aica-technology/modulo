import pytest
import rclpy.exceptions
import rclpy.node
import state_representation as sr
from modulo_components.component_interface import ComponentInterface
from modulo_core.exceptions import ParameterError
from rcl_interfaces.msg import SetParametersResult


class ComponentInterfaceTest(ComponentInterface):
    def __init__(self, node_name, *kargs, **kwargs):
        self.validation_return_value = True
        super().__init__(node_name, *kargs, **kwargs)
        self.validate_was_called = False
        self.param = sr.Parameter("test", 1, sr.ParameterType.INT)

    def get_ros_parameter(self, name: str) -> rclpy.Parameter:
        return rclpy.node.Node.get_parameter(self, name)

    def set_ros_parameter(self, param: rclpy.Parameter) -> SetParametersResult:
        return rclpy.node.Node.set_parameters(self, [param])[0]

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        self.validate_was_called = True
        return self.validation_return_value


@pytest.fixture()
def component_interface(ros_context):
    yield ComponentInterfaceTest('component_interface')


def assert_param_value_equal(
        component_interface: ComponentInterfaceTest,
        name: str,
        value: float):
    assert component_interface.get_ros_parameter(
        name).get_parameter_value().integer_value == value
    assert component_interface.get_parameter_value(name) == value
    assert component_interface.get_parameter(name).get_value() == value


def test_add_parameter(component_interface):
    with pytest.raises(ParameterError):
        component_interface.get_parameter("test")
    with pytest.raises(rclpy.exceptions.ParameterNotDeclaredException):
        rclpy.node.Node.get_parameter(component_interface, "test")

    component_interface.add_parameter("param", "Test parameter")
    assert component_interface.validate_was_called
    component_interface.get_parameter("test")
    rclpy.node.Node.get_parameter(component_interface, "test")
    assert component_interface.param.get_parameter_type() == sr.ParameterType.INT
    assert component_interface.param.get_value() == 1


def test_add_parameter_again(component_interface):
    with pytest.raises(ParameterError):
        component_interface.get_parameter_value("test")
    with pytest.raises(rclpy.exceptions.ParameterNotDeclaredException):
        rclpy.node.Node.get_parameter(component_interface, "test")

    component_interface.add_parameter("param", "Test parameter")
    component_interface.validate_was_called = False
    component_interface.add_parameter(
        sr.Parameter("test", 2, sr.ParameterType.INT), "foo")
    assert not component_interface.validate_was_called
    assert_param_value_equal(component_interface, "test", 1)


def test_add_parameter_again_not_attribute(component_interface):
    with pytest.raises(ParameterError):
        component_interface.get_parameter("test")
    with pytest.raises(rclpy.exceptions.ParameterNotDeclaredException):
        rclpy.node.Node.get_parameter(component_interface, "test")

    component_interface.add_parameter(
        component_interface.param, "Test parameter")
    component_interface.validate_was_called = False
    component_interface.add_parameter(
        sr.Parameter("test", 2, sr.ParameterType.INT), "foo")
    assert not component_interface.validate_was_called
    assert_param_value_equal(component_interface, "test", 1)


def test_set_parameter(component_interface):
    assert not component_interface.validate_was_called
    component_interface.set_parameter_value("test", 1, sr.ParameterType.INT)
    assert not component_interface.validate_was_called
    with pytest.raises(rclpy.exceptions.ParameterNotDeclaredException):
        component_interface.get_ros_parameter("test")

    component_interface.add_parameter("param", "Test parameter")

    component_interface.validate_was_called = False
    component_interface.set_parameter_value("test", 2, sr.ParameterType.INT)
    assert component_interface.validate_was_called
    assert_param_value_equal(component_interface, "test", 2)
    assert component_interface.param.get_value() == 2

    component_interface.validate_was_called = False
    component_interface.validation_return_value = False
    component_interface.set_parameter_value("test", 3, sr.ParameterType.INT)
    assert component_interface.validate_was_called
    assert_param_value_equal(component_interface, "test", 2)
    assert component_interface.param.get_value() == 2

    component_interface.set_parameter_value(
        "test", "test", sr.ParameterType.STRING)
    assert component_interface.validate_was_called
    assert_param_value_equal(component_interface, "test", 2)
    assert component_interface.param.get_value() == 2


def test_set_parameter_ros(component_interface):
    component_interface.add_parameter("param", "Test parameter")

    component_interface.validate_was_called = False
    result = component_interface.set_ros_parameter(
        rclpy.Parameter("test", value=2))
    assert component_interface.validate_was_called
    assert result.successful
    assert_param_value_equal(component_interface, "test", 2)
    assert component_interface.param.get_value() == 2

    component_interface.validate_was_called = False
    component_interface.validation_return_value = False
    result = component_interface.set_ros_parameter(
        rclpy.Parameter("test", value=3))
    assert component_interface.validate_was_called
    assert not result.successful
    assert_param_value_equal(component_interface, "test", 2)
    assert component_interface.param.get_value() == 2


def test_get_parameter_description(component_interface):
    component_interface.add_parameter(
        component_interface.param, "Test parameter")
    assert component_interface.describe_parameter(
        "test").description == "Test parameter"

    with pytest.raises(rclpy.exceptions.ParameterNotDeclaredException):
        component_interface.describe_parameter("foo")


def test_read_only_parameter(component_interface):
    component_interface.add_parameter("param", "Test parameter", True)
    assert component_interface.validate_was_called
    component_interface.get_parameter("test")
    component_interface.get_ros_parameter("test")
    assert_param_value_equal(component_interface, "test", 1)

    component_interface.validate_was_called = False
    component_interface.set_parameter_value("test", 2, sr.ParameterType.INT)
    assert not component_interface.validate_was_called
    assert_param_value_equal(component_interface, "test", 1)
    assert component_interface.param.get_value() == 1

    component_interface.validate_was_called = False
    result = component_interface.set_ros_parameter(
        rclpy.Parameter("test", value=2))
    assert not component_interface.validate_was_called
    assert not result.successful
    assert_param_value_equal(component_interface, "test", 1)
