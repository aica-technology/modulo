import pytest
import rclpy
import state_representation as sr
from modulo_components.component_interface import ComponentInterface
from modulo_core.exceptions import ParameterError
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from rclpy.parameter import Parameter


class EmtpyParameterInterface(ComponentInterface):
    def __init__(self, node_name, allow_empty=True, add_parameter=True, empty_parameter=True, *kargs, **kwargs):
        super().__init__(node_name, *kargs, **kwargs)
        self._allow_empty = allow_empty
        if add_parameter:
            if empty_parameter:
                self.add_parameter(sr.Parameter("name", sr.ParameterType.STRING), "Name parameter")
            else:
                self.add_parameter(sr.Parameter("name", "test", sr.ParameterType.STRING), "Name parameter")
        self.add_parameter(sr.Parameter("value", 0.0, sr.ParameterType.DOUBLE), "Value parameter")

    def get_ros_parameter(self, name: str) -> rclpy.Parameter:
        return rclpy.node.Node.get_parameter(self, name)

    def set_ros_parameter(self, param: rclpy.Parameter) -> SetParametersResult:
        return rclpy.node.Node.set_parameters(self, [param])[0]

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() == "name":
            if parameter.is_empty():
                return self._allow_empty
            elif not parameter.get_value():
                self.get_logger().error("Provide a non empty value for parameter 'name'")
                return False
        elif parameter.get_name() == "value":
            value = parameter.get_value()
            if value < 0.0:
                parameter.set_value(abs(value))
            if abs(value) > 10:
                return False
            return True
        return True


@pytest.fixture()
def component_test(ros_context):
    yield EmtpyParameterInterface("component")


def test_not_allow_empty_on_construction(ros_context):
    # Construction with empty parameter should raise if the empty parameter is not allowed
    with pytest.raises(ParameterError):
        EmtpyParameterInterface("component", False)


def test_not_allow_empty(ros_context):
    component = EmtpyParameterInterface("component", allow_empty=False, add_parameter=False)
    with pytest.raises(ParameterError):
        component.add_parameter(sr.Parameter("name", sr.ParameterType.STRING), "Test parameter")
    with pytest.raises(ParameterError):
        component.get_parameter("name")
    with pytest.raises(ParameterNotDeclaredException):
        assert Node.get_parameter(component, "name")

    component = EmtpyParameterInterface("component", allow_empty=True, add_parameter=False)
    component.add_parameter(sr.Parameter("name", sr.ParameterType.STRING), "Test parameter")
    assert component.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component.get_parameter("name").is_empty()
    assert Node.get_parameter(component, "name").type_ == Parameter.Type.NOT_SET


def test_validate_empty_parameter(component_test):
    # component_test comes with empty parameter 'name'
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter("name").is_empty()
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.NOT_SET

    # Trying to overwrite a parameter is not possible
    component_test.add_parameter(sr.Parameter("name", sr.ParameterType.BOOL), "Test parameter")
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter("name").is_empty()
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.NOT_SET

    # Set parameter with empty parameter isn't possible because there is no method for that
    # component_test.set_parameter(sr.Parameter("name", sr.ParameterType.STRING))

    # Set parameter value from ROS interface should update ROS parameter type
    Node.set_parameters(component_test, [Parameter("name", value="test")])
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "test"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "test"

    # Set parameter value from component interface
    component_test.set_parameter_value("name", "again", sr.ParameterType.STRING)
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "again"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "again"

    # Setting it with empty value should be rejected in parameter evaluation
    component_test.set_parameter_value("name", "", sr.ParameterType.STRING)
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "again"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "again"

    # Setting it with empty value should be rejected in parameter evaluation
    Node.set_parameters(component_test, [Parameter("name", type_=Parameter.Type.STRING, value="")])
    assert component_test.get_parameter("name").get_parameter_type() == sr.ParameterType.STRING
    assert component_test.get_parameter_value("name") == "again"
    assert Node.get_parameter(component_test, "name").type_ == Parameter.Type.STRING
    assert Node.get_parameter(component_test, "name").value == "again"

    # TODO clarify that behavior somewhere
    # Setting a parameter with type NOT_SET undeclares that parameter
    Node.set_parameters(component_test, [Parameter("name", type_=Parameter.Type.NOT_SET)])
    with pytest.raises(ParameterNotDeclaredException):
        Node.get_parameter(component_test, "name")


def test_change_parameter_type(component_test):
    # Add parameter from component interface
    component_test.add_parameter(sr.Parameter("int", sr.ParameterType.INT), "Test parameter")
    assert component_test.describe_parameter("int").dynamic_typing
    component_test.set_parameter_value("int", 1, sr.ParameterType.INT)
    assert component_test.get_parameter("int").get_parameter_type() == sr.ParameterType.INT
    assert component_test.get_parameter_value("int") == 1
    ros_param = component_test.get_ros_parameter("int")
    assert ros_param.type_ == Parameter.Type.INTEGER
    assert ros_param.value == 1

    # Set parameter value from component interface with different type should not work
    component_test.set_parameter_value("int", 2.0, sr.ParameterType.DOUBLE)
    assert component_test.get_parameter("int").get_parameter_type() == sr.ParameterType.INT
    assert component_test.get_parameter_value("int") == 1
    ros_param = component_test.get_ros_parameter("int")
    assert ros_param.type_ == Parameter.Type.INTEGER
    assert ros_param.value == 1

    # Set parameter value from ROS interface with different type should not work
    component_test.set_ros_parameter(Parameter("int", value=2.0))
    assert component_test.get_parameter("int").get_parameter_type() == sr.ParameterType.INT
    assert component_test.get_parameter_value("int") == 1
    ros_param = component_test.get_ros_parameter("int")
    assert ros_param.type_ == Parameter.Type.INTEGER
    assert ros_param.value == 1


def test_parameter_overrides(ros_context):
    # Construction with not allowing empty parameters but providing the parameter override should succeed
    EmtpyParameterInterface("component", False, parameter_overrides=[Parameter("name", value="test")])


def test_parameter_overrides_empty(ros_context):
    # Construction with not allowing empty parameters and providing an uninitialized parameter override should not succeed
    with pytest.raises(ParameterError):
        EmtpyParameterInterface("component", allow_empty=False, empty_parameter=False,
                                parameter_overrides=[Parameter("name")])


def test_value_parameter(component_test):
    assert component_test.get_parameter_value("value") == 0.0
    component_test.set_parameter_value("value", 1.0, sr.ParameterType.DOUBLE)
    assert component_test.get_parameter_value("value") == 1.0
    assert component_test.get_ros_parameter("value").get_parameter_value().double_value == 1.0
    component_test.set_parameter_value("value", -2.0, sr.ParameterType.DOUBLE)
    assert component_test.get_parameter_value("value") == 2.0
    assert component_test.get_ros_parameter("value").get_parameter_value().double_value == 2.0
    component_test.set_parameter_value("value", 11.0, sr.ParameterType.DOUBLE)
    assert component_test.get_parameter_value("value") == 2.0
    assert component_test.get_ros_parameter("value").get_parameter_value().double_value == 2.0

    component_test.set_ros_parameter(Parameter("value", value=3.0))
    assert component_test.get_parameter_value("value") == 3.0
    assert component_test.get_ros_parameter("value").get_parameter_value().double_value == 3.0
    component_test.set_ros_parameter(Parameter("value", value=-4.0))
    assert component_test.get_parameter_value("value") == 4.0
    assert component_test.get_ros_parameter("value").get_parameter_value().double_value == 4.0
    component_test.set_ros_parameter(Parameter("value", value=-11.0))
    assert component_test.get_parameter_value("value") == 4.0
    assert component_test.get_ros_parameter("value").get_parameter_value().double_value == 4.0
