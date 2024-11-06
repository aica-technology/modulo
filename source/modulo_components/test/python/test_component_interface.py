import time

import clproto
import numpy as np
import pytest
import rclpy
import state_representation as sr
from modulo_interfaces.srv import EmptyTrigger, StringTrigger
from modulo_components.component_interface import ComponentInterface
from modulo_core.exceptions import CoreError, LookupTransformError
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState


def raise_(ex):
    raise ex


@pytest.fixture()
def component_interface(ros_context):
    yield ComponentInterface('component_interface')


def test_rate_parameter(ros_context):
    component_interface = ComponentInterface('component_interface')
    assert component_interface.get_parameter_value("rate") == 10.0
    assert component_interface.get_rate() == 10.0
    assert component_interface.get_period() == 0.1

    parameter_overrides = [rclpy.Parameter("rate", value=200.0)]
    component_interface = ComponentInterface('component_interface', parameter_overrides=parameter_overrides)
    assert component_interface.get_parameter_value("rate") == 200
    assert component_interface.get_rate() == 200.0
    assert component_interface.get_period() == 0.005


def test_add_bool_predicate(component_interface):
    component_interface.add_predicate('foo', True)
    assert 'foo' in component_interface._ComponentInterface__predicates.keys()
    assert component_interface._ComponentInterface__predicates['foo'].get_value()


def test_add_function_predicate(component_interface):
    component_interface.add_predicate('foo', lambda: False)
    assert 'foo' in component_interface._ComponentInterface__predicates.keys()
    assert not component_interface._ComponentInterface__predicates['foo'].get_value()


def test_get_predicate(component_interface):
    component_interface.add_predicate('foo', True)
    assert component_interface.get_predicate('foo')
    component_interface.add_predicate('bar', lambda: True)
    assert component_interface.get_predicate('bar')
    # predicate does not exist, expect false
    assert not component_interface.get_predicate('test')
    # error in callback function except false
    component_interface.add_predicate('error', lambda: raise_(RuntimeError("An error occurred")))
    assert not component_interface.get_predicate('error')


def test_set_predicate(component_interface):
    component_interface.add_predicate('foo', True)
    component_interface.set_predicate('foo', False)
    assert not component_interface.get_predicate('foo')
    # predicate does not exist so setting won't do anything
    component_interface.set_predicate('bar', True)
    assert not component_interface.get_predicate('bar')
    component_interface.add_predicate('bar', True)
    component_interface.set_predicate('bar', lambda: False)
    assert not component_interface.get_predicate('bar')


def test_declare_signal(component_interface):
    component_interface.declare_input("input", "test")
    assert component_interface.get_parameter_value("input_topic") == "test"
    assert "input" not in component_interface._ComponentInterface__inputs.keys()
    component_interface.declare_output("output", "test_again")
    assert component_interface.get_parameter_value("output_topic") == "test_again"
    assert "test_again" not in component_interface._ComponentInterface__outputs.keys()


def test_add_remove_input(component_interface):
    component_interface.add_input("8_teEsTt_#1@3", "test", Bool)
    assert "test_13" in component_interface._ComponentInterface__inputs.keys()
    assert component_interface.get_parameter_value("test_13_topic") == "~/test_13"

    component_interface.add_input("9_tEestT_#1@5", "test", Bool, default_topic="/topic")
    assert "test_15" in component_interface._ComponentInterface__inputs.keys()
    assert component_interface.get_parameter_value("test_15_topic") == "/topic"

    component_interface.add_input("test_13", "test", String)
    assert component_interface._ComponentInterface__inputs["test_13"].msg_type == Bool

    component_interface.remove_input("test_13")
    assert "test_13" not in component_interface._ComponentInterface__inputs.keys()


def test_add_service(component_interface, ros_exec, make_service_client):
    def empty_callback():
        return {"success": True, "message": "test"}

    component_interface.add_service("empty", empty_callback)
    assert len(component_interface._ComponentInterface__services_dict) == 1
    assert "empty" in component_interface._ComponentInterface__services_dict.keys()

    def string_callback(payload: str):
        return {"success": True, "message": payload}

    component_interface.add_service("string", string_callback)
    assert len(component_interface._ComponentInterface__services_dict) == 2
    assert "string" in component_interface._ComponentInterface__services_dict.keys()

    # adding a service under an existing name should fail for either callback type, but is exception safe
    component_interface.add_service("empty", empty_callback)
    component_interface.add_service("empty", string_callback)
    assert len(component_interface._ComponentInterface__services_dict) == 2

    component_interface.add_service("string", empty_callback)
    component_interface.add_service("string", string_callback)
    assert len(component_interface._ComponentInterface__services_dict) == 2

    # adding an empty service name should fail
    component_interface.add_service("", empty_callback)
    component_interface.add_service("", string_callback)
    assert len(component_interface._ComponentInterface__services_dict) == 2

    # adding a mangled service name should succeed under a sanitized name
    component_interface.add_service("8_teEsTt_#1@3", empty_callback)
    assert len(component_interface._ComponentInterface__services_dict) == 3
    assert "test_13" in component_interface._ComponentInterface__services_dict.keys()

    client = make_service_client(
        {"/component_interface/empty": EmptyTrigger, "/component_interface/string": StringTrigger})
    ros_exec.add_node(component_interface)
    ros_exec.add_node(client)
    future = client.call_async("/component_interface/empty", EmptyTrigger.Request())
    ros_exec.spin_until_future_complete(future, timeout_sec=0.5)
    assert future.done() and future.result().success
    assert future.result().message == "test"

    future = client.call_async("/component_interface/string", StringTrigger.Request(payload="payload"))
    ros_exec.spin_until_future_complete(future, timeout_sec=0.5)
    assert future.done() and future.result().success
    assert future.result().message == "payload"


def test_create_output(component_interface):
    component_interface._ComponentInterface__create_output(
        "test", "test", Bool, clproto.MessageType.UNKNOWN_MESSAGE, "/topic", True, True)
    assert "test" in component_interface._ComponentInterface__outputs.keys()
    assert component_interface.get_parameter_value("test_topic") == "/topic"
    assert component_interface._ComponentInterface__outputs["test"]["message_type"] == Bool
    assert component_interface._ComponentInterface__periodic_outputs["test"]

    component_interface._ComponentInterface__create_output(
        "8_teEsTt_#1@3", "test", Bool, clproto.MessageType.UNKNOWN_MESSAGE, "", True, False)
    assert not component_interface._ComponentInterface__periodic_outputs["test_13"]
    component_interface.publish_output("8_teEsTt_#1@3")
    component_interface.publish_output("test_13")
    with pytest.raises(CoreError):
        component_interface.publish_output("")

    component_interface._ComponentInterface__create_output("test_custom", "test", JointState,
                                                           clproto.MessageType.UNKNOWN_MESSAGE, "/topic", True, True)
    assert "test_custom" in component_interface._ComponentInterface__outputs.keys()
    assert component_interface.get_parameter_value("test_custom_topic") == "/topic"
    assert component_interface._ComponentInterface__outputs["test_custom"]["message_type"] == JointState
    data = JointState()
    data.name = ["joint_1", "joint_2"]
    msg = JointState()
    component_interface._ComponentInterface__outputs["test_custom"]["translator"](msg, data)
    assert msg.name == data.name
    assert component_interface._ComponentInterface__periodic_outputs["test_custom"]


def test_tf(component_interface):
    component_interface.add_tf_broadcaster()
    component_interface.add_static_tf_broadcaster()
    component_interface.add_tf_listener()
    send_tf = sr.CartesianPose().Random("test", "world")
    send_static_tf = sr.CartesianPose().Random("static_test", "world")
    component_interface.send_transform(send_tf)
    component_interface.send_static_transform(send_static_tf)
    for i in range(10):
        rclpy.spin_once(component_interface)
    with pytest.raises(LookupTransformError):
        component_interface.lookup_transform("dummy", "world")
    lookup_tf = component_interface.lookup_transform("test", "world")
    identity = send_tf * lookup_tf.inverse()
    assert np.linalg.norm(identity.data()) - 1 < 1e-3
    assert abs(identity.get_orientation().w) - 1 < 1e-3

    time.sleep(1.0)
    with pytest.raises(LookupTransformError):
        component_interface.lookup_transform("test", "world", validity_period=0.9)

    lookup_tf = component_interface.lookup_transform("static_test", "world")
    identity = send_static_tf * lookup_tf.inverse()
    assert np.linalg.norm(identity.data()) - 1 < 1e-3
    assert abs(identity.get_orientation().w) - 1 < 1e-3

    send_tfs = []
    for idx in range(3):
        send_tfs.append(sr.CartesianPose().Random("test_" + str(idx), "world"))
    component_interface.send_transforms(send_tfs)
    for i in range(10):
        rclpy.spin_once(component_interface)
    for tf in send_tfs:
        lookup_tf = component_interface.lookup_transform(tf.get_name(), tf.get_reference_frame())
        identity = tf * lookup_tf.inverse()
        assert np.linalg.norm(identity.data()) - 1 < 1e-3
        assert abs(identity.get_orientation().w) - 1 < 1e-3

    send_static_tfs = []
    for idx in range(3):
        send_static_tfs.append(sr.CartesianPose().Random("test_static_" + str(idx), "world"))
    component_interface.send_static_transforms(send_static_tfs)
    for i in range(10):
        rclpy.spin_once(component_interface)
    for tf in send_static_tfs:
        lookup_tf = component_interface.lookup_transform(tf.get_name(), tf.get_reference_frame())
        identity = tf * lookup_tf.inverse()
        assert np.linalg.norm(identity.data()) - 1 < 1e-3
        assert abs(identity.get_orientation().w) - 1 < 1e-3


def test_get_set_qos(component_interface):
    qos = QoSProfile(depth=5)
    component_interface.set_qos(qos)
    assert qos == component_interface.get_qos()


def test_add_trigger(component_interface):
    component_interface.add_trigger("trigger")
    assert "trigger" in component_interface._ComponentInterface__triggers
    assert not component_interface.get_predicate("trigger")
    component_interface.trigger("trigger")
