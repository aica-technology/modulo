import time

import numpy as np
import pytest
import rclpy
import state_representation as sr
from modulo_components.component_interface import ComponentInterface
from modulo_components.exceptions import LookupTransformError
from std_msgs.msg import Bool, String


def raise_(ex):
    raise ex


@pytest.fixture()
def component_interface(ros_context):
    yield ComponentInterface('component_interface')


def test_add_bool_predicate(component_interface):
    component_interface.add_predicate('foo', True)
    assert 'foo' in component_interface._predicates.keys()
    assert component_interface._predicates['foo']


def test_add_function_predicate(component_interface):
    component_interface.add_predicate('foo', lambda: False)
    assert 'foo' in component_interface._predicates.keys()
    assert not component_interface._predicates['foo']()


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
    assert "input" not in component_interface._inputs.keys()
    component_interface.declare_output("output", "test_again")
    assert component_interface.get_parameter_value("output_topic") == "test_again"
    assert "test_again" not in component_interface._outputs.keys()


def test_add_remove_input(component_interface):
    component_interface.add_input("_tEsT_#1@3", "test", Bool)
    assert "test_13" in component_interface._inputs.keys()
    assert component_interface.get_parameter_value("test_13_topic") == "~/test_13"

    component_interface.add_input("_tEsT_#1@5", "test", Bool, default_topic="/topic")
    assert "test_15" in component_interface._inputs.keys()
    assert component_interface.get_parameter_value("test_15_topic") == "/topic"

    component_interface.add_input("test_13", "test", String)
    assert component_interface._inputs["test_13"].msg_type == Bool

    component_interface.remove_input("test_13")
    assert "test_13" not in component_interface._inputs.keys()


def test_add_service(component_interface):
    def empty_callback():
        return {"success": True, "message": "test"}
    component_interface.add_service("empty", empty_callback)
    assert len(component_interface._services_dict) == 1
    assert "empty" in component_interface._services_dict.keys()

    def string_callback():
        return {"success": True, "message": "test"}
    component_interface.add_service("string", string_callback)
    assert len(component_interface._services_dict) == 2
    assert "string" in component_interface._services_dict.keys()

    # adding a service under an existing name should fail for either callback type, but is exception safe
    component_interface.add_service("empty", empty_callback)
    component_interface.add_service("empty", string_callback)
    assert len(component_interface._services_dict) == 2

    component_interface.add_service("string", empty_callback)
    component_interface.add_service("string", string_callback)
    assert len(component_interface._services_dict) == 2

    # adding a mangled service name should succeed under a sanitized name
    component_interface.add_service("_tEsT_#1@3", empty_callback)
    assert len(component_interface._services_dict) == 3
    assert "test_13" in component_interface._services_dict.keys()

    # TODO: use a service client to trigger the service and test the behaviour


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


def test_add_trigger(component_interface):
    component_interface.add_trigger("trigger")
    assert "trigger" in component_interface._triggers.keys()
    assert not component_interface._triggers["trigger"]
    assert not component_interface.get_predicate("trigger")
    component_interface.trigger("trigger")
    # When reading, the trigger will be true only once
    component_interface._triggers["trigger"] = True
    assert component_interface._triggers["trigger"]
    assert component_interface.get_predicate("trigger")
    # After the predicate function was evaluated once, the trigger is back to false
    assert not component_interface._triggers["trigger"]
    assert not component_interface.get_predicate("trigger")
