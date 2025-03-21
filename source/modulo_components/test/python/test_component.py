import pytest
from modulo_components.component import Component
from modulo_core.exceptions import CoreError
from std_msgs.msg import Bool, String


@pytest.fixture()
def component(ros_context):
    yield Component('component')


def test_add_remove_output(component):
    component.add_output("8_teEsTt_#1@3", "test", Bool)
    assert "test_13" in component._ComponentInterface__outputs.keys()
    assert component.get_parameter_value("test_13_topic") == "~/test_13"
    with pytest.raises(CoreError):
        component.publish_output("test_13")

    component.add_output("9_tEestT_#1@5", "test", Bool, default_topic="/topic")
    assert "test_15" in component._ComponentInterface__outputs.keys()
    assert component.get_parameter_value("test_15_topic") == "/topic"

    component.add_output("test_13", "test", String)
    assert component._ComponentInterface__outputs["test_13"]["message_type"] == Bool

    component.remove_output("test_13")
    assert "test_13" not in component._ComponentInterface__outputs.keys()

    component.add_output("8_teEsTt_#1@3", "test", Bool, publish_on_step=False)
    assert not component._ComponentInterface__periodic_outputs["test_13"]
    component.publish_output("8_teEsTt_#1@3")
    component.publish_output("test_13")
    with pytest.raises(CoreError):
        component.publish_output("")
