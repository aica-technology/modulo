import pytest
from modulo_components.exceptions import ComponentError
from modulo_components.lifecycle_component import LifecycleComponent
from std_msgs.msg import Bool, String


@pytest.fixture()
def lifecycle_component(ros_context):
    yield LifecycleComponent('lifecycle_component')


def test_add_remove_output(lifecycle_component):
    lifecycle_component.add_output("8_tEsT_#1@3", "test", Bool)
    assert "test_13" in lifecycle_component._outputs.keys()
    assert lifecycle_component.get_parameter_value("test_13_topic") == "~/test_13"
    with pytest.raises(ComponentError):
        lifecycle_component.publish_output("test_13")

    lifecycle_component.add_output("9_tEsT_#1@5", "test", Bool, default_topic="/topic")
    assert "test_15" in lifecycle_component._outputs.keys()
    assert lifecycle_component.get_parameter_value("test_15_topic") == "/topic"

    lifecycle_component.add_output("test_13", "test", String)
    assert lifecycle_component._outputs["test_13"]["message_type"] == Bool

    lifecycle_component.remove_output("test_13")
    assert "test_13" not in lifecycle_component._inputs.keys()

    lifecycle_component.add_output("8_tEsT_#1@3", "test", Bool, publish_on_step=False)
    assert not lifecycle_component._periodic_outputs["test_13"]
    lifecycle_component.publish_output("8_tEsT_#1@3")
    lifecycle_component.publish_output("test_13")
    with pytest.raises(ComponentError):
        lifecycle_component.publish_output("")
