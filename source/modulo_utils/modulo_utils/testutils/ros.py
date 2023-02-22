import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ros_exec():
    rclpy.init()
    executor = SingleThreadedExecutor()
    yield executor
    rclpy.shutdown()
