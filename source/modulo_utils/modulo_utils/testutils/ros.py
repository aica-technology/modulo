import pytest
import rclpy


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ros_exec():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    yield executor
    rclpy.shutdown()
