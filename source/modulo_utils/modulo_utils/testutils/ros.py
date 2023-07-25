import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def ros_exec():
    if not rclpy.ok():
        rclpy.init()
    executor = SingleThreadedExecutor()
    yield executor
    if rclpy.ok():
        rclpy.shutdown()
