from typing import Dict

import pytest
from rclpy import Future
from rclpy.node import Node
from rclpy.service import SrvType, SrvTypeRequest


class ClientNode(Node):
    def __init__(self, services: Dict[str, SrvType], timeout_sec=0.5):
        """
        Client node for triggering service calls

        :param services: Dict where each item represents the full service name with corresponding service type
        :param timeout_sec: The timeout for waiting for the service
        :raise RuntimeError if a service is not available within the specified timeout
        """
        super().__init__("client_node")
        self.__clients = {}
        for name, srv_type in services.items():
            client = self.create_client(srv_type, name)
            if not client.wait_for_service(timeout_sec):
                raise RuntimeError(f"Service {name} not available")
            self.__clients[name] = client

    def call_async(self, name: str, request: SrvTypeRequest) -> Future:
        return self.__clients[name].call_async(request)


@pytest.fixture
def make_service_client():
    """
    Create a client node for triggering service calls. Provide\n
    services (Dict[str, SrvType]): Dict where each item represents the full service name with corresponding service
    type\n
    timeout_sec (float): Optional timeout for waiting for the service

    :raise RuntimeError: if a service is not available within the specified timeout
    """

    def _make_service_client(services: Dict[str, SrvType], timeout_sec=0.5):
        return ClientNode(services, timeout_sec)

    return _make_service_client
