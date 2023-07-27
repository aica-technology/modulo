from typing import List

import pytest
from rclpy import Future
from rclpy.node import Node

from modulo_component_interfaces.msg import Predicate


class PredicatesListener(Node):
    def __init__(self, component: str, predicates: List[str]):
        """
        Listener node subscribing to predicate topics and setting its Future to done once at least one of the predicates
        is True

        :param component: Name of the publishing component
        :param predicates: A list of predicates to subscribe to
        """
        super().__init__('predicates_listener')
        self.__future = Future()
        self.__component = component
        self.__predicates = {}
        for predicate in predicates:
            self.__predicates[predicate] = None
        self.__subscription = self.create_subscription(Predicate, "/predicates", self.__callback, 10)

    @property
    def predicates_future(self) -> Future:
        """
        The Future object of the node
        """
        return self.__future

    @property
    def predicate_values(self) -> dict:
        """
        The values of the predicates
        """
        return self.__predicates

    def __callback(self, message):
        if message.component == self.__component:
            for predicate in self.__predicates.keys():
                if message.predicate == predicate:
                    self.__predicates[predicate] = message.value
                    if message.value:
                        self.__future.set_result(True)


@pytest.fixture
def make_predicates_listener():
    """
    Create a listener node subscribing to predicate topics and setting its Future to done once at least one of the
    predicates is True. Provide\n
    component (str): Name of the publishing component\n
    predicates (List[str]): A list of predicates to subscribe to
    """

    def _make_predicates_listener(component: str, predicates: List[str]):
        return PredicatesListener(component, predicates)

    return _make_predicates_listener
