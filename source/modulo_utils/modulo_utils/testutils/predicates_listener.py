from functools import partial
from typing import List

import pytest
from rclpy import Future
from rclpy.node import Node
from std_msgs.msg import Bool


class PredicatesListener(Node):
    def __init__(self, namespace: str, predicates: List[str]):
        """
        Listener node subscribing to predicate topics and setting its Future to done once at least one of the predicates
        is True

        :param namespace: Namespace of the publishing component (usually '<component_name>')
        :param predicates: A list of predicates to subscribe to
        """
        super().__init__('predicates_listener')
        self.__future = Future()
        self.__predicates = {}
        self.__subscriptions = {}
        for predicate in predicates:
            self.__predicates[predicate] = None
            self.__subscriptions[predicate] = self.create_subscription(Bool, f'/predicates/{namespace}/{predicate}',
                                                                       partial(self.__callback,
                                                                               predicate_name=predicate), 10)
        self.create_timer(0.1, self.__check_future)

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

    def __callback(self, msg, predicate_name):
        self.__predicates[predicate_name] = msg.data

    def __check_future(self):
        if any([value for value in self.__predicates.values()]):
            self.__future.set_result(True)


@pytest.fixture
def make_predicates_listener():
    """
    Create a listener node subscribing to predicate topics and setting its Future to done once at least one of the
    predicates is True. Provide\n
    namespace (str): Namespace of the publishing component (usually '<component_name>')\n
    predicates (List[str]): A list of predicates to subscribe to
    """

    def _make_predicates_listener(namespace: str, predicates: List[str]):
        return PredicatesListener(namespace, predicates)

    return _make_predicates_listener
