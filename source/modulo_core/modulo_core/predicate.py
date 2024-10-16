from typing import Callable, Optional

from modulo_core.exceptions import CoreError


class Predicate:
    def __init__(self, predicate_function: Callable[[], bool]):
        self.__predicate = predicate_function
        value = predicate_function()
        if not isinstance(value, bool):
            raise CoreError("Predicate function does not return a bool")
        self.__previous_value: Optional[bool] = None

    def get_value(self) -> bool:
        return self.__predicate()

    def set_predicate(self, predicate_function: Callable[[], bool]):
        value = predicate_function()
        if not isinstance(value, bool):
            raise RuntimeError("Predicate function does not return a bool")
        self.__predicate = predicate_function

    def query(self) -> Optional[bool]:
        new_value = self.__predicate()
        if new_value is not self.__previous_value:
            self.__previous_value = new_value
            return new_value
        return None
