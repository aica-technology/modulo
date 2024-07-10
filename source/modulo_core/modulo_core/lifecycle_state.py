from enum import Enum

class LifecycleState(Enum):
    UNKNOWN = 0
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4
