from enum import Enum, auto


class SystemState(Enum):
    IDLE = auto()
    READY_CHECK = auto()
    INBOUND_PICKUP = auto()
    ROBOT_LOADING = auto()
    MAIN_CONVEYOR = auto()
    SORTING = auto()
    REGION_CONVEYOR = auto()
    BOX_LOADING = auto()
    BOX_CHECK = auto()
    OUTBOUND = auto()
    BOX_REPLACE = auto()
    COMPLETE = auto()
    ERROR = auto()
    RECOVERY = auto()
    E_STOP = auto()


class MissionFSM:
    def __init__(self):
        self.state = SystemState.IDLE

    def step(self):
        # TODO: transition logic
        return self.state
