from enum import Enum
from core.logger import Logger

class SystemState(Enum):
    BOOT = "BOOT"; INIT = "INIT"; READY = "READY"; ARMED = "ARMED"
    TAKEOFF = "TAKEOFF"; MISSION = "MISSION"; LAND = "LAND"
    DISARM = "DISARM"; EMERGENCY = "EMERGENCY"

class FlightMode(Enum):
    MANUAL = "MANUAL"; STABILIZE = "STABILIZE"; ALTITUDE_HOLD = "ALTITUDE_HOLD"
    POSITION_HOLD = "POSITION_HOLD"; MISSION = "MISSION"; AUTO = "AUTO"
    RTL = "RTL"; LAND = "LAND"; KILL = "KILL"

class SystemStateMachine:
    def __init__(self) -> None:
        self.state = SystemState.BOOT
        self.is_armed = False
        self.log = Logger.get_logger("StateMachine")
        self.transition_to(SystemState.INIT)

    def transition_to(self, new_state: SystemState) -> None:
        if self.state != new_state:
            self.log.info(f"🔄 CHUYỂN TRẠNG THÁI HỆ THỐNG: [{self.state.value}] ---> [{new_state.value}]")
            self.state = new_state
            if new_state in [SystemState.ARMED, SystemState.TAKEOFF, SystemState.MISSION]:
                self.is_armed = True
            elif new_state in [SystemState.READY, SystemState.DISARM, SystemState.EMERGENCY]:
                self.is_armed = False

    def arm_command(self) -> bool:
        if self.state in [SystemState.READY, SystemState.INIT]:
            self.transition_to(SystemState.ARMED)
            return True
        return False

    def disarm_command(self) -> None:
        self.transition_to(SystemState.DISARM)
        self.transition_to(SystemState.READY)
