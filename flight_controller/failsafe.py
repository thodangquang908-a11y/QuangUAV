import numpy as np
from flight_controller.state_machine import SystemState, FlightMode, SystemStateMachine
from core.logger import Logger

class FailsafeSystem:
    """Hệ thống an toàn chuẩn PX4 R&D: Kiểm tra độ tuổi gói dữ liệu (Data Age Monitoring)."""
    def __init__(self, sm: SystemStateMachine) -> None:
        self.sm = sm
        self.log = Logger.get_logger("Failsafe")
        self.rc_loss_timer = 0.0
        self.battery_level = 100.0

    def monitor(self, sensors: dict, rc_valid: bool, current_mode: FlightMode, sim_time: float) -> FlightMode:
        dt = 0.02 # Chu kỳ chạy 50Hz của Failsafe loop
        self.battery_level -= 0.1 * dt
        if self.battery_level < 15.0 and current_mode != FlightMode.LAND:
            self.log.warning("🔋 PIN CRITICAL LOW (<15%)! Kích hoạt Failsafe -> LAND ngay lập tức!")
            return FlightMode.LAND
            
        if not rc_valid:
            self.rc_loss_timer += dt
            if self.rc_loss_timer > 1.0 and self.sm.is_armed:
                self.log.error("📡 MẤT SÓNG ĐIỀU KHIỂN RC > 1.0s! Kích hoạt EMERGENCY STOP!")
                self.sm.transition_to(SystemState.EMERGENCY)
                return FlightMode.KILL
        else:
            self.rc_loss_timer = 0.0

        # GPS Monitoring: Kiểm tra độ tuổi của mẫu dữ liệu (Data Age > 1.5s mới kích hoạt)
        gps = sensors.get("gps", {})
        last_gps_time = gps.get("timestamp", sim_time)
        if sim_time - last_gps_time > 1.5 and sim_time > 1.0:
            if current_mode in [FlightMode.POSITION_HOLD, FlightMode.MISSION]:
                self.log.warning("⚠️ Mất tín hiệu GPS > 1.5s -> Hạ cấp về ALTITUDE HOLD!")
                return FlightMode.ALTITUDE_HOLD

        # Barometer Monitoring: Kiểm tra độ tuổi mẫu Baro (Data Age > 0.5s mới kích hoạt)
        baro = sensors.get("baro", {})
        last_baro_time = baro.get("timestamp", sim_time)
        if sim_time - last_baro_time > 0.5 and sim_time > 0.5:
            if current_mode in [FlightMode.ALTITUDE_HOLD, FlightMode.POSITION_HOLD, FlightMode.MISSION]:
                self.log.error("⚠️ Mất cảm biến áp suất Baro > 0.5s -> Kích hoạt LAND!")
                return FlightMode.LAND

        return current_mode
