import numpy as np
import time

class HealthMonitor:
    """TẦNG GIÁM SÁT SỨC KHỎE HỆ THỐNG (HEALTH MONITOR)"""
    def __init__(self, max_latency_ms: float = 5.0):
        self.max_latency_ms = max_latency_ms
        self.health_status = "HEALTHY"
        self.last_compute_time_ms = 0.0

    def inspect(self, compute_time_ms: float, control_signals: np.ndarray) -> bool:
        self.last_compute_time_ms = compute_time_ms
        
        # 1. Kiểm tra thời gian tính toán vòng lặp (Compute Latency Guard)
        if compute_time_ms > self.max_latency_ms:
            self.health_status = "LATENCY_WARNING"
            return False
            
        # 2. Kiểm tra tín hiệu NaN hoặc Inf (Safety Guard)
        if np.any(np.isnan(control_signals)) or np.any(np.isinf(control_signals)):
            self.health_status = "CRITICAL_SIGNAL_ERROR"
            return False

        self.health_status = "HEALTHY"
        return True


class SafeTransitionLayer:
    """TẦNG CHUYỂN TIẾP TRẠNG THÁI AN TOÀN (SAFE TRANSITION - CROSS FADING)"""
    def __init__(self, blend_duration_sec: float = 0.1):
        self.blend_duration = blend_duration_sec
        self.is_blending = False
        self.start_time = 0.0
        self.old_controller = None
        self.new_controller = None

    def start_transition(self, old_ctrl, new_ctrl, current_sim_time: float):
        self.old_controller = old_ctrl
        self.new_controller = new_ctrl
        if hasattr(self.new_controller, 'reset'):
            self.new_controller.reset() # Warm-start controller mới
            
        self.start_time = current_sim_time
        self.is_blending = True

    def compute_blended_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float, current_sim_time: float) -> np.ndarray:
        elapsed = current_sim_time - self.start_time
        alpha = min(1.0, max(0.0, elapsed / self.blend_duration))

        u_old = self.old_controller.compute_control_signals(target_state, current_state, dt)
        u_new = self.new_controller.compute_control_signals(target_state, current_state, dt)

        # Trộn tín hiệu điều khiển theo trọng số alpha mượt mà (Cross-fading)
        u_blended = (1.0 - alpha) * u_old + alpha * u_new

        if alpha >= 1.0:
            self.is_blending = False

        return u_blended
