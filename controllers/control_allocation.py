import numpy as np

class ControlAllocator:
    """Bộ phân bổ lực chuẩn 22:49: Mở rộng trần lực động cơ lên 7.5 N."""
    def __init__(self, arm_length: float = 0.25, km: float = 0.05) -> None:
        self.L_eff = arm_length * np.sqrt(2.0) / 2.0
        self.km = km

    def allocate(self, total_thrust: float, torques: np.ndarray) -> np.ndarray:
        tau_r, tau_p, tau_y = torques[0], torques[1], torques[2]
        
        f1 = 0.25 * total_thrust - tau_r / (4.0 * self.L_eff) + tau_p / (4.0 * self.L_eff) + tau_y / (4.0 * self.km)
        f2 = 0.25 * total_thrust + tau_r / (4.0 * self.L_eff) - tau_p / (4.0 * self.L_eff) + tau_y / (4.0 * self.km)
        f3 = 0.25 * total_thrust + tau_r / (4.0 * self.L_eff) + tau_p / (4.0 * self.L_eff) - tau_y / (4.0 * self.km)
        f4 = 0.25 * total_thrust - tau_r / (4.0 * self.L_eff) - tau_p / (4.0 * self.L_eff) - tau_y / (4.0 * self.km)
        
        motor_thrusts = np.array([f1, f2, f3, f4], dtype=np.float64)
        
        # [Yêu cầu c]: Tăng giới hạn motor thrust từ 6.5N lên 7.5N
        motor_thrusts = np.clip(motor_thrusts, 0.0, 7.5)
        
        ct = 1.0e-5
        rpms = np.sqrt(np.maximum(0.0, motor_thrusts) / ct)
        pwms = 1000.0 + (rpms / 1200.0) * 1000.0
        return np.clip(pwms, 1000.0, 2000.0)
