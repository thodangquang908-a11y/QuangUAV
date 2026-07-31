import numpy as np
from controllers.pid import PIDController
from core.config_loader import ConfigLoader

class RateController:
    """Bộ điều khiển Tốc độ góc (Angular Rate Controller - 400Hz) chuẩn PX4."""
    def __init__(self) -> None:
        cfg = ConfigLoader()
        r = cfg.get("pid.yaml", "controllers.rate")
        self.pid_p = PIDController(r['roll']['kp'], r['roll']['ki'], r['roll']['kd'], 
                                   r['roll']['limit_out'], r['roll']['limit_int'], r['roll']['lpf_cutoff'])
        self.pid_q = PIDController(r['pitch']['kp'], r['pitch']['ki'], r['pitch']['kd'], 
                                   r['pitch']['limit_out'], r['pitch']['limit_int'], r['pitch']['lpf_cutoff'])
        self.pid_r = PIDController(r['yaw']['kp'], r['yaw']['ki'], r['yaw']['kd'], 
                                   r['yaw']['limit_out'], r['yaw']['limit_int'], r['yaw']['lpf_cutoff'])

    def compute(self, target_rates: np.ndarray, current_rates: np.ndarray, dt: float) -> np.ndarray:
        tau_x = self.pid_p.compute(target_rates[0], current_rates[0], dt)
        tau_y = self.pid_q.compute(target_rates[1], current_rates[1], dt)
        tau_z = self.pid_r.compute(target_rates[2], current_rates[2], dt)
        return np.array([tau_x, tau_y, tau_z], dtype=np.float64)
