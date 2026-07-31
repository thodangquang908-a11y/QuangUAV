import numpy as np
from controllers.pid import PIDController
from core.config_loader import ConfigLoader

class AttitudeController:
    def __init__(self) -> None:
        cfg = ConfigLoader()
        att = cfg.get("pid.yaml", "controllers.attitude")
        self.pid_roll  = PIDController(att['roll']['kp'], 0.0, 0.0, att['roll']['limit_out'])
        self.pid_pitch = PIDController(att['pitch']['kp'], 0.0, 0.0, att['pitch']['limit_out'])
        self.pid_yaw   = PIDController(att['yaw']['kp'], 0.0, 0.0, att['yaw']['limit_out'])

    def compute(self, target_euler: np.ndarray, current_euler: np.ndarray, dt: float) -> np.ndarray:
        rate_x = self.pid_roll.compute(target_euler[0], current_euler[0], dt)
        rate_y = self.pid_pitch.compute(target_euler[1], current_euler[1], dt)
        
        yaw_err = np.mod(target_euler[2] - current_euler[2] + np.pi, 2*np.pi) - np.pi
        rate_z = self.pid_yaw.kp * yaw_err
        rate_z = float(np.clip(rate_z, -self.pid_yaw.limit_out, self.pid_yaw.limit_out))
        
        return np.array([rate_x, rate_y, rate_z], dtype=np.float64)
