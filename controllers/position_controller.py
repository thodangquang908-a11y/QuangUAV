import numpy as np
from controllers.pid import PIDController
from core.config_loader import ConfigLoader

class PositionController:
    """Bộ điều khiển Vị trí chuẩn 22:49: Đã nâng trần góc nghiêng lên 25.0 độ."""
    def __init__(self) -> None:
        cfg = ConfigLoader()
        p = cfg.get("pid.yaml", "controllers.position.pos_xy")
        v = cfg.get("pid.yaml", "controllers.position.vel_xy")
        self.pid_x_pos = PIDController(p['kp'], 0.0, 0.0, p['limit_out'])
        self.pid_y_pos = PIDController(p['kp'], 0.0, 0.0, p['limit_out'])
        self.pid_x_vel = PIDController(v['kp'], v['ki'], v['kd'], v['limit_out'], v['limit_int'], v['lpf_cutoff'])
        self.pid_y_vel = PIDController(v['kp'], v['ki'], v['kd'], v['limit_out'], v['limit_int'], v['lpf_cutoff'])
        
        # [Yêu cầu a]: Tăng giới hạn nghiêng từ 18.0° lên 25.0°
        self.max_tilt_rad = np.radians(25.0)

    def compute(self, target_xy: np.ndarray, current_xy: np.ndarray, current_vel_xy: np.ndarray, 
                yaw: float, dt: float, ff_vel: np.ndarray = np.zeros(2), ff_acc: np.ndarray = np.zeros(2)) -> np.ndarray:
        target_vx = self.pid_x_pos.compute(target_xy[0], current_xy[0], dt) + ff_vel[0]
        target_vy = self.pid_y_pos.compute(target_xy[1], current_xy[1], dt) + ff_vel[1]
        
        acc_x = self.pid_x_vel.compute(target_vx, current_vel_xy[0], dt, feedforward=ff_acc[0])
        acc_y = self.pid_y_vel.compute(target_vy, current_vel_xy[1], dt, feedforward=ff_acc[1])
        
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        acc_x_body =  acc_x * cos_y + acc_y * sin_y
        acc_y_body = -acc_x * sin_y + acc_y * cos_y
        
        target_pitch = float(np.clip(-acc_x_body, -self.max_tilt_rad, self.max_tilt_rad))
        target_roll  = float(np.clip( acc_y_body, -self.max_tilt_rad, self.max_tilt_rad))
        return np.array([target_roll, target_pitch], dtype=np.float64)
