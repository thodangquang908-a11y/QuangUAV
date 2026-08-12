import numpy as np
from interfaces.i_controller import IController

class PIDController(IController):
    def __init__(self, mass: float = 1.2, g: float = 9.81):
        super().__init__()
        self.mass = mass
        self.g = g
        self.hover_thrust = mass * g
        self.integral_pos = np.zeros(3)

    def reset(self) -> None:
        self.integral_pos = np.zeros(3)

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        pos, target_pos = current_state[0:3], target_state[0:3]
        vel, target_vel = current_state[3:6], target_state[3:6]
        att, target_att = current_state[6:9], target_state[6:9]
        omega = current_state[9:12]

        err_pos = target_pos - pos
        
        self.integral_pos += err_pos * dt
        self.integral_pos[0] = np.clip(self.integral_pos[0], -2.0, 2.0)
        self.integral_pos[1] = np.clip(self.integral_pos[1], -2.0, 2.0)
        self.integral_pos[2] = np.clip(self.integral_pos[2], -10.0, 10.0)

        # Tăng Ki_z = 2.0 để PID triệt tiêu hoàn toàn 20cm sai số trước 8s
        kp_pos = np.array([1.8, 1.8, 5.0])
        ki_pos = np.array([0.1, 0.1, 2.0])
        kd_pos = np.array([2.2, 2.2, 4.2])

        acc_cmd = kp_pos * err_pos + ki_pos * self.integral_pos + kd_pos * (target_vel - vel)

        total_thrust = self.mass * (self.g - acc_cmd[2])
        total_thrust = np.clip(total_thrust, 4.0, 22.0)

        psi = att[2]
        acc_x_body = acc_cmd[0] * np.cos(psi) + acc_cmd[1] * np.sin(psi)
        acc_y_body = -acc_cmd[0] * np.sin(psi) + acc_cmd[1] * np.cos(psi)

        pitch_cmd = np.clip(-acc_x_body / self.g, -0.35, 0.35)
        roll_cmd = np.clip(acc_y_body / self.g, -0.35, 0.35)

        kp_att = np.array([8.0, 8.0, 4.0])
        kd_att = np.array([0.8, 0.8, 0.5])

        err_att = np.array([roll_cmd - att[0], pitch_cmd - att[1], target_att[2] - att[2]])
        err_att[2] = (err_att[2] + np.pi) % (2 * np.pi) - np.pi

        torques = kp_att * err_att - kd_att * omega

        return np.array([total_thrust, 
                         np.clip(torques[0], -0.3, 0.3), 
                         np.clip(torques[1], -0.3, 0.3), 
                         np.clip(torques[2], -0.3, 0.3)])
