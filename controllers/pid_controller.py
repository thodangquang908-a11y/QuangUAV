import numpy as np
from interfaces.i_controller import IController

class PIDController(IController):
    def __init__(self, mass: float = 1.2, g: float = 9.81):
        super().__init__()
        self.mass = mass
        self.g = g
        self.hover_thrust = mass * g
        self.integral_z = 0.0

    def reset(self) -> None:
        self.integral_z = 0.0

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        pos, target_pos = current_state[0:3], target_state[0:3]
        vel = current_state[3:6]
        att = current_state[6:9]
        omega = current_state[9:12]

        # Hệ NED: pos[2]=0 (đất), target_pos[2]=-10.0 (10m) -> err_z = +10.0
        err_z = pos[2] - target_pos[2]
        self.integral_z += err_z * dt
        self.integral_z = np.clip(self.integral_z, -2.0, 2.0)

        delta_thrust = 3.5 * err_z + 0.8 * self.integral_z - 2.0 * vel[2]
        total_thrust = np.clip(self.hover_thrust + delta_thrust, 5.0, 20.0)

        err_x = pos[0] - target_pos[0]
        err_y = pos[1] - target_pos[1]

        pitch_cmd = np.clip(0.08 * err_x + 0.15 * vel[0], -0.2, 0.2)
        roll_cmd = np.clip(-0.08 * err_y - 0.15 * vel[1], -0.2, 0.2)

        t_roll = -4.0 * (att[0] - roll_cmd) - 0.5 * omega[0]
        t_pitch = -4.0 * (att[1] - pitch_cmd) - 0.5 * omega[1]
        t_yaw = -2.0 * (att[2] - target_state[8]) - 0.3 * omega[2]

        return np.array([total_thrust, np.clip(t_roll, -0.2, 0.2), np.clip(t_pitch, -0.2, 0.2), np.clip(t_yaw, -0.2, 0.2)])
