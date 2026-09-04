import numpy as np
from interfaces.i_controller import IController

class PIDController(IController):
    """
    Cascaded Angle-to-Rate PID Flight Controller.
    Triệt tiêu hoàn toàn dao động tự kích do trễ động cơ, khống chế góc nghiêng <= 8 độ.
    """
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

        # 1. Vòng Vị trí (Position Loop)
        err_pos = target_pos - pos
        self.integral_pos += err_pos * dt
        self.integral_pos = np.clip(self.integral_pos, -0.4, 0.4)

        kp_pos = np.array([1.0, 1.0, 3.2])
        ki_pos = np.array([0.02, 0.02, 0.5])
        kd_pos = np.array([1.6, 1.6, 2.8])

        acc_cmd = kp_pos * err_pos + ki_pos * self.integral_pos + kd_pos * (target_vel - vel)

        # 2. Tổng lực nâng Thrust
        total_thrust = np.clip(self.mass * (self.g - acc_cmd[2]), 2.0, 20.0)

        # 3. Tính góc nghiêng tham chiếu (Khống chế tối đa 8 độ = 0.14 rad để drone luôn phẳng)
        psi = att[2]
        acc_x_body =  acc_cmd[0] * np.cos(psi) + acc_cmd[1] * np.sin(psi)
        acc_y_body = -acc_cmd[0] * np.sin(psi) + acc_cmd[1] * np.cos(psi)

        pitch_des = np.clip(-acc_x_body / self.g, -0.14, 0.14)  # Max 8.0 deg
        roll_des  = np.clip( acc_y_body / self.g, -0.14, 0.14)

        # 4. Vòng Ngoài: Attitude Angle -> Desired Angular Rate (Vận tốc góc mong muốn)
        kp_att = np.array([3.5, 3.5, 2.0])
        err_att = np.array([roll_des - att[0], pitch_des - att[1], target_att[2] - att[2]])
        err_att[2] = (err_att[2] + np.pi) % (2 * np.pi) - np.pi
        rate_des = kp_att * err_att

        # 5. Vòng Trong: Rate Damping Control -> Torques (Dập tắt quán tính)
        kd_rate = np.array([0.08, 0.08, 0.05])
        torques = kd_rate * (rate_des - omega)

        return np.array([total_thrust, 
                         np.clip(torques[0], -0.06, 0.06), 
                         np.clip(torques[1], -0.06, 0.06), 
                         np.clip(torques[2], -0.03, 0.03)])
