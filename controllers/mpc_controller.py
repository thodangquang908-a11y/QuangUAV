import numpy as np
from interfaces.i_controller import IController

class MPCController(IController):
    """
    PROTOTYPE / STUB: Simplified Model Predictive Control Placeholder.
    
    [CẢNH BÁO KIẾN TRÚC]: 
    Tệp này hiện tại đóng vai trò khung Mẫu (Prototype Interface) phục vụ kiểm thử
    chức năng Runtime Hot-Swap. Thuật toán MPC thực tế với Mô hình Dự báo (Prediction Model),
    Cửa sổ Dự báo N (Horizon N), Ma trận Hằng số & Ràng buộc (Constraints) và Giải Bài toán
    Tối ưu Quadratic Programming (QP Solver) sẽ được triển khai ở Sprint tiếp theo.
    """

    def __init__(self, mass: float = 1.2, g: float = 9.81):
        super().__init__()
        self.mass = mass
        self.g = g
        self.hover_thrust = mass * g
        print("⚠️ [MPC STUB] Initialized as Runtime Placeholder Prototype.")

    def reset(self) -> None:
        pass

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        pos, target_pos = current_state[0:3], target_state[0:3]
        vel, target_vel = current_state[3:6], target_state[3:6]
        att, target_att = current_state[6:9], target_state[6:9]
        omega, target_omega = current_state[9:12], target_state[9:12]

        # Khung thuật toán xấp xỉ dùng phản hồi đơn giản cho Prototype
        err_z = pos[2] - target_pos[2]
        delta_thrust = 4.0 * err_z - 2.5 * vel[2]
        total_thrust = np.clip(self.hover_thrust + delta_thrust, 5.0, 20.0)

        err_x = pos[0] - target_pos[0]
        err_y = pos[1] - target_pos[1]

        pitch_cmd = np.clip(0.08 * err_x + 0.15 * vel[0], -0.2, 0.2)
        roll_cmd = np.clip(-0.08 * err_y - 0.15 * vel[1], -0.2, 0.2)

        t_roll = -4.0 * (att[0] - roll_cmd) - 0.5 * omega[0]
        t_pitch = -4.0 * (att[1] - pitch_cmd) - 0.5 * omega[1]
        t_yaw = -2.0 * (att[2] - target_att[2]) - 0.3 * omega[2]

        return np.array([total_thrust, np.clip(t_roll, -0.2, 0.2), np.clip(t_pitch, -0.2, 0.2), np.clip(t_yaw, -0.2, 0.2)])
