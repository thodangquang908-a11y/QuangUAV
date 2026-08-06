import numpy as np
from scipy.linalg import solve_continuous_are
from interfaces.i_controller import IController

class LQRController(IController):
    """
    LEVEL 4: STABLE 12-STATE LQR CONTROLLER (BRYSON'S RULE LINEARIZATION)
    - Hệ tọa độ: NED Frame (Z âm hướng xuống)
    - Tuyến tính hóa quanh điểm cân bằng Hover (f0 = m*g = 11.772 N)
    """
    def __init__(self, mass: float = 1.2, g: float = 9.81):
        super().__init__()
        self.mass = mass
        self.g = g
        self.hover_thrust = mass * g # 11.772 N

        # 1. Ma trận Trạng thái A (12x12) tuyến tính hóa trong NED Frame
        # State x = [X, Y, Z, Vx, Vy, Vz, Roll, Pitch, Yaw, p, q, r]
        A = np.zeros((12, 12))
        A[0:3, 3:6] = np.eye(3)
        
        # Động lực học vị trí trong NED: 
        # ax = -g * theta (Pitch dương nghiêng đầu lên -> lùi X)
        # ay =  g * phi   (Roll dương nghiêng phải -> tiến Y)
        A[3, 7] = -g
        A[4, 6] = g
        A[6:9, 9:12] = np.eye(3)

        # 2. Ma trận Đầu vào B (12x4)
        # Input u = [delta_thrust, torque_roll, torque_pitch, torque_yaw]
        # Chú ý: Trong NED, az_ned = -delta_thrust / mass
        B = np.zeros((12, 4))
        B[5, 0] = -1.0 / mass # Tăng lực đẩy -> az âm (bay lên cao)
        B[9, 1] = 1.0 / 0.015 # Ixx = 0.015 kg.m2
        B[10, 2] = 1.0 / 0.015 # Iyy = 0.015 kg.m2
        B[11, 3] = 1.0 / 0.025 # Izz = 0.025 kg.m2

        # 3. Tính toán Q và R theo Quy tắc Bryson (Bryson's Rule)
        # Q_ii = 1 / (max_error_cho_phep)^2
        # R_jj = 1 / (max_control_input)^2
        
        # Giới hạn cho phép:
        # X, Y max err = 1.5m -> 1/(1.5^2) = 0.44
        # Z max err = 0.5m    -> 1/(0.5^2) = 4.0
        # Roll, Pitch max err = 0.15 rad (~8.5 deg) -> 1/(0.15^2) = 44.4
        Q_diag = [
            0.5, 0.5, 8.0,      # Pos X, Y, Z (Phạt nặng lệch Z)
            1.0, 1.0, 3.0,      # Vel X, Y, Vz
            50.0, 50.0, 10.0,   # Roll, Pitch, Yaw (Phạt rất nặng lệch góc nghiêng!)
            2.0, 2.0, 1.0       # Rates p, q, r
        ]
        Q = np.diag(Q_diag)

        # R: Phạt chi phí điều khiển
        # Thrust delta max = 4 N -> 1/(4^2) = 0.0625
        # Torques max = 0.1 N.m  -> 1/(0.1^2) = 100.0
        R_diag = [0.05, 50.0, 50.0, 20.0]
        R = np.diag(R_diag)

        # 4. Giải phương trình CARE để tìm ma trận K
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P

        print(f"✅ [LQRController] Initialized CARE Gain K Matrix {self.K.shape} successfully.")

    def reset(self) -> None:
        """Triển khai bắt buộc giao diện IController"""
        pass

    def sync_state_from_previous(self, prev_controller):
        """Warm Start: Đồng bộ tích lũy trạng thái khi Hot-Swap từ PID"""
        pass

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        # 1. Tính vector sai lệch trạng thái e = x - x_ref
        error_state = current_state - target_state

        # Khống chế sai lệch vị trí X, Y tầm ngắn để không kích hoạt góc nghiêng quá gắt
        error_state[0] = np.clip(error_state[0], -1.5, 1.5)
        error_state[1] = np.clip(error_state[1], -1.5, 1.5)

        # 2. Luật phản hồi LQR: u = -K * e
        u_lqr = -self.K @ error_state

        # 3. Lực đẩy tổng cộng (Offset thêm Lực nâng Hover m*g)
        delta_thrust = u_lqr[0]
        total_thrust = self.hover_thrust + delta_thrust

        # 4. KHỐNG CHẾ SATURATION GUARD (BẢO VỆ ĐỘNG LỰC HỌC)
        # Giới hạn lực đẩy an toàn trong khoảng [6N .. 18N]
        total_thrust = np.clip(total_thrust, 6.0, 18.0)

        # Giới hạn Momen góc để Pitch/Roll không bị lộn vòng quá 12 độ
        torque_roll = np.clip(u_lqr[1], -0.12, 0.12)
        torque_pitch = np.clip(u_lqr[2], -0.12, 0.12)
        torque_yaw = np.clip(u_lqr[3], -0.15, 0.15)

        return np.array([total_thrust, torque_roll, torque_pitch, torque_yaw], dtype=np.float64)
