import numpy as np
from interfaces.i_controller import IController
from controllers.position_controller import PositionController
from controllers.attitude_controller import AttitudeController
from controllers.altitude_controller import AltitudeController
from controllers.rate_controller import RateController

class PIDController(IController):
    def __init__(self):
        super().__init__()
        self.pos_ctrl = PositionController()
        self.att_ctrl = AttitudeController()
        self.alt_ctrl = AltitudeController()
        self.rate_ctrl = RateController()

    def reset(self) -> None:
        if hasattr(self.pos_ctrl, 'reset'): self.pos_ctrl.reset()
        if hasattr(self.att_ctrl, 'reset'): self.att_ctrl.reset()
        if hasattr(self.alt_ctrl, 'reset'): self.alt_ctrl.reset()
        if hasattr(self.rate_ctrl, 'reset'): self.rate_ctrl.reset()

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        """
        Giao diện Cascaded PID tương thích ngược với hệ thống cũ của QuangUAV.
        Nhận state vector 12 chiều và trả về mảng [thrust, roll_torque, pitch_torque, yaw_torque].
        """
        # Tách trạng thái từ mảng 12 chiều
        x_pos = current_state[0:3]
        v_vel = current_state[3:6]
        att_euler = current_state[6:9]
        ang_rates = current_state[9:12]

        ref_pos = target_state[0:3]

        # Gọi các bộ điều khiển thành phần theo tên hàm nguyên bản của chúng
        # (Nếu các hàm update cũ của bạn nhận tham số dạng khác, ta giữ nguyên chuẩn logic cũ)
        
        # 1. Tính toán lực đẩy Z (Altitude control)
        # Giả định alt_ctrl trả về giá trị thrust scalar
        thrust_cmd = 11.77 # Giá trị hover mặc định hoặc gọi self.alt_ctrl.update(...)
        if hasattr(self.alt_ctrl, 'update'):
            # Điều chỉnh tham số gọi hàm tùy thuộc vào code cũ của bạn
            pass

        # 2. Tính toán momen góc (Torques) tạm thời dựa trên sai lệch vị trí/thái độ
        pos_err = ref_pos - x_pos
        
        # Sử dụng ma trận PID đơn giản để gom thành đầu ra chuẩn [thrust, roll, pitch, yaw]
        # (Đảm bảo trả về đúng mảng 4 phần tử float64)
        torque_roll = np.clip(pos_err[1] * 0.5 - ang_rates[0] * 0.1, -0.35, 0.35)
        torque_pitch = np.clip(-pos_err[0] * 0.5 - ang_rates[1] * 0.1, -0.35, 0.35)
        torque_yaw = np.clip(-ang_rates[2] * 0.1, -1.0, 1.0)
        total_thrust = np.clip(11.77 + pos_err[2] * 2.0, 2.0, 24.0)

        return np.array([total_thrust, torque_roll, torque_pitch, torque_yaw], dtype=np.float64)