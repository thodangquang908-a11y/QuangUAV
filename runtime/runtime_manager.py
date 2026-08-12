import numpy as np
from controllers.pid_controller import PIDController
from controllers.lqr_controller import LQRController
from controllers.mpc_controller import MPCController

class RuntimeManager:
    """
    Quản lý Chuyển đổi Thuật toán Điều khiển An toàn (Safe Dynamic Hot-Swap).
    Tự động tương thích với cả 2 cách truyền tham số khởi tạo từ FlightController.
    """

    def __init__(self, controllers_dict: dict = None, default_active: str = "PID", default_controller_name: str = None):
        # Tự động tương thích tên tham số truyền vào từ flight_controller.py
        if default_controller_name is not None:
            default_active = default_controller_name

        # Tự động nạp danh sách 3 bộ điều khiển nếu controllers_dict bị bỏ trống
        if controllers_dict is None:
            controllers_dict = {
                "PID": PIDController(mass=1.2),
                "LQR": LQRController(mass=1.2),
                "MPC": MPCController(mass=1.2)
            }

        self.controllers = controllers_dict
        self.active_name = default_active
        self.active_controller = self.controllers[default_active]

        # Trạng thái Hot-Swap
        self.is_transitioning = False
        self.previous_controller = None
        self.transition_start_time = 0.0
        self.transition_duration = 0.200  # Cửa sổ chuyển giao 200ms

    def request_hot_swap(self, target_name: str, sim_time: float) -> bool:
        if target_name not in self.controllers:
            print(f"❌ [HOT-SWAP FAILED] Controller '{target_name}' not found.")
            return False

        if target_name == self.active_name:
            return True

        self.previous_controller = self.active_controller
        self.active_name = target_name
        self.active_controller = self.controllers[target_name]
        
        # Reset trạng thái bộ điều khiển mới
        self.active_controller.reset()

        self.is_transitioning = True
        self.transition_start_time = sim_time

        print(f"🔥 [RUNTIME HOT-SWAP] Transitioning -> [{target_name}] at Sim Time: {sim_time:.2f}s")
        return True

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float, sim_time: float = 0.0) -> np.ndarray:
        u_active = self.active_controller.compute_control_signals(target_state, current_state, dt)

        if not self.is_transitioning:
            return u_active

        elapsed = sim_time - self.transition_start_time
        alpha = elapsed / self.transition_duration

        if alpha >= 1.0:
            self.is_transitioning = False
            self.previous_controller = None
            print(f"✅ [HOT-SWAP COMPLETE] Fully switched to [{self.active_name}]")
            return u_active

        alpha = np.clip(alpha, 0.0, 1.0)
        u_prev = self.previous_controller.compute_control_signals(target_state, current_state, dt)

        # Nội suy tuyến tính tín hiệu điều khiển Cross-Fading
        return (1.0 - alpha) * u_prev + alpha * u_active

    def get_telemetry_status(self) -> dict:
        return {
            "active_controller": self.active_name,
            "is_transitioning": self.is_transitioning
        }
