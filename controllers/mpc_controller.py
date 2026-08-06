import numpy as np
from interfaces.i_controller import IController

class MPCController(IController):
    def __init__(self, horizon: int = 10):
        super().__init__()
        self.N = horizon
        self.mass = 1.2
        self.g = 9.81
        self.u_hover = self.mass * self.g
        print(f"[MPCController] Initialized MPC Controller Framework (Horizon N={self.N}).")

    def reset(self) -> None:
        pass

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        # Tạm thời trả về tín hiệu Hover an toàn cho MPC Framework
        return np.array([self.u_hover, 0.0, 0.0, 0.0], dtype=np.float64)
