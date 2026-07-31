from abc import ABC, abstractmethod
import numpy as np

class IController(ABC):
    """Interface bắt buộc cho mọi thuật toán điều khiển (PID, LQR, MPC)."""
    @abstractmethod
    def reset(self) -> None:
        pass

    @abstractmethod
    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        pass
