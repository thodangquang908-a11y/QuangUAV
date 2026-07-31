from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple

class IActuator(ABC):
    """Interface bắt buộc cho mô hình Động cơ và Bộ điều tốc ESC."""
    @abstractmethod
    def update_inputs(self, control_signals: np.ndarray, dt: float) -> None:
        pass

    @abstractmethod
    def get_forces_and_torques(self) -> Tuple[float, np.ndarray]:
        pass
