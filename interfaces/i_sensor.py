from abc import ABC, abstractmethod
import numpy as np

class ISensor(ABC):
    """Interface bắt buộc cho mọi loại cảm biến (IMU, GPS, Baro)."""
    @abstractmethod
    def update(self, true_state: np.ndarray, dt: float) -> np.ndarray:
        pass
