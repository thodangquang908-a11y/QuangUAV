from abc import ABC, abstractmethod
from typing import Dict, Any

class BaseController(ABC):
    def __init__(self, name: str):
        self.name = name
        self.is_initialized = False

    @abstractmethod
    def initialize(self, config: Dict[str, Any] = None) -> bool:
        """Khởi tạo tham số (Gains PID, Limits, Matrices...)."""
        pass

    @abstractmethod
    def update(self, state: Dict[str, Any], setpoint: Dict[str, Any], dt: float) -> Dict[str, float]:
        """
        Vòng lặp tính toán tín hiệu điều khiển.
        Trả về dict: {"thrust": float, "roll": float, "pitch": float, "yaw_rate": float}
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reset các trạng thái tích lũy (I_term, filters...)."""
        pass
