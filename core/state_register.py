import threading
import numpy as np
from typing import Dict
from core.exceptions import StateRegisterError

class StateRegister:
    """Kho dữ liệu trạng thái tập trung, bảo vệ bằng Mutex RLock."""
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state: Dict[str, np.ndarray] = {
            "position": np.zeros(3, dtype=np.float64),
            "velocity": np.zeros(3, dtype=np.float64),
            "orientation": np.zeros(3, dtype=np.float64),
            "angular_rate": np.zeros(3, dtype=np.float64),
            "actuators": np.zeros(4, dtype=np.float64)
        }

    def update(self, key: str, value: np.ndarray) -> None:
        with self._lock:
            if key not in self._state:
                raise StateRegisterError(f"Trường trạng thái '{key}' chưa được đăng ký.")
            try:
                self._state[key] = np.array(value, dtype=np.float64).copy()
            except Exception as e:
                raise StateRegisterError(f"Lỗi định dạng dữ liệu khi cập nhật '{key}': {str(e)}")

    def get(self, key: str) -> np.ndarray:
        with self._lock:
            if key not in self._state:
                raise StateRegisterError(f"Trường trạng thái '{key}' không tồn tại.")
            return self._state[key].copy()

    def snapshot(self) -> Dict[str, np.ndarray]:
        with self._lock:
            return {k: v.copy() for k, v in self._state.items()}
