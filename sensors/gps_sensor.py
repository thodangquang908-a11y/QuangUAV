import numpy as np
from interfaces.i_sensor import ISensor

class GPSSensor(ISensor):
    """Giả lập GPS công nghiệp: Cập nhật chậm 10Hz + Nhiễu mặt phẳng XY/Z + Rớt gói tin."""
    def __init__(self, pos_noise_std: float = 0.5, update_rate: float = 10.0, dropout_prob: float = 0.02) -> None:
        self.std = pos_noise_std
        self.rate = update_rate
        self.dropout = dropout_prob
        self.interval = 1.0 / self.rate
        self.time_since_last_fix = 0.0
        self.last_fix = np.zeros(3, dtype=np.float64)
        self.valid_fix = False

    def update(self, true_state: dict, dt: float) -> dict:
        self.time_since_last_fix += dt
        if self.time_since_last_fix >= self.interval:
            self.time_since_last_fix = 0.0
            if np.random.random() < self.dropout:
                self.valid_fix = False
            else:
                true_pos = true_state["position"]
                noise = np.random.normal(0.0, self.std, 3)
                noise[2] *= 1.5
                self.last_fix = true_pos + noise
                self.valid_fix = True
        return {"position": self.last_fix.copy(), "valid": self.valid_fix}
