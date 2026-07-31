import numpy as np
from interfaces.i_sensor import ISensor

class BaroSensor(ISensor):
    """Giả lập cảm biến Barometer đo độ cao có nhiễu và trôi theo thời gian."""
    def __init__(self, alt_noise_std: float = 0.15, drift_rate: float = 0.005) -> None:
        self.std = alt_noise_std
        self.drift_rate = drift_rate
        self.drift_val = 0.0

    def update(self, true_state: dict, dt: float) -> dict:
        self.drift_val += np.random.normal(0.0, self.drift_rate * dt)
        true_z = true_state["position"][2]
        noisy_alt = -true_z + self.drift_val + np.random.normal(0.0, self.std)
        return {"altitude": noisy_alt, "valid": True}
