import numpy as np
from interfaces.i_sensor import ISensor

class IMUSensor(ISensor):
    """Giả lập cảm biến IMU 6-DOF: Gyroscope (Nhiễu + Random Walk Bias) & Accelerometer."""
    def __init__(self, gyro_std: float = 0.005, gyro_rw: float = 0.0001, accel_std: float = 0.05) -> None:
        self.g_std = gyro_std
        self.rw = gyro_rw
        self.a_std = accel_std
        self.gyro_bias = np.zeros(3, dtype=np.float64)

    def update(self, true_state: dict, dt: float) -> dict:
        self.gyro_bias += np.random.normal(0.0, self.rw * np.sqrt(dt), 3)
        true_gyro = true_state["angular_rate"]
        true_accel = true_state.get("acceleration", np.array([0.0, 0.0, -9.81]))
        
        noisy_gyro  = true_gyro + self.gyro_bias + np.random.normal(0.0, self.g_std, 3)
        noisy_accel = true_accel + np.random.normal(0.0, self.a_std, 3)
        
        return {
            "gyro": noisy_gyro,
            "accel": noisy_accel,
            "gyro_bias_truth": self.gyro_bias.copy()
        }
