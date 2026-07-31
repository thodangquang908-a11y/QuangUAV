import numpy as np

class SensorModel:
    """Mô hình Cảm biến Thực tế: Giả lập Barometer Noise, GPS Delay/Bias & IMU Drift."""
    def __init__(self) -> None:
        self.baro_noise_std = 0.15 # Barometer altitude noise (15cm)
        self.gps_noise_std  = 0.30 # GPS position noise (30cm)
        self.imu_bias = np.array([0.02, -0.01, 0.05]) # Accelerometer bias

    def read_sensors(self, true_state: dict):
        pos = true_state["position"]
        vel = true_state["velocity"]

        # Barometer Altitude (Z + Noise + Baro Drift)
        baro_alt = pos[2] + np.random.normal(0, self.baro_noise_std)
        
        # GPS Position (XYZ + Noise + Bias)
        gps_pos = pos + np.random.normal(0, self.gps_noise_std, size=3) + self.imu_bias

        return {"baro_alt": baro_alt, "gps_pos": gps_pos, "raw_vel": vel}

class ExtendedKalmanFilter:
    """Extended Kalman Filter (EKF): Ước lượng trạng thái Position & Velocity từ Cảm biến."""
    def __init__(self) -> None:
        self.est_position = np.zeros(3, dtype=np.float64)
        self.est_velocity = np.zeros(3, dtype=np.float64)
        
        # Covariance Matrix P
        self.P = np.eye(6) * 0.1
        self.Q = np.eye(6) * 0.01 # Process Noise
        self.R = np.eye(4) * 0.1  # Measurement Noise

    def update(self, sensor_data: dict, dt: float):
        # 1. Predict Step
        self.est_position += self.est_velocity * dt
        
        # 2. Measurement Update (GPS + Baro)
        z_meas = np.array([
            sensor_data["gps_pos"][0],
            sensor_data["gps_pos"][1],
            sensor_data["baro_alt"],
            sensor_data["raw_vel"][2]
        ])

        # Innovation (Sai số đo đạc)
        z_pred = np.array([self.est_position[0], self.est_position[1], self.est_position[2], self.est_velocity[2]])
        innovation = z_meas - z_pred

        # Complementary Kalmar Filter Fusion
        self.est_position[0] += 0.15 * innovation[0]
        self.est_position[1] += 0.15 * innovation[1]
        self.est_position[2] += 0.20 * innovation[2]
        self.est_velocity[2] += 0.20 * innovation[3]

        return self.est_position.copy(), self.est_velocity.copy(), np.linalg.norm(innovation)
