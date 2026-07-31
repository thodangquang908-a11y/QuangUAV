import numpy as np
from collections import deque
from typing import Dict, Any

class SensorSuite:
    """Hệ thống mô phỏng cảm biến chuẩn PX4 uORB: Chốt dữ liệu (Latching) kèm Timestamp chống lệch pha."""
    def __init__(self) -> None:
        self.imu_rate, self.baro_rate, self.mag_rate, self.gps_rate = 400.0, 50.0, 50.0, 10.0
        self.t_imu, self.t_baro, self.t_mag, self.t_gps = 0.0, 0.0, 0.0, 0.0
        
        self.gyro_bias = np.array([0.005, -0.003, 0.002], dtype=np.float64)
        self.accel_bias = np.array([0.02, -0.01, 0.01], dtype=np.float64)
        self.baro_drift = 0.0
        
        self.gps_delay_buffer: deque = deque(maxlen=int(0.1 / (1.0 / self.gps_rate)) + 1)
        
        # Bộ nhớ chốt dữ liệu (Zero-Order Hold Registers)
        self.last_imu = {"valid": False, "gyro": np.zeros(3), "gyr": np.zeros(3), "accel": np.array([0.0, 0.0, -9.81]), "acc": np.array([0.0, 0.0, -9.81]), "dt": 0.0025, "timestamp": 0.0}
        self.last_baro = {"valid": False, "altitude": 0.0, "alt": 0.0, "z": 0.0, "timestamp": 0.0}
        self.last_mag = {"valid": False, "yaw": 0.0, "heading": 0.0, "timestamp": 0.0}
        self.last_gps = {"valid": False, "position": np.zeros(3), "pos": np.zeros(3), "velocity": np.zeros(3), "vel": np.zeros(3), "timestamp": 0.0}

    def step(self, true_state: Dict[str, np.ndarray], dt: float, sim_time: float) -> Dict[str, Any]:
        pos = true_state.get("position", true_state.get("pos", np.zeros(3)))
        vel = true_state.get("velocity", true_state.get("vel", np.zeros(3)))
        euler = true_state.get("orientation", np.zeros(3))
        rates = true_state.get("angular_rate", np.zeros(3))
        
        # 1. IMU (400Hz)
        imu_data = self.last_imu.copy(); imu_data["valid"] = False; imu_data["dt"] = dt
        if sim_time - self.t_imu >= (1.0 / self.imu_rate) - 1e-5:
            self.t_imu = sim_time
            self.gyro_bias += np.random.normal(0.0, 1e-5, 3) * dt
            self.accel_bias += np.random.normal(0.0, 5e-4, 3) * dt
            gyro_noisy = rates + self.gyro_bias + np.random.normal(0.0, 0.005, 3)
            accel_noisy = np.array([0.0, 0.0, -9.81]) + self.accel_bias + np.random.normal(0.0, 0.05, 3)
            imu_data = {"valid": True, "gyro": gyro_noisy, "gyr": gyro_noisy, "accel": accel_noisy, "acc": accel_noisy, "dt": 1.0 / self.imu_rate, "timestamp": sim_time}
            self.last_imu = imu_data.copy()

        # 2. Barometer (50Hz)
        baro_data = self.last_baro.copy(); baro_data["valid"] = False
        if sim_time - self.t_baro >= (1.0 / self.baro_rate) - 1e-5:
            self.t_baro = sim_time
            self.baro_drift += np.random.normal(0.0, 0.001) * dt
            z_noisy = pos[2] + self.baro_drift + np.random.normal(0.0, 0.10)
            baro_data = {"valid": True, "altitude": z_noisy, "alt": z_noisy, "z": z_noisy, "timestamp": sim_time}
            self.last_baro = baro_data.copy()

        # 3. Magnetometer (50Hz)
        mag_data = self.last_mag.copy(); mag_data["valid"] = False
        if sim_time - self.t_mag >= (1.0 / self.mag_rate) - 1e-5:
            self.t_mag = sim_time
            yaw_noisy = euler[2] + np.random.normal(0.0, 0.02)
            mag_data = {"valid": True, "yaw": yaw_noisy, "heading": yaw_noisy, "timestamp": sim_time}
            self.last_mag = mag_data.copy()

        # 4. GPS (10Hz kèm trễ 100ms FIFO)
        gps_data = self.last_gps.copy(); gps_data["valid"] = False
        if sim_time - self.t_gps >= (1.0 / self.gps_rate) - 1e-5:
            self.t_gps = sim_time
            pos_noisy = pos + np.random.normal(0.0, 0.30, 3)
            vel_noisy = vel + np.random.normal(0.0, 0.05, 3)
            pkt = {"valid": True, "position": pos_noisy, "pos": pos_noisy, "velocity": vel_noisy, "vel": vel_noisy, "timestamp": sim_time}
            self.gps_delay_buffer.append(pkt)
            if len(self.gps_delay_buffer) >= self.gps_delay_buffer.maxlen:
                self.last_gps = self.gps_delay_buffer.popleft()
                gps_data = self.last_gps.copy()

        return {"imu": imu_data, "baro": baro_data, "mag": mag_data, "gps": gps_data}
