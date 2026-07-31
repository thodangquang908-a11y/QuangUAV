import numpy as np

class Quaternion:
    """Toán học Quaternion [q0, q1, q2, q3] xử lý xoay không gian 3D."""
    
    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy
        return np.array([q0, q1, q2, q3], dtype=np.float64)

    @staticmethod
    def to_euler(q: np.ndarray) -> np.ndarray:
        q0, q1, q2, q3 = q
        
        # Roll (Trục X)
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (Trục Y)
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if np.abs(sinp) >= 1.0:
            pitch = np.copysign(np.pi / 2.0, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (Trục Z)
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=np.float64)

    @staticmethod
    def to_dcm(q: np.ndarray) -> np.ndarray:
        """Ma trận xoay DCM (Body Frame sang NED Frame)."""
        q0, q1, q2, q3 = q
        return np.array([
            [1 - 2*(q2**2 + q3**2),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
            [    2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2),     2*(q2*q3 - q0*q1)],
            [    2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
        ], dtype=np.float64)
