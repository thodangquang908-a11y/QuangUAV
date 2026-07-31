import numpy as np
from scipy.spatial.transform import Rotation as R

class RigidBody6DoF:
    """Mô hình động lực học Quadcopter 6DoF chuẩn xác (NED frame)."""
    def __init__(self, mass: float = 1.2, arm_length: float = 0.25):
        self.mass = mass
        self.g = 9.81
        self.arm_length = arm_length
        
        self.position = np.zeros(3)      # x, y, z (NED)
        self.velocity = np.zeros(3)      # vx, vy, vz
        self.orientation = np.zeros(3)   # roll, pitch, yaw (rad)
        self.angular_rate = np.zeros(3)  # p, q, r (rad/s)
        
        # Inertia matrix (Quadcopter X)
        self.I = np.diag([0.012, 0.012, 0.022])

    def reset(self, initial_pos=np.array([0.0, 0.0, 0.0])):
        self.position = initial_pos.copy()
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)
        self.angular_rate = np.zeros(3)

    def get_state_dict(self):
        return {
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "orientation": self.orientation.copy(),
            "angular_rate": self.angular_rate.copy()
        }

    def step(self, dt: float, total_thrust: float, body_torques: np.ndarray):
        if dt <= 0: return

        # 1. Translational Dynamics (Force)
        rot = R.from_euler('xyz', self.orientation, degrees=False)
        thrust_body = np.array([0.0, 0.0, -total_thrust])  # Thrust theo trục Z body
        thrust_world = rot.apply(thrust_body)
        
        force_world = thrust_world + np.array([0.0, 0.0, self.mass * self.g])
        acc = force_world / self.mass
        
        self.velocity += acc * dt
        self.position += self.velocity * dt

        # Va chạm mặt đất
        if self.position[2] >= 0.0:
            if total_thrust <= self.mass * self.g:
                self.position[2] = 0.0
                self.velocity = np.zeros(3)
                self.angular_rate = np.zeros(3)

        # 2. Rotational Dynamics (Torque)
        torque = body_torques.copy()
        ang_acc = np.linalg.solve(self.I, torque - np.cross(self.angular_rate, self.I @ self.angular_rate))
        
        self.angular_rate += ang_acc * dt
        self.orientation += self.angular_rate * dt
        
        # Normalize góc Euler
        self.orientation[2] = (self.orientation[2] + np.pi) % (2 * np.pi) - np.pi
        self.orientation[0] = np.clip(self.orientation[0], -1.4, 1.4)
        self.orientation[1] = np.clip(self.orientation[1], -1.4, 1.4)
