import numpy as np
from enum import Enum
from runtime.runtime_manager import RuntimeManager

class FlightMode(Enum):
    INIT = "INIT"
    READY = "READY"
    ARMED = "ARMED"
    TAKEOFF = "TAKEOFF"
    MISSION = "MISSION"
    LAND = "LAND"

class FlightController:
    """
    AUTOPILOT CORE HUB - SOFTWARE-DEFINED UAV ARCHITECTURE
    Ủy quyền hoàn toàn việc tính toán tín hiệu điều khiển cho RuntimeManager.
    """
    def __init__(self, mass: float = 1.2) -> None:
        self.mass = mass
        self.g = 9.81
        
        # 🟢 TRẠM ĐIỀU PHỐI RUNTIME TRUNG TÂM
        self.runtime = RuntimeManager(default_controller_name="PID")

        self.mode = FlightMode.READY
        self.target_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.takeoff_target_z = -10.0
        
        self.mission_start_time = 0.0
        self.traj_radius_x = 15.0
        self.traj_radius_y = 10.0
        self.omega = 0.25

    def arm_and_takeoff(self, target_z: float = -10.0) -> None:
        self.mode = FlightMode.TAKEOFF
        self.takeoff_target_z = target_z
        self.target_pos = np.array([0.0, 0.0, target_z], dtype=np.float64)

    def set_target(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        self.target_pos = np.array([x, y, z], dtype=np.float64)

    def update_sensors_and_estimate(self, true_state: dict, sensors: dict, sim_time: float):
        curr_pos = true_state["position"]
        curr_vel = true_state["velocity"]
        curr_att = true_state["orientation"]
        curr_rate = true_state["angular_rate"]

        pos_z = curr_pos[2]

        # 1. TẠO QUỸ ĐẠO BAY (TRAJECTORY GENERATOR)
        if self.mode == FlightMode.TAKEOFF:
            if abs(pos_z - self.takeoff_target_z) < 0.6:
                self.mode = FlightMode.MISSION
                self.mission_start_time = sim_time

        elif self.mode == FlightMode.MISSION:
            t = sim_time - self.mission_start_time
            w = self.omega
            des_x = self.traj_radius_x * np.sin(w * t)
            des_y = self.traj_radius_y * np.sin(2.0 * w * t)
            des_z = -10.0 - 3.0 * np.sin(0.5 * w * t)
            self.target_pos = np.array([des_x, des_y, des_z], dtype=np.float64)

        # 2. ĐÓNG GÓI STATE & TARGET STATE VECTOR (12-STATES)
        # Vector state: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        target_state_vec = np.zeros(12)
        target_state_vec[0:3] = self.target_pos

        current_state_vec = np.zeros(12)
        current_state_vec[0:3] = curr_pos
        current_state_vec[3:6] = curr_vel
        current_state_vec[6:9] = curr_att
        current_state_vec[9:12] = curr_rate

        # 🟢 3. MẠCH NỐI CHÍ MẠNG: ỦY QUYỀN TÍNH TOÁN CHO RUNTIMEMANAGER!
        # Tùy thuộc vào controller đang active (PID/LQR/MPC), Runtime sẽ tính toán u_control
        u_control = self.runtime.compute_control_signals(
            target_state=target_state_vec,
            current_state=current_state_vec,
            dt=0.0025,
            sim_time=sim_time
        )

        thrust_cmd = u_control[0]
        torques = u_control[1:4]
        torque_roll, torque_pitch, torque_yaw = torques[0], torques[1], torques[2]

        # 4. QUADCOPTER X MOTOR MIXER MATRIX
        f_base = thrust_cmd / 4.0
        f1 = f_base - torque_roll * 2.0 + torque_pitch * 2.0 - torque_yaw * 1.0
        f2 = f_base + torque_roll * 2.0 - torque_pitch * 2.0 - torque_yaw * 1.0
        f3 = f_base + torque_roll * 2.0 + torque_pitch * 2.0 + torque_yaw * 1.0
        f4 = f_base - torque_roll * 2.0 - torque_pitch * 2.0 + torque_yaw * 1.0

        motor_forces = np.array([f1, f2, f3, f4], dtype=np.float64)
        motor_forces = np.clip(motor_forces, 0.1, 8.0)

        pwms = 1000.0 + (motor_forces / 8.0) * 1000.0
        pwms = np.clip(pwms, 1000.0, 1950.0)

        return thrust_cmd, torques, pwms