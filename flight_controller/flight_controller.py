import numpy as np
from enum import Enum

class FlightMode(Enum):
    INIT = "INIT"
    READY = "READY"
    ARMED = "ARMED"
    TAKEOFF = "TAKEOFF"
    MISSION = "MISSION"
    LAND = "LAND"

class FlightController:
    """Autopilot Core Hub: Bộ tạo Quỹ đạo Bay 3D Figure-8 (Lissajous Curve)."""
    def __init__(self, mass: float = 1.2) -> None:
        self.mass = mass
        self.g = 9.81
        self.hover_thrust = self.mass * self.g # ~11.77 N

        self.mode = FlightMode.READY
        self.target_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.takeoff_target_z = -10.0
        
        # Biến điều khiển Quỹ đạo Figure-8
        self.mission_start_time = 0.0
        self.traj_radius_x = 15.0 # Bán kính X (15m)
        self.traj_radius_y = 10.0 # Bán kính Y (10m)
        self.omega = 0.25         # Tốc độ góc (rad/s)

        # PID Gains
        self.kp_z = 1.5
        self.kd_z = 2.0
        self.kp_xy = 0.8
        self.kd_xy = 1.1
        self.kp_att = 4.0
        self.kd_rate = 0.18

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

        # 1. GENERATE FIGURE-8 TRAJECTORY WHEN REACHED TAKEOFF ALTITUDE
        if self.mode == FlightMode.TAKEOFF:
            if abs(pos_z - self.takeoff_target_z) < 0.6:
                self.mode = FlightMode.MISSION
                self.mission_start_time = sim_time

        elif self.mode == FlightMode.MISSION:
            t = sim_time - self.mission_start_time
            w = self.omega
            
            # Phương trình tham số 3D Figure-8
            des_x = self.traj_radius_x * np.sin(w * t)
            des_y = self.traj_radius_y * np.sin(2.0 * w * t)
            des_z = -10.0 - 3.0 * np.sin(0.5 * w * t) # Uốn lượn 7m đến 13m

            self.target_pos = np.array([des_x, des_y, des_z], dtype=np.float64)

        # 2. ALTITUDE POSITION & VELOCITY CONTROL
        pos_err_z = self.target_pos[2] - pos_z
        target_vel_z = np.clip(pos_err_z * self.kp_z, -2.5, 2.5)
        vel_err_z = target_vel_z - curr_vel[2]

        thrust_cmd = self.hover_thrust - vel_err_z * self.kd_z
        thrust_cmd = np.clip(thrust_cmd, 2.0, 24.0)

        # 3. HORIZONTAL POSITION & VELOCITY CONTROL
        pos_err_xy = self.target_pos[0:2] - curr_pos[0:2]
        target_vel_xy = np.clip(pos_err_xy * self.kp_xy, -3.0, 3.0)
        vel_err_xy = target_vel_xy - curr_vel[0:2]

        pitch_cmd = np.clip(-(vel_err_xy[0] * 0.15 + pos_err_xy[0] * 0.05), -0.35, 0.35)
        roll_cmd  = np.clip( (vel_err_xy[1] * 0.15 + pos_err_xy[1] * 0.05), -0.35, 0.35)

        # 4. ATTITUDE RATE PID CONTROL
        att_err_roll  = roll_cmd - curr_att[0]
        att_err_pitch = pitch_cmd - curr_att[1]

        torque_roll  = att_err_roll * self.kp_att - curr_rate[0] * self.kd_rate
        torque_pitch = att_err_pitch * self.kp_att - curr_rate[1] * self.kd_rate
        torque_yaw   = (0.0 - curr_att[2]) * 0.8 - curr_rate[2] * 0.1

        torques = np.array([torque_roll, torque_pitch, torque_yaw], dtype=np.float64)

        # 5. QUADCOPTER X MOTOR MIXER MATRIX
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
