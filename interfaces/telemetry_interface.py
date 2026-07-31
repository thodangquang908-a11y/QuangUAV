import numpy as np

class TelemetryData:
    """Central State Bus Data Structure: Dùng chung cho cả 3 cửa sổ GUI."""
    def __init__(self) -> None:
        self.sim_time = 0.0
        self.position = np.zeros(3, dtype=np.float64)      # True Position NED (m)
        self.velocity = np.zeros(3, dtype=np.float64)      # True Velocity NED (m/s)
        self.orientation = np.zeros(3, dtype=np.float64)   # Roll, Pitch, Yaw (rad)
        self.angular_rate = np.zeros(3, dtype=np.float64)  # p, q, r (rad/s)
        
        self.desired_pos = np.zeros(3, dtype=np.float64)   # Desired S-Curve Pos (m)
        self.target_pos = np.zeros(3, dtype=np.float64)    # Target Waypoint (m)
        
        self.pwms = np.ones(4, dtype=np.float64) * 1000.0  # Motor PWMs [M1..M4]
        self.actual_thrust = 0.0                           # Actual Thrust Force (N)
        self.torques = np.zeros(3, dtype=np.float64)       # Torques [Roll, Pitch, Yaw]
        
        # Dữ liệu phục vụ Data Scope
        self.alt_error = 0.0
        self.pos_error = 0.0
        self.battery_level = 100.0
        self.mode = "POSITION_HOLD"
        self.loop_time_ms = 2.5
