import threading
import numpy as np

class SharedStateBus:
    """Bộ đệm dữ liệu tập trung Thread-Safe cho GCS, Viewer & Scope."""
    def __init__(self) -> None:
        self._lock = threading.Lock()
        
        self.sim_time = 0.0
        self.position = np.zeros(3, dtype=np.float64)      # NED (m)
        self.velocity = np.zeros(3, dtype=np.float64)      # NED (m/s)
        self.orientation = np.zeros(3, dtype=np.float64)   # Roll, Pitch, Yaw (rad)
        self.angular_rate = np.zeros(3, dtype=np.float64)  # p, q, r (rad/s)
        
        self.desired_pos = np.zeros(3, dtype=np.float64)   # Target Position S-Curve
        self.target_pos = np.zeros(3, dtype=np.float64)    # Target Waypoint
        
        self.pwms = np.ones(4, dtype=np.float64) * 1000.0  # Motor PWMs
        self.actual_thrust = 0.0                           # Thrust Force (N)
        self.torques = np.zeros(3, dtype=np.float64)
        
        # System Health Telemetry
        self.battery_pct = 100.0
        self.gps_fix = "3D FIX (18 Sats)"
        self.ekf_status = "OK (GOOD)"
        self.cpu_load = 12.5                               # %
        self.mode = "POSITION_HOLD"
        self.loop_time_ms = 2.5
        self.paused = False

    def update_from_sim(self, true_state, fc, sim_time, act_thrust, torques, pwms, loop_time) -> None:
        with self._lock:
            self.sim_time = sim_time
            self.position[:] = true_state["position"]
            self.velocity[:] = true_state["velocity"]
            self.orientation[:] = true_state["orientation"]
            self.angular_rate[:] = true_state["angular_rate"]
            
            if hasattr(fc, 'target_pos'):
                self.desired_pos[:] = fc.target_pos
                self.target_pos[:] = fc.target_pos
                
            self.pwms[:] = pwms
            self.actual_thrust = act_thrust
            self.torques[:] = torques
            self.mode = fc.mode.value if hasattr(fc.mode, 'value') else str(fc.mode)
            self.loop_time_ms = loop_time
            
            # Giả lập hao pin & CPU Load
            self.battery_pct = max(0.0, 100.0 - (sim_time * 0.12))
            self.cpu_load = 12.0 + np.random.uniform(-1.0, 1.0)

    def get_snapshot(self) -> dict:
        with self._lock:
            return {
                "sim_time": self.sim_time,
                "position": self.position.copy(),
                "velocity": self.velocity.copy(),
                "orientation": self.orientation.copy(),
                "angular_rate": self.angular_rate.copy(),
                "desired_pos": self.desired_pos.copy(),
                "target_pos": self.target_pos.copy(),
                "pwms": self.pwms.copy(),
                "actual_thrust": self.actual_thrust,
                "torques": self.torques.copy(),
                "battery_pct": self.battery_pct,
                "gps_fix": self.gps_fix,
                "ekf_status": self.ekf_status,
                "cpu_load": self.cpu_load,
                "mode": self.mode,
                "loop_time_ms": self.loop_time_ms,
                "paused": self.paused
            }

shared_bus = SharedStateBus()
