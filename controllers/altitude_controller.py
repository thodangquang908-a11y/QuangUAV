import numpy as np
from controllers.pid import PIDController
from core.config_loader import ConfigLoader

class AltitudeController:
    def __init__(self, mass: float = 1.2) -> None:
        self.mass = mass
        self.hover_thrust = mass * 9.81
        cfg = ConfigLoader()
        pz = cfg.get("pid.yaml", "controllers.altitude.pos_z")
        vz = cfg.get("pid.yaml", "controllers.altitude.vel_z")
        self.pid_z_pos = PIDController(pz['kp'], 0.0, 0.0, pz['limit_out'])
        self.pid_z_vel = PIDController(vz['kp'], vz['ki'], vz['kd'], vz['limit_out'], vz['limit_int'], vz['lpf_cutoff'])

    def compute(self, target_z: float, current_z: float, current_vz: float, dt: float, 
                ff_vz: float = 0.0, ff_az: float = 0.0) -> float:
        target_h, current_h, current_vh = -target_z, -current_z, -current_vz
        target_vh = self.pid_z_pos.compute(target_h, current_h, dt) + (-ff_vz)
        accel_ff_thrust = self.mass * (-ff_az)
        thrust_adj = self.pid_z_vel.compute(target_vh, current_vh, dt, feedforward=accel_ff_thrust)
        
        total_thrust = self.hover_thrust + thrust_adj
        return float(np.clip(total_thrust, 0.0, 24.0))
