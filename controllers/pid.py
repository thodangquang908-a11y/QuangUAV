import numpy as np

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, limit_out: float = 1.0, 
                 limit_int: float = 0.5, lpf_cutoff: float = 30.0) -> None:
        self.kp = kp; self.ki = ki; self.kd = kd
        self.limit_out = limit_out; self.limit_int = limit_int; self.lpf_cutoff = lpf_cutoff
        self.integral: float = 0.0; self.prev_error: float = 0.0; self.d_filtered: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0; self.prev_error = 0.0; self.d_filtered = 0.0

    def compute(self, target: float, current: float, dt: float, feedforward: float = 0.0) -> float:
        error = target - current
        P = self.kp * error
        
        if dt > 0.0 and self.kd > 0.0:
            d_raw = (error - self.prev_error) / dt
            rc = 1.0 / (2.0 * np.pi * self.lpf_cutoff)
            alpha = dt / (rc + dt)
            self.d_filtered = alpha * d_raw + (1.0 - alpha) * self.d_filtered
        else:
            self.d_filtered = 0.0
        D = self.kd * self.d_filtered
        self.prev_error = error
        
        output_unsat = P + (self.ki * self.integral) + D + feedforward
        is_saturated = abs(output_unsat) >= self.limit_out
        
        if not is_saturated or (output_unsat > 0 and error < 0) or (output_unsat < 0 and error > 0):
            self.integral = float(np.clip(self.integral + error * dt, -self.limit_int, self.limit_int))
            
        I = self.ki * self.integral
        output = P + I + D + feedforward
        return float(np.clip(output, -self.limit_out, self.limit_out))
