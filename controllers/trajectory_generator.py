import numpy as np

class JerkLimitedTrajectory:
    def __init__(self, max_vel: float = 3.0, max_acc: float = 2.0, max_jerk: float = 8.0) -> None:
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_jerk = max_jerk
        self.pos = 0.0; self.vel = 0.0; self.acc = 0.0

    def reset(self, current_pos: float) -> None:
        self.pos = current_pos; self.vel = 0.0; self.acc = 0.0

    def step(self, target_pos: float, dt: float) -> tuple[float, float, float]:
        pos_err = target_pos - self.pos
        kp_t, kv_t, ka_t = 2.5, 3.5, 1.8
        jerk_cmd = kp_t * pos_err - kv_t * self.vel - ka_t * self.acc
        jerk_cmd = float(np.clip(jerk_cmd, -self.max_jerk, self.max_jerk))
        
        self.acc = float(np.clip(self.acc + jerk_cmd * dt, -self.max_acc, self.max_acc))
        self.vel = float(np.clip(self.vel + self.acc * dt, -self.max_vel, self.max_vel))
        self.pos += self.vel * dt
        
        if abs(pos_err) < 0.01 and abs(self.vel) < 0.02:
            self.pos = target_pos; self.vel = 0.0; self.acc = 0.0
        return self.pos, self.vel, self.acc
