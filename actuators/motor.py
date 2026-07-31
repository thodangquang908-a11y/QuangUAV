import numpy as np

class ESC:
    def __init__(self, tau: float = 0.02) -> None:
        self.tau = tau
        self.current_rpm = 0.0

    def step(self, target_pwm: float, dt: float) -> float:
        # Biến đổi PWM (1000..2000) -> Target RPM (0..1200)
        target_rpm = max(0.0, (target_pwm - 1000.0) / 1000.0 * 1200.0)
        alpha = dt / (self.tau + dt)
        self.current_rpm += alpha * (target_rpm - self.current_rpm)
        return self.current_rpm

class Propeller:
    def __init__(self, ct: float = 1.0e-5, cq: float = 1.0e-7) -> None:
        self.ct = ct; self.cq = cq

    def compute_forces(self, rpm: float) -> tuple[float, float]:
        thrust = self.ct * (rpm ** 2)
        torque = self.cq * (rpm ** 2)
        return thrust, torque

class Motor:
    def __init__(self) -> None:
        self.esc = ESC()
        self.propeller = Propeller()

    def step(self, pwm: float, dt: float) -> float:
        rpm = self.esc.step(pwm, dt)
        return rpm
