import numpy as np
from typing import Callable

class RK4Integrator:
    """Bộ giải vi phân số Runge-Kutta bậc 4 (RK4)."""
    
    @staticmethod
    def step(func: Callable[[float, np.ndarray, np.ndarray], np.ndarray], 
             t: float, y: np.ndarray, dt: float, u: np.ndarray) -> np.ndarray:
        k1 = func(t, y, u)
        k2 = func(t + 0.5 * dt, y + 0.5 * dt * k1, u)
        k3 = func(t + 0.5 * dt, y + 0.5 * dt * k2, u)
        k4 = func(t + dt, y + dt * k3, u)
        
        return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
