import time
import numpy as np
from PyQt5.QtCore import QThread
from interfaces.shared_state import shared_bus

class SimulationThread(QThread):
    """Luồng tính toán mô phỏng 400Hz độc lập ghi dữ liệu vào Shared Bus."""
    def __init__(self, flight_controller, physics_engine) -> None:
        super().__init__()
        self.fc = flight_controller
        self.physics = physics_engine
        self.running = True
        self.dt = 0.0025 # 400Hz Loop

    def run(self) -> None:
        sim_time = 0.0
        while self.running:
            if not shared_bus.paused:
                t_start = time.time()
                
                # 1. Physics State
                true_state = self.physics.get_state_dict()
                
                # 2. Flight Controller Update
                act_thrust, torques, pwms = self.fc.update_sensors_and_estimate(
                    true_state, {}, sim_time
                )
                
                # 3. Physics Step
                self.physics.step(self.dt, act_thrust, torques)
                sim_time += self.dt

                # 4. Write Shared Bus Snapshot
                loop_time = (time.time() - t_start) * 1000.0
                shared_bus.update_from_sim(true_state, self.fc, sim_time, act_thrust, torques, pwms, loop_time)

                # Maintain 400Hz Rate
                elapsed = time.time() - t_start
                time.sleep(max(0.0001, self.dt - elapsed))
            else:
                time.sleep(0.02)

    def stop(self) -> None:
        self.running = False
        self.wait()
