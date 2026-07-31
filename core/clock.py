import time

class Clock:
    """Bộ đồng bộ thời gian thực cho hệ thống mô phỏng."""
    def __init__(self, dt: float = 0.005) -> None:
        self.dt = dt
        self.sim_time: float = 0.0
        self._start_perf: float = 0.0

    def start(self) -> None:
        self.sim_time = 0.0
        self._start_perf = time.perf_counter()

    def tick(self) -> None:
        self.sim_time += self.dt

    def sleep_until_next_tick(self) -> None:
        expected_time = self.sim_time
        actual_time = time.perf_counter() - self._start_perf
        if actual_time < expected_time:
            time.sleep(expected_time - actual_time)
