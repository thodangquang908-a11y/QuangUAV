import time
from typing import List, Dict, Callable, Any
from core.exceptions import TaskOverrunError
from core.logger import Logger

class Scheduler:
    """Bộ lập lịch thời gian thực có tính năng đo lường hiệu năng và phát hiện Overrun."""
    def __init__(self, strict_overrun: bool = False) -> None:
        self._tasks: List[Dict[str, Any]] = []
        self._strict_overrun = strict_overrun
        self._log = Logger.get_logger("Scheduler")

    def add_task(self, name: str, interval: float, callback: Callable[[], None]) -> None:
        self._tasks.append({
            "name": name,
            "interval": interval,
            "callback": callback,
            "next_run": 0.0,
            "max_exec_time": 0.0
        })
        self._log.debug(f"Đã thêm tác vụ '{name}' với chu kỳ {interval*1000:.1f}ms")

    def run(self, current_sim_time: float) -> None:
        for task in self._tasks:
            if current_sim_time >= task["next_run"]:
                t_start = time.perf_counter()
                
                task["callback"]()
                
                exec_time = time.perf_counter() - t_start
                if exec_time > task["max_exec_time"]:
                    task["max_exec_time"] = exec_time

                if exec_time > task["interval"]:
                    msg = f"Tác vụ '{task['name']}' vi phạm Overrun! Chu kỳ: {task['interval']*1000:.1f}ms, Thời gian chạy: {exec_time*1000:.2f}ms"
                    if self._strict_overrun:
                        raise TaskOverrunError(msg)
                    else:
                        self._log.warning(msg)

                task["next_run"] = current_sim_time + task["interval"]

    def get_profiling_report(self) -> Dict[str, float]:
        """Kết xuất báo cáo thời gian chạy đỉnh (Peak Execution Time) của các tác vụ."""
        return {t["name"]: t["max_exec_time"] * 1000 for t in self._tasks}
