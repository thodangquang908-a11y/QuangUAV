import threading
import traceback
from typing import Dict, List, Callable, Any
from core.logger import Logger

class EventBus:
    """Trục phân phối sự kiện bất đồng bộ, an toàn luồng và cô lập ngoại lệ."""
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._subscribers: Dict[str, List[Callable[[Any], None]]] = {}
        self._log = Logger.get_logger("EventBus")

    def subscribe(self, event_name: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            if event_name not in self._subscribers:
                self._subscribers[event_name] = []
            if callback not in self._subscribers[event_name]:
                self._subscribers[event_name].append(callback)
                self._log.debug(f"Đăng ký listener cho sự kiện '{event_name}'")

    def publish(self, event_name: str, data: Any = None) -> None:
        with self._lock:
            listeners = self._subscribers.get(event_name, []).copy()

        for callback in listeners:
            try:
                callback(data)
            except Exception as e:
                self._log.error(f"Lỗi thực thi callback trong sự kiện '{event_name}': {str(e)}")
                self._log.debug(traceback.format_exc())
