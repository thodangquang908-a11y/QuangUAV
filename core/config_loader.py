import os
import yaml
from typing import Any, Dict
from core.exceptions import ConfigurationError
from core.logger import Logger

class ConfigLoader:
    """Trình quản lý cấu hình YAML với khả năng kiểm chứng lỗi định dạng."""
    def __init__(self, config_dir: str = "configs") -> None:
        self.config_dir = config_dir
        self._configs: Dict[str, Dict[str, Any]] = {}
        self._log = Logger.get_logger("ConfigLoader")

    def load(self, filename: str) -> Dict[str, Any]:
        path = os.path.join(self.config_dir, filename)
        if not os.path.exists(path):
            raise ConfigurationError(f"File cấu hình không tồn tại: {path}")
        
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if not isinstance(data, dict):
                    raise ConfigurationError(f"Cấu trúc YAML trong '{filename}' không hợp lệ.")
                self._configs[filename] = data
                self._log.debug(f"Đã nạp thành công cấu hình: {filename}")
                return data
        except yaml.YAMLError as e:
            raise ConfigurationError(f"Lỗi cú pháp YAML tại file {filename}: {str(e)}")

    def get(self, filename: str, query_path: str, default: Any = None) -> Any:
        if filename not in self._configs:
            self.load(filename)
        
        keys = query_path.split('.')
        val: Any = self._configs[filename]
        try:
            for k in keys:
                val = val[k]
            return val
        except (KeyError, TypeError):
            if default is not None:
                return default
            raise ConfigurationError(f"Không tìm thấy từ khóa '{query_path}' trong '{filename}'")
