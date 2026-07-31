import logging
import sys
from typing import Optional

class Logger:
    """Trình ghi log chuẩn R&D, hỗ trợ định dạng mili-giây và an toàn đa luồng."""
    _instance: Optional[logging.Logger] = None

    @classmethod
    def get_logger(cls, name: str = "QuangUAV") -> logging.Logger:
        if cls._instance is None:
            logger = logging.getLogger(name)
            logger.setLevel(logging.DEBUG)

            if not logger.handlers:
                formatter = logging.Formatter(
                    '[%(asctime)s.%(msecs)03d] [%(levelname)-7s] [%(filename)s:%(lineno)d] %(message)s',
                    datefmt='%H:%M:%S'
                )
                console_handler = logging.StreamHandler(sys.stdout)
                console_handler.setFormatter(formatter)
                logger.addHandler(console_handler)

            cls._instance = logger
        return cls._instance
