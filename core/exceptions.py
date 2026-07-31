"""Hệ thống ngoại lệ đặc thù của nền tảng QuangUAV."""

class QuangUAVError(Exception):
    """Lớp ngoại lệ cơ sở cho toàn bộ hệ thống."""
    pass

class ConfigurationError(QuangUAVError):
    """Lỗi liên quan đến nạp, phân tích hoặc sai định dạng YAML."""
    pass

class StateRegisterError(QuangUAVError):
    """Lỗi truy xuất hoặc ghi đè trạng thái hệ thống."""
    pass

class TaskOverrunError(QuangUAVError):
    """Lỗi vi phạm thời gian gian thực: Tác vụ vượt quá chu kỳ cho phép."""
    pass

class EventBusError(QuangUAVError):
    """Lỗi trong quá trình phân phối sự kiện giữa các module."""
    pass
