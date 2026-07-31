import pytest
import time
import numpy as np
from core.state_register import StateRegister
from core.event_bus import EventBus
from core.scheduler import Scheduler
from core.exceptions import StateRegisterError, TaskOverrunError

def test_state_register_thread_safety():
    state = StateRegister()
    pos = np.array([10.0, -5.0, 0.0])
    state.update("position", pos)
    
    retrieved = state.get("position")
    assert np.array_equal(pos, retrieved)
    
    retrieved[0] = 999.0
    assert state.get("position")[0] == 10.0

def test_state_register_invalid_key():
    state = StateRegister()
    with pytest.raises(StateRegisterError):
        state.get("invalid_sensor_key")

def test_event_bus_error_isolation():
    bus = EventBus()
    results = []

    def faulty_listener(data):
        raise ValueError("Cố tình tạo lỗi callback!")

    def good_listener(data):
        results.append(data)

    bus.subscribe("test_event", faulty_listener)
    bus.subscribe("test_event", good_listener)
    
    bus.publish("test_event", "Payload Data")
    assert results == ["Payload Data"]

def test_scheduler_overrun_detection():
    scheduler = Scheduler(strict_overrun=True)
    
    def slow_task():
        time.sleep(0.02)

    scheduler.add_task("slow_task", interval=0.01, callback=slow_task)
    
    with pytest.raises(TaskOverrunError):
        scheduler.run(current_sim_time=0.01)
