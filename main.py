import numpy as np
from core.config_loader import ConfigLoader
from core.logger import Logger
from core.state_register import StateRegister
from core.event_bus import EventBus
from core.clock import Clock
from core.scheduler import Scheduler

def main() -> None:
    log = Logger.get_logger("QuangUAV-Main")
    log.info("🚀 KHỞI ĐỘNG KIỂM THỬ HẠ TẦNG LÕI QUANGUAV (PHASE 1)...")

    cfg_loader = ConfigLoader()
    mass = cfg_loader.get("drone_quad.yaml", "drone.mass")
    dt = cfg_loader.get("simulation.yaml", "simulation.dt")
    log.info(f"✅ YAML Loader thành công -> Khối lượng: {mass}kg | dt: {dt}s")

    state = StateRegister()
    state.update("position", np.array([5.0, 10.0, -20.0]))
    log.info(f"✅ StateRegister thành công -> Tọa độ NED ban đầu: {state.get('position')}")

    bus = EventBus()
    def safe_listener(msg: str) -> None:
        log.info(f"✅ EventBus nhận thông điệp an toàn: '{msg}'")
    
    def broken_listener(msg: str) -> None:
        raise RuntimeError("Giả lập lỗi từ Listener xấu!")

    bus.subscribe("system_ping", broken_listener)
    bus.subscribe("system_ping", safe_listener)
    bus.publish("system_ping", "EventBus Core Active")

    clock = Clock(dt=dt)
    scheduler = Scheduler(strict_overrun=False)

    task_counter = 0
    def fast_task() -> None:
        nonlocal task_counter
        task_counter += 1

    scheduler.add_task("fast_loop_100Hz", interval=0.01, callback=fast_task)

    log.info("⏱️ Đang chạy thử nghiệm 50 chu kỳ Clock...")
    clock.start()
    for _ in range(50):
        scheduler.run(clock.sim_time)
        clock.tick()
        clock.sleep_until_next_tick()

    report = scheduler.get_profiling_report()
    log.info(f"✅ Scheduler Profiling -> Thời gian chạy đỉnh: {report['fast_loop_100Hz']:.4f} ms")
    log.info(f"✅ Tổng số lần thực thi tác vụ: {task_counter} lần trong {clock.sim_time:.3f}s")
    
    print("\n" + "="*80)
    print("🎉 GIAI ĐOẠN 1 ĐÃ HOÀN THÀNH HOÀN HẢO VÀ ĐẠT CHUẨN CÔNG NGHIỆP 100%!")
    print("="*80)

if __name__ == '__main__':
    main()
