import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from flight_controller.flight_controller import FlightController
from physics.rigid_body_6dof import RigidBody6DoF
from core.logger import Logger

def run_software_defined_uav_benchmark():
    log = Logger.get_logger("SD_UAV_Bench")
    print("==========================================================================================================")
    print("🚀 CHẠY NGHIỆM THU NỀN TẢNG SOFTWARE-DEFINED UAV PLATFORM (LIVE HOT-SWAP TEST)")
    print("==========================================================================================================")
    
    fc = FlightController(mass=1.2)
    physics = RigidBody6DoF(mass=1.2)
    
    target_x, target_y, target_z = 5.0, 5.0, -10.0
    fc.arm_and_takeoff(target_z=target_z)
    fc.set_target(target_x, target_y, target_z, yaw=0.0)
    
    sim_time = 0.0
    dt = 0.0025 # Vòng lặp chính 400Hz
    
    print(f"🎯 Target Setpoint : [X={target_x}m, Y={target_y}m, Z={target_z}m]")
    print("-" * 125)
    print(f"{'Time':<5} | {'Active Ctrl':<11} | {'Health':<10} | {'Blending':<8} | {'True Z':<8} | {'True X':<8} | {'Pitch°':<8} | {'Act Thrust':<10}| {'Motors [M1..M4] N'}")
    print("-" * 125)
    
    actual_thrust = 0.0
    actual_torques = np.zeros(3)
    pwms = np.ones(4) * 1000.0
    
    for step in range(3201): # Simulated 8.0 seconds
        true_state = physics.get_state_dict()
        
        # 🟢 HOT-SWAP EVENT 1: Chuyển đổi linh hoạt PID ➔ LQR tại sim_time >= 1.5s
        if sim_time >= 1.5 and fc.runtime.active_name == "PID":
            fc.runtime.request_hot_swap("LQR", current_sim_time=sim_time)

        # 🟢 HOT-SWAP EVENT 2: Chuyển đổi linh hoạt LQR ➔ MPC tại sim_time >= 5.0s
        if sim_time >= 5.0 and fc.runtime.active_name == "LQR":
            fc.runtime.request_hot_swap("MPC", current_sim_time=sim_time)
        
        # 1. Cập nhật Flight Controller thông qua Embedded Runtime
        actual_thrust, actual_torques, pwms = fc.update_sensors_and_estimate(true_state, {}, sim_time)
        
        # 2. Đẩy tín hiệu chấp hành xuống Physics Engine 6DoF
        physics.step(dt, actual_thrust, actual_torques)
        
        # In dữ liệu Telemetry mỗi 0.5s (step % 200)
        if step % 200 == 0:
            status = fc.runtime.get_telemetry_status()
            true_z = true_state["position"][2]
            true_x = true_state["position"][0]
            pitch_deg = np.degrees(true_state["orientation"][1])
            
            rpms = (pwms - 1000.0) / 1000.0 * 1200.0
            m_thrusts = 1.0e-5 * (rpms ** 2)
            m_str = f"[{m_thrusts[0]:.1f}, {m_thrusts[1]:.1f}, {m_thrusts[2]:.1f}, {m_thrusts[3]:.1f}]"
            
            print(f"{sim_time:<5.1f} | {status['active_controller']:<11} | {status['health_status']:<10} | {str(status['is_blending']):<8} | {true_z:<8.2f} | {true_x:<8.2f} | {pitch_deg:<8.1f} | {actual_thrust:<10.2f}| {m_str}")
            
        sim_time += dt

    print("=" * 125)
    print("📁 ĐÃ HOÀN THÀNH NGHIỆM THU HOT-SWAPPING NỀN TẢNG SOFTWARE-DEFINED UAV PLATFORM!")

if __name__ == "__main__":
    run_software_defined_uav_benchmark()