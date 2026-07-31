import time
import numpy as np
from flight_controller.flight_controller import FlightController
from physics.rigid_body_6dof import RigidBody6DoF
from core.logger import Logger

def run_px4_rd_benchmark():
    log = Logger.get_logger("PX4_Bench")
    print("================================================================================================")
    print("🚀 CHẠY NGHIỆM THU TIÊU CHUẨN PX4 R&D: RESTORED CONTROL CHAIN (NỀN TẢNG 22:49)...")
    print("================================================================================================")
    
    fc = FlightController(mass=1.2)
    physics = RigidBody6DoF(mass=1.2)
    
    # Thiết lập mục tiêu bay
    target_x, target_y, target_z = 5.0, 5.0, -10.0
    fc.arm_and_takeoff(target_z=target_z)
    fc.set_target(target_x, target_y, target_z, yaw=0.0)
    
    sim_time = 0.0
    dt = 0.0025 # Vòng lặp chính 400Hz
    
    print(f"🎯 Target: [X={target_x}m, Y={target_y}m, Z={target_z}m]")
    print("-" * 112)
    print(f"{'Time':<5} | {'Traj Z':<8} | {'True Z':<8} | {'Traj X':<8} | {'True X':<8} | {'Pitch°':<8} | {'FC Thrust':<10} | {'Act Thrust':<10}| {'Motors [M1..M4] N'}")
    print("-" * 112)
    
    actual_thrust = 0.0
    actual_torques = np.zeros(3)
    pwms = np.ones(4) * 1000.0
    
    for step in range(3201): # Simulated 8.0 seconds
        true_state = physics.get_state_dict()
        
        # 1. Cập nhật Flight Controller -> Tính toán FC Thrust, Torques & Motor PWMs
        actual_thrust, actual_torques, pwms = fc.update_sensors_and_estimate(true_state, {}, sim_time)
        
        # 2. BƯỚC NỐI MẠCH CHÍ MẠNG: Đưa Lực thực tế & Torque từ Motor vào Physics Engine!
        physics.step(dt, actual_thrust, actual_torques)
        
        # In dữ liệu log mỗi 0.5s
        if step % 200 == 0:
            traj_z = fc.traj_z.pos
            traj_x = fc.traj_x.pos
            true_z = true_state["position"][2]
            true_x = true_state["position"][0]
            pitch_deg = np.degrees(true_state["orientation"][1])
            
            # Tính lực nâng từng motor từ PWM để in log
            rpms = (pwms - 1000.0) / 1000.0 * 1200.0
            m_thrusts = 1.0e-5 * (rpms ** 2)
            m_str = f"[{m_thrusts[0]:.1f}, {m_thrusts[1]:.1f}, {m_thrusts[2]:.1f}, {m_thrusts[3]:.1f}]"
            
            print(f"{sim_time:<5.1f} | {traj_z:<8.2f} | {true_z:<8.2f} | {traj_x:<8.2f} | {true_x:<8.2f} | {pitch_deg:<8.1f} | {fc.total_thrust:<10.2f} | {actual_thrust:<10.2f}| {m_str}")
            
        sim_time += dt

    print("=" * 112)
    print("📁 Đã hoàn tất mô phỏng nối mạch điều khiển chuẩn xác!")

if __name__ == "__main__":
    run_px4_rd_benchmark()
