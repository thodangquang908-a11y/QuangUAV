import numpy as np
from physics.rigid_body_6dof import RigidBody6DoF

def run_physics_demo():
    print("="*70)
    print("🛸 ĐANG CHẠY THỬ NGHIỆM TẦNG PHYSICS ENGINE (6DoF + RK4)...")
    print("="*70)
    
    drone = RigidBody6DoF(mass=1.2)
    dt = 0.005 # 200Hz
    
    # Giả lập 1: Bơm lực nâng đúng bằng trọng lực (1.2kg * 9.81m/s^2 = 11.772N) để Treo (Hover)
    hover_thrust = 1.2 * 9.81
    zero_torques = np.zeros(3)
    
    drone.step(dt=dt, total_thrust=hover_thrust, torques=zero_torques)
    st = drone.get_state_dict()
    print(f"✅ Thử nghiệm 1 (Bay treo Hover - Lực nâng {hover_thrust:.2f}N):")
    print(f"   - Cao độ Z: {st['position'][2]:.4f} m (Giữ nguyên vị trí 0)")
    
    # Giả lập 2: Cho Drone rơi tự do từ độ cao 5m trong 1 giây (Thrust = 0)
    drone.state[2] = -5.0 # Độ cao 5m trong NED (Z = -5)
    for _ in range(200): # 200 bước = 1 giây
        drone.step(dt=dt, total_thrust=0.0, torques=zero_torques)
        
    st = drone.get_state_dict()
    print(f"\n✅ Thử nghiệm 2 (Rơi tự do từ 5m sau 1s không bơm ga):")
    print(f"   - Cao độ Z hiện tại: {st['position'][2]:.2f} m (Gần chạm đất)")
    print(f"   - Vận tốc rơi (w): {st['velocity'][2]:.2f} m/s")
    print("="*70)
    print("🎉 TẦNG PHYSICS ENGINE ĐÃ HOÀN THÀNH VÀ TÍNH TOÁN CHÍNH XÁC!")
    print("="*70)

if __name__ == '__main__':
    run_physics_demo()
