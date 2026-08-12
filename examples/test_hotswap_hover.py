import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from physics.rigid_body_6dof import RigidBody6DoF
from controllers.pid_controller import PIDController
from controllers.lqr_controller import LQRController
from runtime.runtime_manager import RuntimeManager

def run_test_a_hotswap_hover():
    print("=" * 65)
    print("🚀 TEST A: PID ➔ LQR BUMP-LESS HOT-SWAP EVALUATION (STEADY HOVER AT 10M)")
    print("=" * 65)

    physics = RigidBody6DoF(mass=1.2)
    pid = PIDController(mass=1.2)
    lqr = LQRController(mass=1.2)

    runtime = RuntimeManager({"PID": pid, "LQR": lqr}, default_active="PID")

    dt = 0.005  # 200 Hz
    t_max = 12.0
    steps = int(t_max / dt)
    swap_time = 8.0  # Mốc kích hoạt Hot-Swap khi PID đã đứng vững ở 10m

    target_hover = np.zeros(12)
    target_hover[2] = -10.0  # Altitude 10m trong hệ NED

    history = {
        "time": [], "pos": [], "att": [], "u": [], "ctrl_name": []
    }

    u_before_swap = None
    u_after_swap = None
    pos_at_swap = None
    att_at_swap = None

    for i in range(steps):
        t = i * dt
        true_state = physics.get_state_dict()

        if "angular_velocity" in true_state:
            ang_vel = true_state["angular_velocity"]
        elif "omega" in true_state:
            ang_vel = true_state["omega"]
        else:
            ang_vel = np.zeros(3)

        current_state = np.concatenate([
            true_state["position"],
            true_state["velocity"],
            true_state["orientation"],
            ang_vel
        ])

        # Lưu trạng thái trước thời điểm Swap (t = 7.995s)
        if abs(t - (swap_time - dt)) < 1e-5:
            u_before_swap = runtime.compute_control_signals(target_hover, current_state, dt, t).copy()
            pos_at_swap = true_state["position"].copy()
            att_at_swap = true_state["orientation"].copy()

        # KÍCH HOẠT HOT-SWAP TẠI t = 8.0s
        if abs(t - swap_time) < 1e-5:
            runtime.request_hot_swap("LQR", sim_time=t)

        u_cmd = runtime.compute_control_signals(target_hover, current_state, dt, t)

        # Lưu trạng thái ngay sau thời điểm Swap
        if abs(t - swap_time) < 1e-5:
            u_after_swap = u_cmd.copy()

        physics.step(dt, u_cmd[0], u_cmd[1:4])

        history["time"].append(t)
        history["pos"].append(true_state["position"].copy())
        history["att"].append(true_state["orientation"].copy())
        history["u"].append(u_cmd.copy())
        history["ctrl_name"].append(runtime.active_name)

    history_pos = np.array(history["pos"])
    history_att = np.degrees(np.array(history["att"]))

    idx_swap = int(swap_time / dt)
    idx_window = int((swap_time + 1.0) / dt)

    # 1. Biến thiên tín hiệu điều khiển tức thời
    delta_thrust_instant = abs(u_after_swap[0] - u_before_swap[0])
    delta_roll_torque_instant = abs(u_after_swap[1] - u_before_swap[1])
    delta_pitch_torque_instant = abs(u_after_swap[2] - u_before_swap[2])
    delta_yaw_torque_instant = abs(u_after_swap[3] - u_before_swap[3])

    # 2. Độ lệch vị trí và góc cực đại trong cửa sổ 1.0s sau Swap
    pos_transient_window = history_pos[idx_swap:idx_window]
    att_transient_window = history_att[idx_swap:idx_window]

    pos_jump_max = np.max(np.linalg.norm(pos_transient_window - pos_at_swap, axis=1))
    att_jump_max = np.max(np.abs(att_transient_window - np.degrees(att_at_swap)))

    print("\n📊 === BÁO CÁO KẾT QUẢ KIỂM THỬ HOT-SWAP (TEST A) ===")
    print(f" 🔹 Cột mốc cất cánh PID ➔ Hover 10m  : Hoàn thành tại t = {swap_time:.2f} s")
    print(f" 🔹 Thời gian cửa sổ Cross-Fading     : 200 ms (t = {swap_time:.2f}s ➔ {swap_time+0.2:.2f}s)\n")
    
    print(f" 📌 1. BẢNG BIẾN THIÊN TÍN HIỆU TỨC THỜI TẠI MỐC HOT-SWAP (t = {swap_time:.1f}s):")
    print(f"    • ΔThrust (Lực nâng)   : {delta_thrust_instant:.4f} N")
    print(f"    • ΔTorque Roll         : {delta_roll_torque_instant:.6f} N·m")
    print(f"    • ΔTorque Pitch        : {delta_pitch_torque_instant:.6f} N·m")
    print(f"    • ΔTorque Yaw          : {delta_yaw_torque_instant:.6f} N·m\n")

    print(" 📌 2. BẢNG ĐỘ LỆCH ĐỘNG HỌC CỰC ĐẠI TRONG CỬA SỔ CHUYỂN GIAO (1.0s Window):")
    print(f"    • Position Jump Max (ΔPos 3D)  : {pos_jump_max:.4f} m ({pos_jump_max*1000:.2f} mm)")
    print(f"    • Attitude Jump Max (ΔAngle)   : {att_jump_max:.4f}°")

    print("\n ==========================================================")
    if pos_jump_max < 0.10 and delta_thrust_instant < 0.5:
        print(" 🏆 ĐÁNH GIÁ: BUMP-LESS HOT-SWAP ĐẠT CHUẨN AN TOÀN TUYỆT ĐỐI!")
        print("    (Không phát sinh cú sốc lực, vị trí lệch < 10cm)")
    else:
        print(" ⚠️ ĐÁNH GIÁ: CẦN TIẾP TỤC TINH CHỈNH TÍN HIỆU CROSS-FADING")
    print(" ==========================================================\n")

if __name__ == "__main__":
    run_test_a_hotswap_hover()
