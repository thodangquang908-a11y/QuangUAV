import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from physics.rigid_body_6dof import RigidBody6DoF
from controllers.pid_controller import PIDController
from controllers.lqr_controller import LQRController

def run_evaluation(controller_class, name: str, sim_time_max: float = 10.0, dt: float = 0.005):
    physics = RigidBody6DoF(mass=1.2)
    controller = controller_class(mass=1.2)
    
    time_steps = int(sim_time_max / dt)
    pos_errors = []
    control_efforts = []
    max_z = 0.0

    for i in range(time_steps):
        t = i * dt
        true_state = physics.get_state_dict()
        
        # Kiểm tra an toàn khóa tốc độ góc (omega / angular_velocity)
        if "angular_velocity" in true_state:
            ang_vel = true_state["angular_velocity"]
        elif "omega" in true_state:
            ang_vel = true_state["omega"]
        else:
            ang_vel = np.zeros(3)

        # Quỹ đạo Figure-8 mục tiêu
        target = np.zeros(12)
        R = 5.0
        w = 0.25
        target[0] = R * np.sin(w * t)
        target[1] = R * np.sin(2.0 * w * t) / 2.0
        target[2] = -10.0
        target[3] = R * w * np.cos(w * t)
        target[4] = R * w * np.cos(2.0 * w * t)

        current_state = np.concatenate([
            true_state["position"],
            true_state["velocity"],
            true_state["orientation"],
            ang_vel
        ])

        u_cmd = controller.compute_control_signals(target, current_state, dt)
        physics.step(dt, u_cmd[0], u_cmd[1:4])

        pos = true_state["position"]
        z_actual = -pos[2]
        max_z = max(max_z, z_actual)

        # Tính toán sai số vị trí 3D
        err_3d = np.linalg.norm(pos[0:3] - target[0:3])
        pos_errors.append(err_3d)
        
        # Năng lượng điều khiển: u_1^2 + sum(torque^2)
        control_efforts.append(u_cmd[0]**2 + np.sum(u_cmd[1:4]**2))

    rmse_pos = np.sqrt(np.mean(np.array(pos_errors)**2))
    total_energy = np.sum(control_efforts) * dt

    print(f"\n📊 === BENCHMARK RESULT: [{name}] ===")
    print(f" 🔹 RMSE Position Tracking Error: {rmse_pos:.4f} m")
    print(f" 🔹 Max Altitude Reached      : {max_z:.2f} m (Target: 10.0m)")
    print(f" 🔹 Total Control Energy (N^2s): {total_energy:.2f}")
    return rmse_pos, max_z, total_energy

if __name__ == "__main__":
    print("🚀 Running Quantitative Evaluation: Real Cascaded PID vs Real 12-State LQR...")
    rmse_pid, maxz_pid, e_pid = run_evaluation(PIDController, "Cascaded PID")
    rmse_lqr, maxz_lqr, e_lqr = run_evaluation(LQRController, "12-State CARE LQR")

    print("\n==========================================================")
    print("🏆 SUMMARY COMPARISON FOR THESIS REPORT:")
    print(f" 📌 Tracking Accuracy Improvement : {((rmse_pid - rmse_lqr)/rmse_pid)*100:.2f}%")
    print("==========================================================")
