import numpy as np
import time
from interfaces.i_controller import IController
from controllers.pid_controller import PIDController
from controllers.lqr_controller import LQRController
from controllers.mpc_controller import MPCController
from runtime.safe_transition import HealthMonitor, SafeTransitionLayer

class RuntimeManager(IController):
    """
    SOFTWARE-DEFINED UAV RUNTIME CORE - SPRINT 4 (SAFE TRANSITION & WARM START)
    """
    def __init__(self, default_controller_name: str = "PID"):
        super().__init__()
        
        self.controllers = {
            "PID": PIDController(),
            "LQR": LQRController(),
            "MPC": MPCController()
        }
        
        self.active_name = default_controller_name.upper()
        self.active_controller_name = self.active_name
        self.active_controller = self.controllers.get(self.active_name, self.controllers["PID"])
        
        self.transition_layer = SafeTransitionLayer(blend_duration_sec=0.2) # Cross-fading 200ms
        self.health_monitor = HealthMonitor(max_latency_ms=2.5)
        
        print("==================================================================================")
        print(f"⚙️ [RUNTIMEMANAGER] Initialized Software-Defined UAV Core Platform.")
        print(f"⚙️ Active Core Controller : [{self.active_name}]")
        print("==================================================================================")

    def request_hot_swap(self, new_controller_name: str, sim_time: float = 0.0) -> bool:
        """Hàm đồng bộ chữ ký chuẩn với GCS GUI & Simulator Loop"""
        return self.switch_controller(new_controller_name, sim_time=sim_time)

    def switch_controller(self, name: str, sim_time: float = 0.0) -> bool:
        target_name = name.upper()
        if target_name not in self.controllers:
            print(f"❌ [RUNTIME REJECT] Controller '{target_name}' not registered!")
            return False

        if target_name == self.active_name:
            return True

        old_ctrl = self.active_controller
        new_ctrl = self.controllers[target_name]

        # 🟢 WARM START & STATE SYNCHRONIZATION
        # Chuyển tiếp tích lũy Integrator từ PID sang LQR/MPC để chống giật lực
        if hasattr(new_ctrl, 'sync_state_from_previous'):
            new_ctrl.sync_state_from_previous(old_ctrl)

        self.transition_layer.start_transition(old_ctrl, new_ctrl, sim_time)
        
        print("\n" + "=" * 80)
        print(f"🔥 [RUNTIME HOT-SWAP SUCCESS] Sim Time: {sim_time:.2f}s")
        print(f"🔄 Dynamic Transition : [{self.active_name}]  ➔  [{target_name}]")
        print(f"🛡️ State Synchronization & Cross-Fading : ACTIVE (200ms Window)")
        print("=" * 80 + "\n")

        self.active_name = target_name
        self.active_controller_name = target_name
        self.active_controller = new_ctrl
        return True

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float, sim_time: float = 0.0) -> np.ndarray:
        t_start = time.perf_counter()

        if self.transition_layer.is_blending:
            u_out = self.transition_layer.compute_blended_signals(target_state, current_state, dt, sim_time)
        else:
            u_out = self.active_controller.compute_control_signals(target_state, current_state, dt)

        compute_time_ms = (time.perf_counter() - t_start) * 1000.0
        self.health_monitor.inspect(compute_time_ms, u_out)

        return u_out

    def reset(self) -> None:
        if hasattr(self.active_controller, 'reset'):
            self.active_controller.reset()

    def get_telemetry_status(self) -> dict:
        return {
            "active_controller": self.active_name,
            "is_blending": self.transition_layer.is_blending,
            "health_status": self.health_monitor.health_status,
            "compute_latency_ms": self.health_monitor.last_compute_time_ms
        }
