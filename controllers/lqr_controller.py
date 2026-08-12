import numpy as np
from scipy.linalg import solve_continuous_are
from interfaces.i_controller import IController

class LQRController(IController):
    """
    12-State Continuous-Time LQR with Physical Sign Convention.
    """

    def __init__(self, mass: float = 1.2, g: float = 9.81, 
                 Ixx: float = 0.015, Iyy: float = 0.015, Izz: float = 0.030):
        super().__init__()
        self.mass = mass
        self.g = g
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.hover_thrust = mass * g

        self.A, self.B = self._build_state_space_matrices()
        self.Q, self.R = self._build_cost_matrices()

        # Giải Phương trình Algebraic Riccati (CARE)
        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

        print("==========================================================")
        print("✅ [LQR INITIALIZED] Continuous Algebraic Riccati Equation Solved!")
        print(f"📊 Matrix K Shape: {self.K.shape}")
        print("==========================================================")

    def _build_state_space_matrices(self) -> tuple:
        A = np.zeros((12, 12))
        A[0, 3] = 1.0   # dot{X} = Vx
        A[1, 4] = 1.0   # dot{Y} = Vy
        A[2, 5] = 1.0   # dot{Z} = Vz
        A[3, 7] = -self.g  # dot{Vx} = -g * Pitch
        A[4, 6] = self.g   # dot{Vy} = +g * Roll
        A[6, 9] = 1.0   # dot{Roll} = p
        A[7, 10] = 1.0  # dot{Pitch} = q
        A[8, 11] = 1.0  # dot{Yaw} = r

        B = np.zeros((12, 4))
        # Hệ NED: Tăng lực nâng Delta_T làm giảm Z (dot{Vz} = -1/m * Delta_T)
        B[5, 0] = -1.0 / self.mass  # B[5,0] = -1/m chuẩn vật lý
        B[9, 1] = 1.0 / self.Ixx    # dot{p} / d(Torque_Roll)
        B[10, 2] = 1.0 / self.Iyy   # dot{q} / d(Torque_Pitch)
        B[11, 3] = 1.0 / self.Izz   # dot{r} / d(Torque_Yaw)

        return A, B

    def _build_cost_matrices(self) -> tuple:
        q_diag = [
            15.0,  15.0,  35.0,    # Sai số Vị trí (X, Y, Z)
             4.0,   4.0,   8.0,    # Sai số Vận tốc (Vx, Vy, Vz)
           100.0, 100.0,  20.0,    # Phạt Góc nghiêng (Roll, Pitch, Yaw)
             5.0,   5.0,   2.0     # Phạt Vận tốc góc (p, q, r)
        ]
        Q = np.diag(q_diag)

        r_diag = [0.02, 10.0, 10.0, 5.0]
        R = np.diag(r_diag)

        return Q, R

    def reset(self) -> None:
        pass

    def compute_control_signals(self, target_state: np.ndarray, current_state: np.ndarray, dt: float) -> np.ndarray:
        pos, target_pos = current_state[0:3], target_state[0:3]
        vel, target_vel = current_state[3:6], target_state[3:6]
        att, target_att = current_state[6:9], target_state[6:9]
        omega, target_omega = current_state[9:12], target_state[9:12]

        psi = att[2]
        c_psi, s_psi = np.cos(psi), np.sin(psi)
        
        dx_global = pos[0] - target_pos[0]
        dy_global = pos[1] - target_pos[1]
        
        dx_body = c_psi * dx_global + s_psi * dy_global
        dy_body = -s_psi * dx_global + c_psi * dy_global

        dx_body = np.clip(dx_body, -2.0, 2.0)
        dy_body = np.clip(dy_body, -2.0, 2.0)
        dz_body = pos[2] - target_pos[2]  # e_z trong hệ NED

        dvx_global = vel[0] - target_vel[0]
        dvy_global = vel[1] - target_vel[1]
        
        dvx_body = c_psi * dvx_global + s_psi * dvy_global
        dvy_body = -s_psi * dvy_global + c_psi * dvy_global

        dyaw = (att[2] - target_att[2] + np.pi) % (2 * np.pi) - np.pi

        error_state = np.array([
            dx_body,
            dy_body,
            dz_body,
            dvx_body,
            dvy_body,
            vel[2] - target_vel[2],
            att[0] - target_att[0],
            att[1] - target_att[1],
            dyaw,
            omega[0] - target_omega[0],
            omega[1] - target_omega[1],
            omega[2] - target_omega[2]
        ])

        # u_delta = -K * e_x
        u_delta = -self.K @ error_state

        total_thrust = np.clip(self.hover_thrust + u_delta[0], 4.0, 22.0)
        torque_roll = np.clip(u_delta[1], -0.2, 0.2)
        torque_pitch = np.clip(u_delta[2], -0.2, 0.2)
        torque_yaw = np.clip(u_delta[3], -0.15, 0.15)

        return np.array([total_thrust, torque_roll, torque_pitch, torque_yaw])
