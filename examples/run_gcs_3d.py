import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QProgressBar, QGroupBox, QSplitter)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from flight_controller.flight_controller import FlightController
from physics.rigid_body_6dof import RigidBody6DoF


class IntegratedGCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("QuangUAV - Ground Control Station & 6DoF 3D Trajectory Platform")
        self.setGeometry(30, 50, 1480, 820)
        self.setStyleSheet("background-color: #121212; color: #ffffff; font-family: Ubuntu, sans-serif;")

        self.fc = FlightController(mass=1.2)
        self.physics = RigidBody6DoF(mass=1.2)

        self.trajectory_type = "FIGURE8"
        self.sim_time = 0.0
        self.dt = 0.005          # 200 Hz

        self.traj_3d = []
        self.ref_3d = []

        self._init_ui()

    def _init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        splitter = QSplitter(Qt.Horizontal)

        # ======================================================================
        # CỘT 1: GROUND CONTROL PANEL
        # ======================================================================
        gcs_panel = QWidget()
        gcs_panel.setFixedWidth(300)
        gcs_lay = QVBoxLayout(gcs_panel)

        # System Health
        sys_box = QGroupBox("SYSTEM HEALTH / ENVIRONMENT")
        sys_box.setStyleSheet("QGroupBox { font-weight: bold; color: #ffffff; border: 1px solid #424242; margin-top: 10px; }")
        sys_lay = QVBoxLayout(sys_box)
        sys_lay.addWidget(QLabel("Battery: 97.6%"))
        pbar = QProgressBar()
        pbar.setValue(97)
        pbar.setStyleSheet("QProgressBar::chunk { background-color: #0080ff; }")
        sys_lay.addWidget(pbar)
        sys_lay.addWidget(QLabel("GPS: 3D FIX (18 Sats)  EKF Status: OK"))
        sys_lay.addWidget(QLabel("CPU Load: 12.4%  Flight Time: 19.9 s"))
        gcs_lay.addWidget(sys_box)

        # Telemetry HUD
        hud_box = QGroupBox("REAL-TIME TELEMETRY HUD")
        hud_box.setStyleSheet("QGroupBox { font-weight: bold; color: #ffffff; border: 1px solid #424242; margin-top: 10px; }")
        hud_lay = QVBoxLayout(hud_box)

        self.lbl_z = QLabel("Altitude (Z): 0.00 m    H-Speed: 0.00 m/s")
        self.lbl_pos = QLabel("Pos X: 0.00 m    V-Speed: 0.00 m/s")
        self.lbl_y = QLabel("Pos Y: 0.00 m    Thrust: 0.00 N")
        self.lbl_att = QLabel("Pitch: 0.0°    Roll: 0.0°")
        self.lbl_yaw = QLabel("Yaw: 0.0°    Loop Time: 2.50 ms")
        self.lbl_motors = QLabel("Motors: (1000, 1000, 1000, 1000)")
        self.lbl_mode = QLabel("Mode: MISSION")
        self.lbl_mode.setStyleSheet("color: #00e676; font-weight: bold;")

        for w in [self.lbl_z, self.lbl_pos, self.lbl_y, self.lbl_att,
                  self.lbl_yaw, self.lbl_motors, self.lbl_mode]:
            hud_lay.addWidget(w)
        gcs_lay.addWidget(hud_box)

        # Trajectory Selector
        traj_box = QGroupBox("TRAJECTORY PATTERN SELECTOR")
        traj_box.setStyleSheet("QGroupBox { font-weight: bold; color: #00b0ff; border: 1px solid #424242; margin-top: 10px; }")
        traj_lay = QVBoxLayout(traj_box)

        btn_fig8 = QPushButton("♾ Figure-8 Trajectory")
        btn_fig8.setStyleSheet("background-color: #0288d1; color: white; font-weight: bold; padding: 6px;")
        btn_fig8.clicked.connect(lambda: self._set_trajectory("FIGURE8"))

        btn_helix = QPushButton("🌀 3D Helix / Circular Trajectory")
        btn_helix.setStyleSheet("background-color: #0097a7; color: white; font-weight: bold; padding: 6px;")
        btn_helix.clicked.connect(lambda: self._set_trajectory("HELIX"))

        traj_lay.addWidget(btn_fig8)
        traj_lay.addWidget(btn_helix)
        gcs_lay.addWidget(traj_box)

        # Runtime Control
        rt_box = QGroupBox("SOFTWARE-DEFINED RUNTIME")
        rt_box.setStyleSheet("QGroupBox { font-weight: bold; color: #00e676; border: 1px solid #424242; margin-top: 10px; }")
        rt_lay = QVBoxLayout(rt_box)

        self.lbl_ctrl = QLabel("Active Ctrl: PID")
        self.lbl_ctrl.setStyleSheet("font-size: 15px; font-weight: bold; color: #ffeb3b;")
        self.lbl_health = QLabel("Health: HEALTHY")

        btn_swap = QPushButton("⚡ HOT-SWAP (PID➔LQR➔MPC)")
        btn_swap.setStyleSheet("background-color: #7c4dff; color: white; font-weight: bold; padding: 6px;")
        btn_swap.clicked.connect(self._cycle_controller)

        rt_lay.addWidget(self.lbl_ctrl)
        rt_lay.addWidget(self.lbl_health)
        rt_lay.addWidget(btn_swap)
        gcs_lay.addWidget(rt_box)
        gcs_lay.addStretch()

        splitter.addWidget(gcs_panel)

        # ======================================================================
        # CỘT 2: VIEWPORT 6DOF 3D THẬT (OpenGL)
        # ======================================================================
        self.view3d = gl.GLViewWidget()
        self.view3d.setBackgroundColor('#0d1117')
        self.view3d.setCameraPosition(distance=28, elevation=28, azimuth=45)

        # Grid
        grid = gl.GLGridItem()
        grid.setSize(50, 50)
        grid.setSpacing(2, 2)
        self.view3d.addItem(grid)

        # World Axis
        axis = gl.GLAxisItem()
        axis.setSize(6, 6, 6)
        self.view3d.addItem(axis)

        # Planned Path (đỏ)
        self.planned_path = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                              color=(1.0, 0.2, 0.2, 0.85), width=2.5, antialias=True)
        self.view3d.addItem(self.planned_path)

        # Actual Path (vàng)
        self.actual_path = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                             color=(1.0, 0.9, 0.1, 1.0), width=3.5, antialias=True)
        self.view3d.addItem(self.actual_path)

        # Drone center
        self.drone_center = gl.GLScatterPlotItem(pos=np.zeros((1, 3), dtype=np.float32),
                                                 color=(0.0, 0.9, 1.0, 1.0), size=18)
        self.view3d.addItem(self.drone_center)

        # Drone arms
        self.drone_arms = gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32),
                                            color=(0.85, 0.85, 0.85, 1.0), width=4)
        self.view3d.addItem(self.drone_arms)

        # Propellers
        self.propellers = gl.GLScatterPlotItem(pos=np.zeros((4, 3), dtype=np.float32),
                                               color=(0.2, 1.0, 0.45, 1.0), size=13)
        self.view3d.addItem(self.propellers)

        splitter.addWidget(self.view3d)

        # ======================================================================
        # CỘT 3: DATA SCOPE
        # ======================================================================
        scope_panel = QWidget()
        scope_lay = QVBoxLayout(scope_panel)

        self.p_alt = pg.PlotWidget(title="Altitude Profile (m)")
        self.p_alt.showGrid(x=True, y=True)
        self.c_act_alt = self.p_alt.plot(pen=pg.mkPen('#00b0ff', width=2))
        self.c_ref_alt = self.p_alt.plot(pen=pg.mkPen('#00e676', width=1.5, style=Qt.DashLine))
        scope_lay.addWidget(self.p_alt)

        self.p_att = pg.PlotWidget(title="Attitude Angles (deg)")
        self.p_att.showGrid(x=True, y=True)
        self.c_roll = self.p_att.plot(pen=pg.mkPen('#ff1744', width=1.5))
        self.c_pitch = self.p_att.plot(pen=pg.mkPen('#ffea00', width=1.5))
        self.c_yaw = self.p_att.plot(pen=pg.mkPen('#00e5ff', width=1.5))
        scope_lay.addWidget(self.p_att)

        self.p_pwm = pg.PlotWidget(title="Motor PWM Outputs (μs)")
        self.p_pwm.showGrid(x=True, y=True)
        colors = ['#ff1744', '#00e676', '#2979ff', '#ffea00']
        self.c_pwms = [self.p_pwm.plot(pen=pg.mkPen(colors[i], width=1.5)) for i in range(4)]
        scope_lay.addWidget(self.p_pwm)

        splitter.addWidget(scope_panel)
        splitter.setSizes([300, 700, 480])
        main_layout.addWidget(splitter)

        # Data buffers
        self.t_data = []
        self.alt_data, self.ref_alt_data = [], []
        self.r_data, self.p_data, self.y_data = [], [], []
        self.pwm_data = [[], [], [], []]

        self.timer = QTimer()
        self.timer.timeout.connect(self._loop)
        self.timer.start(5)

    def _set_trajectory(self, traj_name):
        self.trajectory_type = traj_name
        self.traj_3d.clear()
        self.ref_3d.clear()
        print(f"🌐 [GCS COMMAND] Switched Trajectory Pattern to: {traj_name}")

    def _compute_target_trajectory(self, t):
        target = np.zeros(12)
        if self.trajectory_type == "FIGURE8":
            R = 5.0
            w = 0.3
            target[0] = R * np.sin(w * t)
            target[1] = R * np.sin(2.0 * w * t) / 2
            target[2] = -10.0
        elif self.trajectory_type == "HELIX":
            R = 6.0
            w = 0.4
            target[0] = R * np.cos(w * t)
            target[1] = R * np.sin(w * t)
            target[2] = -10.0 - 2.0 * np.sin(0.1 * t)
        else:
            target[0], target[1], target[2] = 0.0, 0.0, -10.0
        return target

    def _cycle_controller(self):
        curr = self.fc.runtime.active_name
        nxt = "LQR" if curr == "PID" else ("MPC" if curr == "LQR" else "PID")
        self.fc.runtime.request_hot_swap(nxt, sim_time=self.sim_time)

    def _loop(self):
        true_state = self.physics.get_state_dict()

        if "angular_velocity" in true_state:
            ang_vel = true_state["angular_velocity"]
        elif "omega" in true_state:
            ang_vel = true_state["omega"]
        else:
            ang_vel = np.zeros(3)

        current_state_array = np.concatenate([
            true_state["position"],
            true_state["velocity"],
            true_state["orientation"],
            ang_vel
        ])

        target_state = self._compute_target_trajectory(self.sim_time)

        # Auto hot-swap demo
        if self.sim_time >= 1.5 and self.fc.runtime.active_name == "PID":
            self.fc.runtime.request_hot_swap("LQR", self.sim_time)
        if self.sim_time >= 5.0 and self.fc.runtime.active_name == "LQR":
            self.fc.runtime.request_hot_swap("MPC", self.sim_time)

        actual_thrust, actual_torques, pwms = self.fc.update_sensors_and_estimate(true_state, {}, self.sim_time)
        u_cmd = self.fc.runtime.compute_control_signals(target_state, current_state_array, self.dt, self.sim_time)

        self.physics.step(self.dt, u_cmd[0], u_cmd[1:4])

        pos = true_state["position"]
        vel = true_state["velocity"]
        att = np.degrees(true_state["orientation"])
        yaw = true_state["orientation"][2]

        x, y, z = pos[0], pos[1], -pos[2]          # z dương hướng lên cho OpenGL

        # ===== HUD =====
        self.lbl_z.setText(f"Altitude (Z): {z:.2f} m    H-Speed: {np.linalg.norm(vel[0:2]):.2f} m/s")
        self.lbl_pos.setText(f"Pos X: {x:.2f} m    V-Speed: {-vel[2]:.2f} m/s")
        self.lbl_y.setText(f"Pos Y: {y:.2f} m    Thrust: {u_cmd[0]:.2f} N")
        self.lbl_att.setText(f"Pitch: {att[1]:.1f}°    Roll: {att[0]:.1f}°")
        self.lbl_yaw.setText(f"Yaw: {att[2]:.1f}°    Loop Time: 2.50 ms")
        self.lbl_motors.setText(f"Motors: ({int(pwms[0])}, {int(pwms[1])}, {int(pwms[2])}, {int(pwms[3])})")

        status = self.fc.runtime.get_telemetry_status()
        self.lbl_ctrl.setText(f"Active Ctrl: {status['active_controller']}")

        # ===== 3D Viewport =====
        self.traj_3d.append([x, y, z])
        self.ref_3d.append([target_state[0], target_state[1], -target_state[2]])

        if len(self.traj_3d) > 700:
            self.traj_3d.pop(0)
            self.ref_3d.pop(0)

        if len(self.traj_3d) > 1:
            self.actual_path.setData(pos=np.array(self.traj_3d, dtype=np.float32))
            self.planned_path.setData(pos=np.array(self.ref_3d, dtype=np.float32))

        # Drone position
        center = np.array([[x, y, z]], dtype=np.float32)
        self.drone_center.setData(pos=center)

        # 4 arms (chữ X)
        arm = 0.65
        c, s = np.cos(yaw), np.sin(yaw)
        arms = np.array([
            [x + arm*(c - s), y + arm*(s + c), z],
            [x - arm*(c - s), y - arm*(s + c), z],
            [x + arm*(c + s), y + arm*(s - c), z],
            [x - arm*(c + s), y - arm*(s - c), z]
        ], dtype=np.float32)

        self.drone_arms.setData(pos=arms)
        self.propellers.setData(pos=arms)

        # ===== Scope =====
        self.t_data.append(self.sim_time)
        self.alt_data.append(z)
        self.ref_alt_data.append(-target_state[2])
        self.r_data.append(att[0])
        self.p_data.append(att[1])
        self.y_data.append(att[2])
        for i in range(4):
            self.pwm_data[i].append(pwms[i])

        if len(self.t_data) > 400:
            self.t_data.pop(0)
            self.alt_data.pop(0)
            self.ref_alt_data.pop(0)
            self.r_data.pop(0)
            self.p_data.pop(0)
            self.y_data.pop(0)
            for i in range(4):
                self.pwm_data[i].pop(0)

        self.c_act_alt.setData(self.t_data, self.alt_data)
        self.c_ref_alt.setData(self.t_data, self.ref_alt_data)
        self.c_roll.setData(self.t_data, self.r_data)
        self.c_pitch.setData(self.t_data, self.p_data)
        self.c_yaw.setData(self.t_data, self.y_data)
        for i in range(4):
            self.c_pwms[i].setData(self.t_data, self.pwm_data[i])

        self.sim_time += self.dt


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = IntegratedGCSWindow()
    window.show()
    sys.exit(app.exec_())
