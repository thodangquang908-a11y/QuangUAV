import os
os.environ["PYOPENGL_PLATFORM"] = "glx"
os.environ["QT_X11_NO_MITSHM"] = "1"

import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QProgressBar, QGroupBox, QSplitter, QGridLayout, QTextEdit)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QSurfaceFormat
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from controllers.pid_controller import PIDController
from physics.rigid_body_6dof import RigidBody6DoF


class IntegratedGCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("QuangUAV - Geofenced 3D Flight Control (Bounded Inside Coordinate Axes)")
        self.setGeometry(20, 40, 1600, 890)
        self.setStyleSheet("background-color: #0e1117; color: #ffffff; font-family: Ubuntu, sans-serif;")

        self.controller = PIDController(mass=1.2)
        self.physics = RigidBody6DoF(mass=1.2)

        # Trạng thái máy bay
        self.is_armed = False
        self.flight_state = "DISARMED"
        self.trajectory_pattern = "HOVER"
        self.camera_follow = False
        
        self.sim_time = 0.0
        self.dt = 0.005          # 200 Hz
        self.target_position = np.array([5.0, 5.0, -10.0]) # Tâm bay ở giữa khung trục tọa độ
        self.land_start_xy = np.array([5.0, 5.0])

        self.prop_angle = 0.0

        self.traj_3d = []
        self.ref_3d = []

        self._init_ui()

    def _generate_1m_grid_planes(self, x_max=10, y_max=10, z_max=12):
        xy_lines, xz_lines, yz_lines = [], [], []
        for y in range(0, y_max + 1):
            xy_lines.extend([[0.0, float(y), 0.0], [float(x_max), float(y), 0.0]])
        for x in range(0, x_max + 1):
            xy_lines.extend([[float(x), 0.0, 0.0], [float(x), float(y_max), 0.0]])

        for z in range(0, z_max + 1):
            xz_lines.extend([[0.0, 0.0, float(z)], [float(x_max), 0.0, float(z)]])
        for x in range(0, x_max + 1):
            xz_lines.extend([[float(x), 0.0, 0.0], [float(x), 0.0, float(z_max)]])

        for z in range(0, z_max + 1):
            yz_lines.extend([[0.0, 0.0, float(z)], [0.0, float(y_max), float(z)]])
        for y in range(0, y_max + 1):
            yz_lines.extend([[0.0, float(y), 0.0], [0.0, float(y), float(z_max)]])

        return np.array(xy_lines, dtype=np.float32), np.array(xz_lines, dtype=np.float32), np.array(yz_lines, dtype=np.float32)

    def _init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        splitter = QSplitter(Qt.Horizontal)

        # =========================================================================
        # CỘT TRÁI: BẢNG LỆNH & TELEMETRY
        # =========================================================================
        gcs_panel = QWidget()
        gcs_panel.setFixedWidth(340)
        gcs_lay = QVBoxLayout(gcs_panel)

        # 1. TRẠNG THÁI BAY
        state_box = QGroupBox("1. SYSTEM FLIGHT STATE")
        state_box.setStyleSheet("QGroupBox { font-weight: bold; color: #00b0ff; border: 1px solid #30363d; margin-top: 5px; }")
        state_lay = QVBoxLayout(state_box)

        self.lbl_mode = QLabel("STATE: DISARMED")
        self.lbl_mode.setStyleSheet("color: #ff5252; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px; border-radius: 4px;")

        self.lbl_cur_pos = QLabel("📍 Actual (X,Y,Z):  [ 0.00,  0.00,  0.00] m")
        self.lbl_cur_pos.setStyleSheet("color: #ffea00; font-size: 12px; font-weight: bold; background-color: #1c2128; padding: 4px;")

        self.lbl_target_pos = QLabel("🎯 Target (X,Y,Z):  [ 5.00,  5.00, 10.00] m")
        self.lbl_target_pos.setStyleSheet("color: #00e676; font-size: 12px; font-weight: bold; background-color: #1c2128; padding: 4px;")

        self.lbl_error_3d = QLabel("⚠️ Error 3D (m)  :   0.000 m")
        self.lbl_error_3d.setStyleSheet("color: #ff9100; font-size: 12px; font-weight: bold; background-color: #271d17; padding: 4px;")

        self.lbl_flight_time = QLabel("⏱ Flight Time   :   0.0 s")

        for w in [self.lbl_mode, self.lbl_cur_pos, self.lbl_target_pos, self.lbl_error_3d, self.lbl_flight_time]:
            state_lay.addWidget(w)
        gcs_lay.addWidget(state_box)

        # 2. LỆNH ĐIỀU KHIỂN BAY
        cmd_box = QGroupBox("2. DIRECT FLIGHT COMMANDS")
        cmd_box.setStyleSheet("QGroupBox { font-weight: bold; color: #00e676; border: 1px solid #30363d; margin-top: 5px; }")
        cmd_grid = QGridLayout(cmd_box)

        self.btn_takeoff = QPushButton("🛫 AUTO TAKEOFF (10m)")
        self.btn_takeoff.setStyleSheet("background-color: #1f6beb; color: white; font-weight: bold; padding: 8px;")
        self.btn_takeoff.clicked.connect(self._cmd_takeoff)

        self.btn_hover = QPushButton("📍 POS HOVER")
        self.btn_hover.setStyleSheet("background-color: #238636; color: white; font-weight: bold; padding: 8px;")
        self.btn_hover.clicked.connect(self._cmd_hover)

        self.btn_land = QPushButton("🛬 AUTO LAND")
        self.btn_land.setStyleSheet("background-color: #d97706; color: white; font-weight: bold; padding: 8px;")
        self.btn_land.clicked.connect(self._cmd_land)

        self.btn_stop = QPushButton("🛑 EMERGENCY STOP")
        self.btn_stop.setStyleSheet("background-color: #da3633; color: white; font-weight: bold; padding: 8px;")
        self.btn_stop.clicked.connect(self._cmd_stop)

        cmd_grid.addWidget(self.btn_takeoff, 0, 0)
        cmd_grid.addWidget(self.btn_hover, 0, 1)
        cmd_grid.addWidget(self.btn_land, 1, 0)
        cmd_grid.addWidget(self.btn_stop, 1, 1)
        gcs_lay.addWidget(cmd_box)

        # 3. QUĨ ĐẠO BÁM (NẰM TRỌN TRONG GÓC PHẦN TƯ KHUNG TRỤC)
        traj_box = QGroupBox("3. BOUNDED 3D TRAJECTORIES")
        traj_box.setStyleSheet("QGroupBox { font-weight: bold; color: #a371f7; border: 1px solid #30363d; margin-top: 5px; }")
        traj_lay = QVBoxLayout(traj_box)

        btn_knot = QPushButton("🪢 1. In-Grid Knot 3D")
        btn_knot.setStyleSheet("background-color: #8957e5; color: white; font-weight: bold; padding: 5px; text-align: left;")
        btn_knot.clicked.connect(lambda: self._set_preset_trajectory("KNOT3D"))

        btn_rose = QPushButton("🌸 2. In-Grid Rose Curve")
        btn_rose.setStyleSheet("background-color: #d2a8ff; color: #0d1117; font-weight: bold; padding: 5px; text-align: left;")
        btn_rose.clicked.connect(lambda: self._set_preset_trajectory("ROSE3D"))

        btn_cone = QPushButton("🌪 3. In-Grid Conical Helix")
        btn_cone.setStyleSheet("background-color: #0891b2; color: white; font-weight: bold; padding: 5px; text-align: left;")
        btn_cone.clicked.connect(lambda: self._set_preset_trajectory("CONICAL"))

        btn_grid = QPushButton("🔲 4. In-Grid Square Sweep")
        btn_grid.setStyleSheet("background-color: #1f6beb; color: white; font-weight: bold; padding: 5px; text-align: left;")
        btn_grid.clicked.connect(lambda: self._set_preset_trajectory("GRIDSEARCH"))

        btn_fig8 = QPushButton("♾ 5. In-Grid Figure-8")
        btn_fig8.setStyleSheet("background-color: #238636; color: white; font-weight: bold; padding: 5px; text-align: left;")
        btn_fig8.clicked.connect(lambda: self._set_preset_trajectory("FIGURE8"))

        for b in [btn_knot, btn_rose, btn_cone, btn_grid, btn_fig8]:
            traj_lay.addWidget(b)
        gcs_lay.addWidget(traj_box)

        # 4. SYSTEM LOG
        log_box = QGroupBox("4. SYSTEM EVENT LOG")
        log_box.setStyleSheet("QGroupBox { font-weight: bold; color: #8b949e; border: 1px solid #30363d; margin-top: 5px; }")
        log_lay = QVBoxLayout(log_box)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFixedHeight(120)
        self.txt_log.setStyleSheet("background-color: #0d1117; color: #3fb950; font-family: monospace; font-size: 11px;")
        log_lay.addWidget(self.txt_log)
        gcs_lay.addWidget(log_box)

        gcs_lay.addStretch()
        splitter.addWidget(gcs_panel)

        # =========================================================================
        # CỘT GIỮA: OPENGL 3D VIEWPORT
        # =========================================================================
        mid_container = QWidget()
        mid_lay = QVBoxLayout(mid_container)

        overlay_bar = QHBoxLayout()
        self.pbar_takeoff = QProgressBar()
        self.pbar_takeoff.setRange(0, 100)
        self.pbar_takeoff.setValue(0)
        self.pbar_takeoff.setFormat("TAKEOFF PROGRESS: %p% (0.0m / 10.0m)")
        self.pbar_takeoff.setStyleSheet("""
            QProgressBar {
                border: 1px solid #30363d;
                border-radius: 4px;
                text-align: center;
                background-color: #161b22;
                color: #ffffff;
                font-weight: bold;
                height: 22px;
            }
            QProgressBar::chunk {
                background-color: #238636;
            }
        """)

        lbl_axes_legend = QLabel("<span style='color:#ff5252; font-weight:bold;'>🔴 X (0-10m)</span> | "
                                 "<span style='color:#69f0ae; font-weight:bold;'>🟢 Y (0-10m)</span> | "
                                 "<span style='color:#40c4ff; font-weight:bold;'>🔵 Z (0-12m)</span>")
        lbl_axes_legend.setStyleSheet("background-color: #161b22; padding: 3px 8px; border-radius: 4px; border: 1px solid #30363d;")

        self.btn_cam_follow = QPushButton("🎥 FOLLOW: OFF")
        self.btn_cam_follow.setStyleSheet("background-color: #21262d; color: #58a6ff; font-weight: bold; padding: 4px;")
        self.btn_cam_follow.clicked.connect(self._toggle_cam_follow)

        btn_cam_iso = QPushButton("📐 3D ISO")
        btn_cam_top = QPushButton("⬇ Top View")
        for b in [btn_cam_iso, btn_cam_top]:
            b.setStyleSheet("background-color: #21262d; color: #c9d1d9; padding: 4px;")

        btn_cam_iso.clicked.connect(lambda: self.view3d.setCameraPosition(distance=24, elevation=25, azimuth=45))
        btn_cam_top.clicked.connect(lambda: self.view3d.setCameraPosition(distance=26, elevation=89, azimuth=0))

        overlay_bar.addWidget(self.pbar_takeoff, stretch=2)
        overlay_bar.addWidget(lbl_axes_legend)
        overlay_bar.addWidget(self.btn_cam_follow)
        overlay_bar.addWidget(btn_cam_iso)
        overlay_bar.addWidget(btn_cam_top)
        mid_lay.addLayout(overlay_bar)

        self.view3d = gl.GLViewWidget()
        self.view3d.setBackgroundColor('#0d1117')
        self.view3d.setCameraPosition(distance=24, elevation=25, azimuth=45)

        # 1. TẠO LƯỚI Ô VUÔNG 1M x 1M TRÊN 3 MẶT PHẲNG TỌA ĐỘ
        xy_pts, xz_pts, yz_pts = self._generate_1m_grid_planes(x_max=10, y_max=10, z_max=12)

        self.grid_xy_plot = gl.GLLinePlotItem(pos=xy_pts, color=(0.45, 0.55, 0.65, 0.45), width=1.2, mode='lines', antialias=True)
        self.grid_xz_plot = gl.GLLinePlotItem(pos=xz_pts, color=(0.85, 0.25, 0.25, 0.35), width=1.0, mode='lines', antialias=True)
        self.grid_yz_plot = gl.GLLinePlotItem(pos=yz_pts, color=(0.25, 0.85, 0.25, 0.35), width=1.0, mode='lines', antialias=True)

        self.view3d.addItem(self.grid_xy_plot)
        self.view3d.addItem(self.grid_xz_plot)
        self.view3d.addItem(self.grid_yz_plot)

        # 2. 3 TRỤC TỌA ĐỘ CHÍNH
        self.w_axis_x = gl.GLLinePlotItem(pos=np.array([[0,0,0], [10.0, 0, 0]], dtype=np.float32),
                                          color=(1.0, 0.2, 0.2, 1.0), width=4.0, antialias=True)
        self.w_axis_y = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0, 10.0, 0]], dtype=np.float32),
                                          color=(0.2, 1.0, 0.2, 1.0), width=4.0, antialias=True)
        self.w_axis_z = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0, 0, 12.0]], dtype=np.float32),
                                          color=(0.2, 0.6, 1.0, 1.0), width=4.0, antialias=True)
        self.view3d.addItem(self.w_axis_x)
        self.view3d.addItem(self.w_axis_y)
        self.view3d.addItem(self.w_axis_z)

        # Gốc tọa độ O(0,0,0)
        self.origin_node = gl.GLScatterPlotItem(pos=np.array([[0,0,0]], dtype=np.float32),
                                                color=(1.0, 0.9, 0.0, 1.0), size=14)
        self.view3d.addItem(self.origin_node)

        # 3. QUỸ ĐẠO THỰC TẾ & MỤC TIÊU
        self.planned_path = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                              color=(1.0, 0.2, 0.2, 0.85), width=2.5, antialias=True)
        self.view3d.addItem(self.planned_path)

        self.actual_path = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                             color=(1.0, 0.85, 0.0, 1.0), width=3.5, antialias=True)
        self.view3d.addItem(self.actual_path)

        # 4. MÔ HÌNH DRONE VÀ BODY AXES
        self.drone_body = gl.GLLinePlotItem(pos=np.zeros((9, 3), dtype=np.float32),
                                           color=(0.0, 0.9, 1.0, 1.0), width=3.0, antialias=True)
        self.view3d.addItem(self.drone_body)

        self.front_arms = gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32),
                                           color=(1.0, 0.2, 0.2, 1.0), width=4.5, mode='lines', antialias=True)
        self.rear_arms = gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32),
                                          color=(0.85, 0.85, 0.85, 1.0), width=4.5, mode='lines', antialias=True)
        self.view3d.addItem(self.front_arms)
        self.view3d.addItem(self.rear_arms)

        self.motor_hubs = gl.GLLinePlotItem(pos=np.zeros((8, 3), dtype=np.float32),
                                            color=(0.3, 0.8, 1.0, 1.0), width=3.5, mode='lines', antialias=True)
        self.view3d.addItem(self.motor_hubs)

        self.prop_blades = [
            gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32), color=(0.2, 1.0, 0.5, 1.0), width=3.0, mode='lines', antialias=True)
            for _ in range(4)
        ]
        for pb in self.prop_blades:
            self.view3d.addItem(pb)

        # BODY AXES TRÊN DRONE
        self.body_axis_x = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                             color=(1.0, 0.1, 0.1, 1.0), width=4.0, antialias=True)
        self.body_axis_y = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                             color=(0.1, 1.0, 0.1, 1.0), width=4.0, antialias=True)
        self.body_axis_z = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                             color=(0.2, 0.6, 1.0, 1.0), width=4.0, antialias=True)
        self.view3d.addItem(self.body_axis_x)
        self.view3d.addItem(self.body_axis_y)
        self.view3d.addItem(self.body_axis_z)

        mid_lay.addWidget(self.view3d)
        splitter.addWidget(mid_container)

        # =========================================================================
        # CỘT PHẢI: TELEMETRY SCOPE PLOTTER
        # =========================================================================
        scope_panel = QWidget()
        scope_lay = QVBoxLayout(scope_panel)

        self.p_err = pg.PlotWidget(title="Position Error 3D (m)")
        self.p_err.showGrid(x=True, y=True)
        self.c_err = self.p_err.plot(pen=pg.mkPen('#ff9100', width=2))
        scope_lay.addWidget(self.p_err)

        self.p_alt = pg.PlotWidget(title="Altitude Profile Z (m)")
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

        splitter.addWidget(scope_panel)
        splitter.setSizes([340, 700, 500])
        main_layout.addWidget(splitter)

        self.t_data = []
        self.err_data = []
        self.alt_data, self.ref_alt_data = [], []
        self.r_data, self.p_data, self.y_data = [], [], []

        self.timer = QTimer()
        self.timer.timeout.connect(self._loop)
        self.timer.start(10)

        self._log_event("GCS Viewport Initialized. Geofenced Boundaries Active.")

    def _toggle_cam_follow(self):
        self.camera_follow = not self.camera_follow
        txt = "🎥 FOLLOW: ON" if self.camera_follow else "🎥 FOLLOW: OFF"
        color = "#238636" if self.camera_follow else "#21262d"
        self.btn_cam_follow.setText(txt)
        self.btn_cam_follow.setStyleSheet(f"background-color: {color}; color: #ffffff; font-weight: bold; padding: 4px;")

    def _log_event(self, msg: str):
        self.txt_log.append(f"[{self.sim_time:6.1f}s] {msg}")

    def _cmd_takeoff(self):
        self.is_armed = True
        self.flight_state = "TAKEOFF"
        self.target_position = np.array([5.0, 5.0, -10.0])
        self.lbl_mode.setText("STATE: AUTO TAKEOFF (10m)")
        self.lbl_mode.setStyleSheet("color: #00e676; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
        self._log_event("🛫 CMD: Auto Takeoff -> Target [5, 5, 10m]")

    def _cmd_hover(self):
        self.is_armed = True
        true_pos = self.physics.get_state_dict()["position"]
        self.flight_state = "HOVER"
        self.target_position = true_pos.copy()
        self.lbl_mode.setText("STATE: POSITION HOVER")
        self.lbl_mode.setStyleSheet("color: #ffeb3b; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
        self._log_event(f"📍 CMD: Position Lock Hover at [{true_pos[0]:.1f}, {true_pos[1]:.1f}, {-true_pos[2]:.1f}]m")

    def _cmd_land(self):
        if not self.is_armed or abs(self.physics.position[2]) < 0.2:
            return
        true_pos = self.physics.get_state_dict()["position"]
        self.flight_state = "LAND"
        self.land_start_xy = true_pos[0:2].copy()
        self.target_position = np.array([self.land_start_xy[0], self.land_start_xy[1], true_pos[2]])
        self.lbl_mode.setText("STATE: AUTO LANDING")
        self.lbl_mode.setStyleSheet("color: #ff9100; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
        self._log_event("🛬 CMD: Auto Landing initiated...")

    def _cmd_stop(self):
        self.is_armed = False
        self.flight_state = "DISARMED"
        self.target_position = np.zeros(3)
        self.lbl_mode.setText("STATE: EMERGENCY DISARMED")
        self.lbl_mode.setStyleSheet("color: #ff5252; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
        self.controller.reset()
        
        self.physics.position[2] = 0.0
        self.physics.velocity = np.zeros(3)
        self.physics.orientation = np.zeros(3)
        self.physics.angular_velocity = np.zeros(3)
        
        self.pbar_takeoff.setValue(0)
        self.pbar_takeoff.setFormat("TAKEOFF PROGRESS: DISARMED")
        self._log_event("🛑 EMERGENCY STOP EXECUTED!")

    def _set_preset_trajectory(self, pattern_name):
        self.is_armed = True
        self.flight_state = "MISSION"
        self.trajectory_pattern = pattern_name
        self.lbl_mode.setText(f"STATE: MISSION ({pattern_name})")
        self.lbl_mode.setStyleSheet("color: #a371f7; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
        self.traj_3d.clear()
        self.ref_3d.clear()
        self._log_event(f"🌐 Switched Trajectory -> {pattern_name}")

    # =========================================================================
    # QUỸ ĐẠO BÁM KHÓA CHẶT TRONG GÓC PHẦN TƯ (X: 1-9m, Y: 1-9m, Z: 1-11m)
    # =========================================================================
    def _compute_target_trajectory(self, t):
        target = np.zeros(12)

        if self.flight_state == "DISARMED":
            target[0:3] = np.zeros(3)

        elif self.flight_state in ["TAKEOFF", "HOVER"]:
            target[0:3] = self.target_position
            if self.flight_state == "TAKEOFF" and abs(self.physics.position[2] - (-10.0)) < 0.2:
                self.flight_state = "HOVER"
                self.lbl_mode.setText("STATE: POSITION HOVER (10m)")
                self.lbl_mode.setStyleSheet("color: #ffeb3b; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
                self._log_event("Takeoff Complete -> Steady Hover at [5, 5, 10m].")

        elif self.flight_state == "LAND":
            self.target_position[0] = self.land_start_xy[0]
            self.target_position[1] = self.land_start_xy[1]
            self.target_position[2] = min(0.0, self.target_position[2] + 0.015)
            target[0:3] = self.target_position

            if self.physics.position[2] >= -0.15:
                self._cmd_stop()

        elif self.flight_state == "MISSION":
            # Tâm lưới ô vuông [5.0, 5.0, -10.0]
            cx, cy, cz = 5.0, 5.0, -10.0

            if self.trajectory_pattern == "KNOT3D":
                w = 0.20
                target[0] = cx + 3.0 * (np.sin(w*t) + 2.0 * np.sin(2.0*w*t)) / 3.0
                target[1] = cy + 3.0 * (np.cos(w*t) - 2.0 * np.cos(2.0*w*t)) / 3.0
                target[2] = cz - 1.5 * np.sin(3.0*w*t)

            elif self.trajectory_pattern == "ROSE3D":
                w = 0.18
                r_t = 3.5 * np.cos(2.0 * w * t)
                target[0] = cx + r_t * np.cos(w * t)
                target[1] = cy + r_t * np.sin(w * t)
                target[2] = cz + 1.2 * np.sin(4.0 * w * t)

            elif self.trajectory_pattern == "CONICAL":
                w = 0.22
                r = 1.5 + 2.0 * (0.5 + 0.5 * np.sin(0.1 * t))
                target[0] = cx + r * np.cos(w * t)
                target[1] = cy + r * np.sin(w * t)
                target[2] = cz - 1.5 * np.sin(0.15 * t)
                target[3] = -r * w * np.sin(w * t)
                target[4] =  r * w * np.cos(w * t)

            elif self.trajectory_pattern == "GRIDSEARCH":
                side = 3.5
                period = 16.0
                t_mod = t % period
                target[2] = cz
                if t_mod < 4.0:
                    s = t_mod / 4.0
                    target[0], target[1] = cx - side + 2*side*s, cy - side
                elif t_mod < 8.0:
                    s = (t_mod - 4.0) / 4.0
                    target[0], target[1] = cx + side, cy - side + 2*side*s
                elif t_mod < 12.0:
                    s = (t_mod - 8.0) / 4.0
                    target[0], target[1] = cx + side - 2*side*s, cy + side
                else:
                    s = (t_mod - 12.0) / 4.0
                    target[0], target[1] = cx - side, cy + side - 2*side*s

            elif self.trajectory_pattern == "FIGURE8":
                R, w = 3.5, 0.22
                target[0] = cx + R * np.sin(w * t)
                target[1] = cy + R * np.sin(2.0 * w * t) / 2.0
                target[2] = cz
                target[3] = R * w * np.cos(w * t)
                target[4] = R * w * np.cos(2.0 * w * t)

        return target

    def _loop(self):
        # KHÓA BIÊN AN TOÀN (HARD SAFETY GEOFENCE: X in [0, 10], Y in [0, 10], Z in [0, 12])
        if self.is_armed:
            self.physics.position[0] = np.clip(self.physics.position[0], 0.0, 10.0)
            self.physics.position[1] = np.clip(self.physics.position[1], 0.0, 10.0)
            self.physics.position[2] = np.clip(self.physics.position[2], -12.0, 0.0)

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

        if self.is_armed:
            u_cmd = self.controller.compute_control_signals(target_state, current_state_array, self.dt)
            self.physics.step(self.dt, u_cmd[0], u_cmd[1:4])
            self.prop_angle += 45.0 * self.dt
        else:
            u_cmd = np.zeros(4)
            self.physics.position[2] = 0.0
            self.physics.velocity = np.zeros(3)
            self.physics.orientation = np.zeros(3)
            self.physics.angular_velocity = np.zeros(3)

        pos = true_state["position"]
        vel = true_state["velocity"]
        att = np.degrees(true_state["orientation"])
        phi, theta, psi = true_state["orientation"]

        x, y_pos, z = pos[0], pos[1], -pos[2]
        ref_x, ref_y, ref_z = target_state[0], target_state[1], -target_state[2]

        error_3d = np.linalg.norm(pos[0:3] - target_state[0:3])

        takeoff_pct = int(np.clip((z / 10.0) * 100, 0, 100))
        if self.is_armed:
            self.pbar_takeoff.setValue(takeoff_pct)
            self.pbar_takeoff.setFormat(f"TAKEOFF PROGRESS: {takeoff_pct}% ({z:4.1f}m / 10.0m) | Vz: {-vel[2]:4.2f}m/s")

        self.lbl_cur_pos.setText(f"📍 Actual (X,Y,Z):  [{x:6.2f}, {y_pos:6.2f}, {z:6.2f}] m")
        self.lbl_target_pos.setText(f"🎯 Target (X,Y,Z):  [{ref_x:6.2f}, {ref_y:6.2f}, {ref_z:6.2f}] m")
        self.lbl_error_3d.setText(f"⚠️ Error 3D (m)  :   {error_3d:6.3f} m")
        self.lbl_flight_time.setText(f"⏱ Flight Time   :   {self.sim_time:6.1f} s")

        if self.is_armed:
            self.traj_3d.append([x, y_pos, z])
            self.ref_3d.append([ref_x, ref_y, ref_z])

            if len(self.traj_3d) > 800:
                self.traj_3d.pop(0)
                self.ref_3d.pop(0)

        # CẬP NHẬT VIEWPORT 3D
        try:
            if len(self.traj_3d) > 1:
                self.actual_path.setData(pos=np.array(self.traj_3d, dtype=np.float32))
                self.planned_path.setData(pos=np.array(self.ref_3d, dtype=np.float32))

            # MA TRẬN XOAY CHUẨN XÁC ĐỒ HỌA 3D
            c_p, s_p = np.cos(phi), np.sin(phi)
            c_t, s_t = np.cos(theta), np.sin(theta)
            c_y, s_y = np.cos(psi), np.sin(psi)

            R_ned = np.array([
                [c_y*c_t, c_y*s_t*s_p - s_y*c_p, c_y*s_t*c_p + s_y*s_p],
                [s_y*c_t, s_y*s_t*s_p + c_y*c_p, s_y*s_t*c_p - c_y*s_p],
                [-s_t,    c_t*s_p,               c_t*c_p]
            ])

            M = np.diag([1.0, 1.0, -1.0])
            R_gl = M @ R_ned @ M

            drone_origin = np.array([x, y_pos, z])

            # BODY AXES (0.9m) GẮN TRỰC TIẾP TẠI TÂM DRONE
            axis_len = 0.9
            bx_end = drone_origin + R_gl @ np.array([axis_len, 0.0, 0.0])
            by_end = drone_origin + R_gl @ np.array([0.0, axis_len, 0.0])
            bz_end = drone_origin + R_gl @ np.array([0.0, 0.0, axis_len])

            self.body_axis_x.setData(pos=np.array([drone_origin, bx_end], dtype=np.float32))
            self.body_axis_y.setData(pos=np.array([drone_origin, by_end], dtype=np.float32))
            self.body_axis_z.setData(pos=np.array([drone_origin, bz_end], dtype=np.float32))

            # Thân Quadrotor Canopy Frame
            b_size = 0.22
            raw_body = np.array([
                [ b_size,  b_size, 0.05],
                [-b_size,  b_size, 0.05],
                [-b_size, -b_size, 0.05],
                [ b_size, -b_size, 0.05],
                [ b_size,  b_size, 0.05],
                [ 0.0,     0.0,    0.12],
                [-b_size, -b_size, 0.05],
                [ 0.0,     0.0,    0.12],
                [-b_size,  b_size, 0.05]
            ])
            body_pts = drone_origin + (R_gl @ raw_body.T).T
            self.drone_body.setData(pos=body_pts.astype(np.float32))

            # 4 Cánh tay Quadrotor
            arm_l = 0.65
            f1 = drone_origin + R_gl @ np.array([ arm_l,  arm_l, 0.0])
            f2 = drone_origin + R_gl @ np.array([ arm_l, -arm_l, 0.0])
            r1 = drone_origin + R_gl @ np.array([-arm_l,  arm_l, 0.0])
            r2 = drone_origin + R_gl @ np.array([-arm_l, -arm_l, 0.0])

            f_lines = np.array([drone_origin, f1, drone_origin, f2], dtype=np.float32)
            r_lines = np.array([drone_origin, r1, drone_origin, r2], dtype=np.float32)

            self.front_arms.setData(pos=f_lines)
            self.rear_arms.setData(pos=r_lines)

            # Cụm động cơ đứng
            motors = [f1, f2, r1, r2]
            hubs = []
            for m in motors:
                top_m = m + R_gl @ np.array([0.0, 0.0, 0.06])
                hubs.extend([m, top_m])
            self.motor_hubs.setData(pos=np.array(hubs, dtype=np.float32))

            # 4 Cánh quạt quay phẳng
            prop_r = 0.35
            dirs = [1.0, -1.0, -1.0, 1.0]
            for i, m in enumerate(motors):
                m_top = m + R_gl @ np.array([0.0, 0.0, 0.06])
                a_i = self.prop_angle * dirs[i]
                
                b1_body = np.array([prop_r * np.cos(a_i), prop_r * np.sin(a_i), 0.0])
                b2_body = np.array([prop_r * np.cos(a_i + np.pi/2), prop_r * np.sin(a_i + np.pi/2), 0.0])

                p1_s = m_top + R_gl @ b1_body
                p1_e = m_top - R_gl @ b1_body
                p2_s = m_top + R_gl @ b2_body
                p2_e = m_top - R_gl @ b2_body

                self.prop_blades[i].setData(pos=np.array([p1_s, p1_e, p2_s, p2_e], dtype=np.float32))

            if self.camera_follow and self.is_armed:
                self.view3d.opts['center'] = pg.Vector(x, y_pos, z)

        except Exception:
            pass

        # CẬP NHẬT ĐỒ THỊ
        self.t_data.append(self.sim_time)
        self.err_data.append(error_3d)
        self.alt_data.append(z)
        self.ref_alt_data.append(ref_z)
        self.r_data.append(att[0])
        self.p_data.append(att[1])
        self.y_data.append(att[2])

        if len(self.t_data) > 400:
            self.t_data.pop(0)
            self.err_data.pop(0)
            self.alt_data.pop(0)
            self.ref_alt_data.pop(0)
            self.r_data.pop(0)
            self.p_data.pop(0)
            self.y_data.pop(0)

        self.c_err.setData(self.t_data, self.err_data)
        self.c_act_alt.setData(self.t_data, self.alt_data)
        self.c_ref_alt.setData(self.t_data, self.ref_alt_data)
        self.c_roll.setData(self.t_data, self.r_data)
        self.c_pitch.setData(self.t_data, self.p_data)
        self.c_yaw.setData(self.t_data, self.y_data)

        if self.is_armed:
            self.sim_time += self.dt


if __name__ == "__main__":
    fmt = QSurfaceFormat()
    fmt.setRenderableType(QSurfaceFormat.OpenGL)
    fmt.setProfile(QSurfaceFormat.CompatibilityProfile)
    fmt.setVersion(2, 1)
    QSurfaceFormat.setDefaultFormat(fmt)

    app = QApplication(sys.argv)
    window = IntegratedGCSWindow()
    window.show()
    sys.exit(app.exec_())
