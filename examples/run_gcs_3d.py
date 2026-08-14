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
        self.setWindowTitle("QuangUAV - Interactive 3D Ground Control Station")
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
        self.target_position = np.array([0.0, 0.0, 0.0]) # System NED
        self.land_start_xy = np.array([0.0, 0.0])

        self.prop_angle = 0.0

        self.traj_3d = []
        self.ref_3d = []

        self._init_ui()

    def _generate_circle_points(self, radius, center_x=0.0, center_y=0.0, center_z=0.0, segments=32):
        angles = np.linspace(0, 2*np.pi, segments)
        xs = center_x + radius * np.cos(angles)
        ys = center_y + radius * np.sin(angles)
        zs = np.full_like(xs, center_z)
        return np.column_stack([xs, ys, zs])

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

        self.lbl_target_pos = QLabel("🎯 Target (X,Y,Z):  [ 0.00,  0.00,  0.00] m")
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

        self.btn_takeoff = QPushButton("🛫 AUTO TAKEOFF")
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

        # 3. QUĨ ĐẠO PHỨC TẠP
        traj_box = QGroupBox("3. ADVANCED 3D TRAJECTORIES")
        traj_box.setStyleSheet("QGroupBox { font-weight: bold; color: #a371f7; border: 1px solid #30363d; margin-top: 5px; }")
        traj_lay = QVBoxLayout(traj_box)

        btn_knot = QPushButton("🪢 1. Trefoil Knot 3D")
        btn_knot.setStyleSheet("background-color: #8957e5; color: white; font-weight: bold; padding: 5px; text-align: left;")
        btn_knot.clicked.connect(lambda: self._set_preset_trajectory("KNOT3D"))

        btn_rose = QPushButton("🌸 2. 3D Rose Curve (4-Petal)")
        btn_rose.setStyleSheet("background-color: #d2a8ff; color: #0d1117; font-weight: bold; padding: 5px; text-align: left;")
        btn_rose.clicked.connect(lambda: self._set_preset_trajectory("ROSE3D"))

        btn_cone = QPushButton("🌪 3. Conical Helix Spiral")
        btn_cone.setStyleSheet("background-color: #0891b2; color: white; font-weight: bold; padding: 5px; text-align: left;")
        btn_cone.clicked.connect(lambda: self._set_preset_trajectory("CONICAL"))

        btn_grid = QPushButton("🔲 4. 3D Grid Search (Sweep)")
        btn_grid.setStyleSheet("background-color: #1f6beb; color: white; font-weight: bold; padding: 5px; text-align: left;")
        btn_grid.clicked.connect(lambda: self._set_preset_trajectory("GRIDSEARCH"))

        btn_fig8 = QPushButton("♾ 5. Standard Figure-8")
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
        # CỘT GIỮA: OPENGL 3D VIEWPORT & KHUNG Ô VUÔNG ĐƠN VỊ 1M (3D UNIT GRID CAGE)
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

        lbl_axes_legend = QLabel("<span style='color:#ff3333; font-weight:bold;'>🔴 X (Bắc)</span> | "
                                 "<span style='color:#33ff33; font-weight:bold;'>🟢 Y (Đông)</span> | "
                                 "<span style='color:#3399ff; font-weight:bold;'>🔵 Z (Cao)</span>")
        lbl_axes_legend.setStyleSheet("background-color: #161b22; padding: 3px 8px; border-radius: 4px; border: 1px solid #30363d;")

        self.btn_cam_follow = QPushButton("🎥 FOLLOW: OFF")
        self.btn_cam_follow.setStyleSheet("background-color: #21262d; color: #58a6ff; font-weight: bold; padding: 4px;")
        self.btn_cam_follow.clicked.connect(self._toggle_cam_follow)

        btn_cam_iso = QPushButton("📐 3D ISO")
        btn_cam_top = QPushButton("⬇ Top View")
        for b in [btn_cam_iso, btn_cam_top]:
            b.setStyleSheet("background-color: #21262d; color: #c9d1d9; padding: 4px;")

        btn_cam_iso.clicked.connect(lambda: self.view3d.setCameraPosition(distance=32, elevation=25, azimuth=45))
        btn_cam_top.clicked.connect(lambda: self.view3d.setCameraPosition(distance=36, elevation=89, azimuth=0))

        overlay_bar.addWidget(self.pbar_takeoff, stretch=2)
        overlay_bar.addWidget(lbl_axes_legend)
        overlay_bar.addWidget(self.btn_cam_follow)
        overlay_bar.addWidget(btn_cam_iso)
        overlay_bar.addWidget(btn_cam_top)
        mid_lay.addLayout(overlay_bar)

        self.view3d = gl.GLViewWidget()
        self.view3d.setBackgroundColor('#0d1117')
        self.view3d.setCameraPosition(distance=32, elevation=25, azimuth=45)

        # ---------------------------------------------------------------------
        # TẠO LƯỚI Ô VUÔNG ĐƠN VỊ 1M x 1M TRÊN CẢ 3 MẶT PHẲNG (XY, XZ, YZ)
        # ---------------------------------------------------------------------
        # 1. Lưới Ô vuông Đơn vị Mặt Đáy XY (Z = 0)
        grid_xy = gl.GLGridItem()
        grid_xy.setSize(24, 24, 0)
        grid_xy.setSpacing(1, 1, 1)  # Mỗi ô đúng 1m x 1m
        grid_xy.setColor((0.3, 0.4, 0.5, 0.30))
        self.view3d.addItem(grid_xy)

        # 2. Lưới Ô vuông Đơn vị Mặt Đứng XZ (Bắc - Cao)
        grid_xz = gl.GLGridItem()
        grid_xz.setSize(24, 20, 0)
        grid_xz.setSpacing(1, 1, 1)  # Mỗi ô đúng 1m x 1m
        grid_xz.rotate(90, 1, 0, 0)
        grid_xz.translate(0, 0, 10)
        grid_xz.setColor((0.6, 0.2, 0.2, 0.20))
        self.view3d.addItem(grid_xz)

        # 3. Lưới Ô vuông Đơn vị Mặt Đứng YZ (Đông - Cao)
        grid_yz = gl.GLGridItem()
        grid_yz.setSize(24, 20, 0)
        grid_yz.setSpacing(1, 1, 1)  # Mỗi ô đúng 1m x 1m
        grid_yz.rotate(90, 0, 1, 0)
        grid_yz.translate(0, 0, 10)
        grid_yz.setColor((0.2, 0.6, 0.2, 0.20))
        self.view3d.addItem(grid_yz)

        # 3 TRỤC TỌA ĐỘ TRỤC TỌA ĐỘ NỔI BẬT CHÍNH
        self.axis_x = gl.GLLinePlotItem(pos=np.array([[0,0,0], [12.0, 0, 0]], dtype=np.float32),
                                       color=(1.0, 0.15, 0.15, 1.0), width=5.5, antialias=True)
        self.axis_y = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0, 12.0, 0]], dtype=np.float32),
                                       color=(0.15, 1.0, 0.15, 1.0), width=5.5, antialias=True)
        self.axis_z = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0, 0, 16.0]], dtype=np.float32),
                                       color=(0.2, 0.6, 1.0, 1.0), width=5.5, antialias=True)

        self.view3d.addItem(self.axis_x)
        self.view3d.addItem(self.axis_y)
        self.view3d.addItem(self.axis_z)

        # Mốc chóp đầu các trục & Gốc O(0,0,0)
        tip_pts = np.array([[12.0, 0, 0], [0, 12.0, 0], [0, 0, 16.0]], dtype=np.float32)
        tip_colors = np.array([[1.0, 0.15, 0.15, 1.0], [0.15, 1.0, 0.15, 1.0], [0.2, 0.6, 1.0, 1.0]], dtype=np.float32)
        self.axis_tips = gl.GLScatterPlotItem(pos=tip_pts, color=tip_colors, size=15)
        self.view3d.addItem(self.axis_tips)

        self.origin_node = gl.GLScatterPlotItem(pos=np.array([[0,0,0]], dtype=np.float32),
                                                color=(1.0, 0.9, 0.0, 1.0), size=18)
        self.view3d.addItem(self.origin_node)

        # CỘT MỐC CẤT CÁNH 10M
        self.beacon_line = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,10]], dtype=np.float32),
                                            color=(1.0, 0.2, 0.2, 0.4), width=1.8, antialias=True)
        self.view3d.addItem(self.beacon_line)

        target_ring_pts = self._generate_circle_points(1.2, 0, 0, 10, 32)
        self.target_ring = gl.GLLinePlotItem(pos=target_ring_pts, color=(1.0, 0.2, 0.2, 0.8), width=2.2, antialias=True)
        self.view3d.addItem(self.target_ring)

        self.drop_line = gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,0]], dtype=np.float32),
                                          color=(0.3, 0.8, 1.0, 0.6), width=1.5, antialias=True)
        self.view3d.addItem(self.drop_line)

        shadow_pts = self._generate_circle_points(0.6, 0, 0, 0, 24)
        self.shadow_disk = gl.GLLinePlotItem(pos=shadow_pts, color=(0.1, 0.5, 0.8, 0.7), width=1.8, antialias=True)
        self.view3d.addItem(self.shadow_disk)

        self.planned_path = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                              color=(1.0, 0.2, 0.2, 0.85), width=2.5, antialias=True)
        self.view3d.addItem(self.planned_path)

        self.actual_path = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                             color=(1.0, 0.85, 0.0, 1.0), width=3.5, antialias=True)
        self.view3d.addItem(self.actual_path)

        # MÔ HÌNH DRONE 3D
        self.drone_body = gl.GLLinePlotItem(pos=np.zeros((9, 3), dtype=np.float32),
                                           color=(0.0, 0.9, 1.0, 1.0), width=3.0, antialias=True)
        self.view3d.addItem(self.drone_body)

        self.front_arms = gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32),
                                           color=(1.0, 0.2, 0.2, 1.0), width=4.5, antialias=True)
        self.rear_arms = gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32),
                                          color=(0.9, 0.9, 0.9, 1.0), width=4.5, antialias=True)
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

        self._log_event("Clean 1m x 1m Unit Grid Cage Loaded.")

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
        self.target_position = np.array([0.0, 0.0, -10.0])
        self.lbl_mode.setText("STATE: AUTO TAKEOFF (10m)")
        self.lbl_mode.setStyleSheet("color: #00e676; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
        self._log_event("🛫 CMD: Auto Takeoff -> Target Z = 10.0m")

    def _cmd_hover(self):
        self.is_armed = True
        true_pos = self.physics.get_state_dict()["position"]
        self.flight_state = "HOVER"
        self.target_position = true_pos.copy()
        self.lbl_mode.setText("STATE: POSITION HOVER")
        self.lbl_mode.setStyleSheet("color: #ffeb3b; font-size: 13px; font-weight: bold; background-color: #161b22; padding: 6px;")
        self._log_event(f"📍 CMD: Position Lock Hover at Z={-true_pos[2]:.2f}m")

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
        self._log_event(f"🌐 Switched Advanced Trajectory -> {pattern_name}")

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
                self._log_event("Takeoff Complete -> Steady Hover at 10m.")

        elif self.flight_state == "LAND":
            self.target_position[0] = self.land_start_xy[0]
            self.target_position[1] = self.land_start_xy[1]
            self.target_position[2] = min(0.0, self.target_position[2] + 0.015)
            target[0:3] = self.target_position

            if self.physics.position[2] >= -0.15:
                self._cmd_stop()

        elif self.flight_state == "MISSION":
            if self.trajectory_pattern == "KNOT3D":
                w = 0.20
                scale = 5.0
                target[0] = scale * (np.sin(w*t) + 2.0 * np.sin(2.0*w*t)) / 3.0
                target[1] = scale * (np.cos(w*t) - 2.0 * np.cos(2.0*w*t)) / 3.0
                target[2] = -10.0 - 3.0 * np.sin(3.0*w*t)

            elif self.trajectory_pattern == "ROSE3D":
                w = 0.18
                R = 6.0
                r_t = R * np.cos(2.0 * w * t)
                target[0] = r_t * np.cos(w * t)
                target[1] = r_t * np.sin(w * t)
                target[2] = -10.0 + 2.5 * np.sin(4.0 * w * t)

            elif self.trajectory_pattern == "CONICAL":
                w = 0.35
                r_base = 1.5 + 0.4 * (t % 15.0)
                target[0] = r_base * np.cos(w * t)
                target[1] = r_base * np.sin(w * t)
                target[2] = -6.0 - 0.5 * (t % 15.0)

            elif self.trajectory_pattern == "GRIDSEARCH":
                period = 20.0
                t_mod = t % period
                target[2] = -10.0
                if t_mod < 5.0:
                    target[0] = -6.0 + 2.4 * t_mod
                    target[1] = -5.0
                elif t_mod < 10.0:
                    target[0] = 6.0
                    target[1] = -5.0 + 2.0 * (t_mod - 5.0)
                elif t_mod < 15.0:
                    target[0] = 6.0 - 2.4 * (t_mod - 10.0)
                    target[1] = 5.0
                else:
                    target[0] = -6.0
                    target[1] = 5.0 - 2.0 * (t_mod - 15.0)

            elif self.trajectory_pattern == "FIGURE8":
                R, w = 5.0, 0.25
                target[0] = R * np.sin(w * t)
                target[1] = R * np.sin(2.0 * w * t) / 2.0
                target[2] = -10.0

        return target

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

        if self.is_armed:
            u_cmd = self.controller.compute_control_signals(target_state, current_state_array, self.dt)
            self.physics.step(self.dt, u_cmd[0], u_cmd[1:4])
            self.prop_angle += 45.0 * self.dt
        else:
            u_cmd = np.zeros(4)
            self.physics.position[2] = 0.0
            self.physics.velocity = np.zeros(3)

        pos = true_state["position"]
        vel = true_state["velocity"]
        att = np.degrees(true_state["orientation"])
        yaw = true_state["orientation"][2]

        x, y, z = pos[0], pos[1], -pos[2]
        ref_x, ref_y, ref_z = target_state[0], target_state[1], -target_state[2]

        error_3d = np.linalg.norm(pos[0:3] - target_state[0:3])

        takeoff_pct = int(np.clip((z / 10.0) * 100, 0, 100))
        if self.is_armed:
            self.pbar_takeoff.setValue(takeoff_pct)
            self.pbar_takeoff.setFormat(f"TAKEOFF PROGRESS: {takeoff_pct}% ({z:4.1f}m / 10.0m) | Vz: {-vel[2]:4.2f}m/s")

        self.lbl_cur_pos.setText(f"📍 Actual (X,Y,Z):  [{x:6.2f}, {y:6.2f}, {z:6.2f}] m")
        self.lbl_target_pos.setText(f"🎯 Target (X,Y,Z):  [{ref_x:6.2f}, {ref_y:6.2f}, {ref_z:6.2f}] m")
        self.lbl_error_3d.setText(f"⚠️ Error 3D (m)  :   {error_3d:6.3f} m")
        self.lbl_flight_time.setText(f"⏱ Flight Time   :   {self.sim_time:6.1f} s")

        if self.is_armed:
            self.traj_3d.append([x, y, z])
            self.ref_3d.append([ref_x, ref_y, ref_z])

            if len(self.traj_3d) > 800:
                self.traj_3d.pop(0)
                self.ref_3d.pop(0)

        # CẬP NHẬT VIEWPORT 3D
        try:
            if len(self.traj_3d) > 1:
                self.actual_path.setData(pos=np.array(self.traj_3d, dtype=np.float32))
                self.planned_path.setData(pos=np.array(self.ref_3d, dtype=np.float32))

            self.beacon_line.setData(pos=np.array([[ref_x, ref_y, 0], [ref_x, ref_y, ref_z]], dtype=np.float32))
            target_ring_pts = self._generate_circle_points(1.2, ref_x, ref_y, ref_z, 32)
            self.target_ring.setData(pos=target_ring_pts.astype(np.float32))

            self.drop_line.setData(pos=np.array([[x, y, 0], [x, y, z]], dtype=np.float32))
            shadow_pts = self._generate_circle_points(0.6, x, y, 0.02, 24)
            self.shadow_disk.setData(pos=shadow_pts.astype(np.float32))

            # Mô hình Quadrotor 3D
            arm_len = 0.65
            c, s = np.cos(yaw), np.sin(yaw)

            b_size = 0.22
            body_pts = np.array([
                [x + b_size*c, y + b_size*s, z + 0.05],
                [x - b_size*s, y + b_size*c, z + 0.05],
                [x - b_size*c, y - b_size*s, z + 0.05],
                [x + b_size*s, y - b_size*c, z + 0.05],
                [x + b_size*c, y + b_size*s, z + 0.05],
                [x, y, z + 0.12],
                [x - b_size*c, y - b_size*s, z + 0.05],
                [x, y, z + 0.12],
                [x - b_size*s, y + b_size*c, z + 0.05]
            ], dtype=np.float32)
            self.drone_body.setData(pos=body_pts)

            f_arm1 = np.array([[x, y, z], [x + arm_len*(c - s), y + arm_len*(s + c), z]], dtype=np.float32)
            f_arm2 = np.array([[x, y, z], [x + arm_len*(c + s), y + arm_len*(s - c), z]], dtype=np.float32)
            r_arm1 = np.array([[x, y, z], [x - arm_len*(c - s), y - arm_len*(s + c), z]], dtype=np.float32)
            r_arm2 = np.array([[x, y, z], [x - arm_len*(c + s), y - arm_len*(s - c), z]], dtype=np.float32)

            self.front_arms.setData(pos=np.vstack([f_arm1, f_arm2]))
            self.rear_arms.setData(pos=np.vstack([r_arm1, r_arm2]))

            motors_pos = [
                [x + arm_len*(c - s), y + arm_len*(s + c), z],
                [x + arm_len*(c + s), y + arm_len*(s - c), z],
                [x - arm_len*(c - s), y - arm_len*(s + c), z],
                [x - arm_len*(c + s), y - arm_len*(s - c), z]
            ]

            hubs_lines = []
            for m in motors_pos:
                hubs_lines.append(m)
                hubs_lines.append([m[0], m[1], m[2] + 0.08])
            self.motor_hubs.setData(pos=np.array(hubs_lines, dtype=np.float32))

            prop_r = 0.38
            dirs = [1.0, -1.0, -1.0, 1.0]
            for i, m in enumerate(motors_pos):
                a_i = self.prop_angle * dirs[i] + yaw
                b1_start = [m[0] + prop_r*np.cos(a_i), m[1] + prop_r*np.sin(a_i), m[2] + 0.08]
                b1_end   = [m[0] - prop_r*np.cos(a_i), m[1] - prop_r*np.sin(a_i), m[2] + 0.08]
                
                b2_start = [m[0] + prop_r*np.cos(a_i + np.pi/2), m[1] + prop_r*np.sin(a_i + np.pi/2), m[2] + 0.08]
                b2_end   = [m[0] - prop_r*np.cos(a_i + np.pi/2), m[1] - prop_r*np.sin(a_i + np.pi/2), m[2] + 0.08]

                blade_lines = np.array([b1_start, b1_end, b2_start, b2_end], dtype=np.float32)
                self.prop_blades[i].setData(pos=blade_lines)

            if self.camera_follow and self.is_armed:
                self.view3d.opts['center'] = pg.Vector(x, y, z)

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
