import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QGridLayout, QGroupBox, QComboBox)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph.opengl as gl

from visualization.camera_controller import CameraController
from interfaces.telemetry_interface import TelemetryData

class DroneGCSWindow(QMainWindow):
    """Ground Control Station (GCS) 3D: Đã bổ sung Trục Tọa Độ XYZ (GLAxisItem)."""
    MAX_POINTS = 800

    def __init__(self, sim_thread, flight_controller) -> None:
        super().__init__()
        self.sim_thread = sim_thread
        self.fc = flight_controller
        
        self.setWindowTitle("QuangUAV - Ground Control Station & 3D Visualizer (PX4 R&D)")
        self.resize(1360, 768)
        
        self.latest_telemetry: TelemetryData = TelemetryData()
        self.flight_history_pos = []
        self.planned_path_history = []
        self.prop_angle = 0.0
        self.frame_count = 0

        self.init_ui()
        self.init_3d_scene()
        
        self.camera_ctrl = CameraController(self.view3d)
        self.sim_thread.telemetry_signal.connect(self.on_telemetry_received)
        
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.render_gui_60hz)
        self.gui_timer.start(16)

    def init_ui(self) -> None:
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        self.view3d = gl.GLViewWidget()
        self.view3d.setCameraPosition(distance=18, elevation=25, azimuth=45)
        main_layout.addWidget(self.view3d, stretch=3)

        dash_panel = QWidget()
        dash_layout = QVBoxLayout(dash_panel)
        main_layout.addWidget(dash_panel, stretch=1)

        hud_box = QGroupBox("📊 REAL-TIME TELEMETRY HUD")
        hud_grid = QGridLayout()
        
        self.lbl_alt     = QLabel("Altitude (Z): 0.00 m")
        self.lbl_pos_x   = QLabel("Pos X: 0.00 m")
        self.lbl_pos_y   = QLabel("Pos Y: 0.00 m")
        self.lbl_h_speed = QLabel("H-Speed: 0.00 m/s")
        self.lbl_v_speed = QLabel("V-Speed (Vz): 0.00 m/s")
        self.lbl_pitch   = QLabel("Pitch: 0.0°")
        self.lbl_roll    = QLabel("Roll: 0.0°")
        self.lbl_yaw     = QLabel("Yaw: 0.0°")
        self.lbl_thrust  = QLabel("Thrust: 0.00 N")
        self.lbl_pwms    = QLabel("Motors: [1000..1000]")
        self.lbl_loop    = QLabel("Loop Time: 0.00 ms")
        self.lbl_mode    = QLabel("Mode: POSITION_HOLD")

        for lbl in [self.lbl_alt, self.lbl_pos_x, self.lbl_pos_y, self.lbl_h_speed, self.lbl_v_speed,
                    self.lbl_pitch, self.lbl_roll, self.lbl_yaw, self.lbl_thrust, 
                    self.lbl_pwms, self.lbl_loop, self.lbl_mode]:
            lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #2c3e50;")

        hud_grid.addWidget(self.lbl_alt, 0, 0);     hud_grid.addWidget(self.lbl_h_speed, 0, 1)
        hud_grid.addWidget(self.lbl_pos_x, 1, 0);   hud_grid.addWidget(self.lbl_v_speed, 1, 1)
        hud_grid.addWidget(self.lbl_pos_y, 2, 0);   hud_grid.addWidget(self.lbl_thrust, 2, 1)
        hud_grid.addWidget(self.lbl_pitch, 3, 0);   hud_grid.addWidget(self.lbl_roll, 3, 1)
        hud_grid.addWidget(self.lbl_yaw, 4, 0);     hud_grid.addWidget(self.lbl_loop, 4, 1)
        hud_grid.addWidget(self.lbl_pwms, 5, 0, 1, 2)
        hud_grid.addWidget(self.lbl_mode, 6, 0, 1, 2)
        hud_box.setLayout(hud_grid)
        dash_layout.addWidget(hud_box)

        cam_box = QGroupBox("🎥 SMART CAMERA MODE")
        cam_layout = QVBoxLayout()
        self.combo_cam = QComboBox()
        self.combo_cam.addItems(["1. Free Camera", "2. Follow Camera", "3. Top-down View", "4. Cockpit FPV", "5. Orbit Camera 360°"])
        self.combo_cam.currentIndexChanged.connect(self.on_camera_changed)
        cam_layout.addWidget(self.combo_cam)
        cam_box.setLayout(cam_layout)
        dash_layout.addWidget(cam_box)

        ctrl_box = QGroupBox("🕹️ FLIGHT CONTROL")
        ctrl_layout = QVBoxLayout()
        
        self.btn_takeoff = QPushButton("🚀 ARM & TAKEOFF")
        self.btn_takeoff.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold; padding: 6px;")
        self.btn_takeoff.clicked.connect(self.cmd_takeoff)

        self.btn_pause = QPushButton("⏸️ PAUSE / RESUME SIM")
        self.btn_pause.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold; padding: 6px;")
        self.btn_pause.clicked.connect(self.cmd_pause)
        
        self.btn_report = QPushButton("📄 GENERATE REPORT & CSV")
        self.btn_report.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; padding: 6px;")
        self.btn_report.clicked.connect(self.generate_report)
        
        ctrl_layout.addWidget(self.btn_takeoff)
        ctrl_layout.addWidget(self.btn_pause)
        ctrl_layout.addWidget(self.btn_report)
        ctrl_box.setLayout(ctrl_layout)
        dash_layout.addWidget(ctrl_box)
        
        dash_layout.addStretch()

    def init_3d_scene(self) -> None:
        self.view3d.setBackgroundColor((0.05, 0.05, 0.1, 1.0))
        
        # 1. Lưới mặt đất
        grid = gl.GLGridItem()
        grid.setSize(50, 50, 1)
        grid.setSpacing(1, 1, 1)
        self.view3d.addItem(grid)

        # 2. BỔ SUNG TRỤC TỌA ĐỘ 3D (X: Đỏ, Y: Xanh lá, Z: Xanh dương)
        axes = gl.GLAxisItem()
        axes.setSize(x=3.0, y=3.0, z=3.0) # Độ dài 3 trục tọa độ (3 mét)
        self.view3d.addItem(axes)

        zero_pos = np.zeros((1, 3), dtype=np.float32)
        line_zeros = np.zeros((2, 3), dtype=np.float32)

        self.home_marker   = gl.GLScatterPlotItem(pos=zero_pos, color=(0.0, 0.4, 1.0, 1.0), size=14.0)
        self.target_marker = gl.GLScatterPlotItem(pos=zero_pos, color=(1.0, 0.0, 0.0, 1.0), size=14.0)
        self.drone_center  = gl.GLScatterPlotItem(pos=zero_pos, color=(0.0, 1.0, 1.0, 1.0), size=16.0)

        self.planned_path_line = gl.GLLinePlotItem(pos=line_zeros, color=(1.0, 1.0, 1.0, 0.8), width=2.0, antialias=False)
        self.actual_path_line  = gl.GLLinePlotItem(pos=line_zeros, color=(1.0, 0.8, 0.0, 1.0), width=3.5, antialias=False)

        self.vel_vector_line   = gl.GLLinePlotItem(pos=line_zeros, color=(0.0, 1.0, 0.2, 1.0), width=3.0)
        self.thrust_vector_line = gl.GLLinePlotItem(pos=line_zeros, color=(1.0, 0.1, 0.1, 1.0), width=4.0)

        self.drone_arms = gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32), color=(0.9, 0.9, 0.9, 1.0), width=4.0)
        self.propellers = gl.GLScatterPlotItem(pos=np.zeros((4, 3), dtype=np.float32), color=(0.0, 1.0, 0.5, 1.0), size=12.0)

        for item in [self.home_marker, self.target_marker, self.drone_center,
                      self.planned_path_line, self.actual_path_line,
                      self.vel_vector_line, self.thrust_vector_line,
                      self.drone_arms, self.propellers]:
            self.view3d.addItem(item)

        t_pos = self.fc.target_pos
        self.target_marker.setData(pos=np.array([[t_pos[0], t_pos[1], -t_pos[2]]], dtype=np.float32))

    def on_camera_changed(self, index: int) -> None:
        self.camera_ctrl.set_mode(index)

    def cmd_takeoff(self) -> None:
        self.fc.arm_and_takeoff(target_z=-10.0)
        self.fc.set_target(5.0, 5.0, -10.0, yaw=0.0)
        t_pos = self.fc.target_pos
        self.target_marker.setData(pos=np.array([[t_pos[0], t_pos[1], -t_pos[2]]], dtype=np.float32))

    def cmd_pause(self) -> None:
        is_paused = self.sim_thread.toggle_pause()
        self.btn_pause.setText("▶️ RESUME SIM" if is_paused else "⏸️ PAUSE SIM")

    def on_telemetry_received(self, data: TelemetryData) -> None:
        self.latest_telemetry = data

    def render_gui_60hz(self) -> None:
        data = self.latest_telemetry
        pos = data.position

        if np.any(np.isnan(pos)) or np.any(np.isinf(pos)): return

        self.frame_count += 1
        euler = np.degrees(data.orientation)
        vel = data.velocity
        h_speed = float(np.linalg.norm(vel[0:2]))
        v_speed = float(-vel[2])

        self.flight_history_pos.append([pos[0], pos[1], -pos[2], data.sim_time, h_speed, pos[2]])
        if len(self.flight_history_pos) > self.MAX_POINTS:
            self.flight_history_pos = self.flight_history_pos[-self.MAX_POINTS:]

        des_pos = data.desired_pos
        self.planned_path_history.append([des_pos[0], des_pos[1], -des_pos[2]])
        if len(self.planned_path_history) > self.MAX_POINTS:
            self.planned_path_history = self.planned_path_history[-self.MAX_POINTS:]

        # 1. CẬP NHẬT MÔ HÌNH DRONE CHỮ X 3D
        drone_center_3d = np.array([[pos[0], pos[1], -pos[2]]], dtype=np.float32)
        self.drone_center.setData(pos=drone_center_3d)

        phi, theta, psi = data.orientation[0], data.orientation[1], data.orientation[2]
        L = 0.35
        arm_nodes_body = np.array([[L*0.707, L*0.707, 0], [-L*0.707, -L*0.707, 0], 
                                   [L*0.707, -L*0.707, 0], [-L*0.707, L*0.707, 0]], dtype=np.float32)

        c_p, s_p = np.cos(phi), np.sin(phi)
        c_t, s_t = np.cos(theta), np.sin(theta)
        c_s, s_s = np.cos(psi), np.sin(psi)
        R = np.array([[c_t*c_s, s_p*s_t*c_s - c_p*s_s, c_p*s_t*c_s + s_p*s_s],
                      [c_t*s_s, s_p*s_t*s_s + c_p*c_s, c_p*s_t*s_s - s_p*c_s],
                      [-s_t,    s_p*c_t,               c_p*c_t]], dtype=np.float32)

        props_world = (R @ arm_nodes_body.T).T
        props_world[:, 2] = -props_world[:, 2]
        props_abs = drone_center_3d + props_world

        if self.frame_count % 2 == 0:
            self.drone_arms.setData(pos=np.array([props_abs[0], props_abs[1], props_abs[2], props_abs[3]], dtype=np.float32))
            avg_pwm = float(np.mean(data.pwms))
            self.prop_angle += (avg_pwm - 1000.0) / 1000.0 * 0.8
            self.propellers.setData(pos=props_abs.astype(np.float32))

        # 2. VECTƠ VẬN TỐC & VECTƠ LỰC NÂNG
        vel_end = drone_center_3d[0] + np.array([vel[0], vel[1], -vel[2]], dtype=np.float32) * 1.2
        self.vel_vector_line.setData(pos=np.array([drone_center_3d[0], vel_end], dtype=np.float32))

        thrust_dir_body = np.array([0.0, 0.0, -data.actual_thrust * 0.15], dtype=np.float32)
        thrust_dir_world = R @ thrust_dir_body
        thrust_dir_world[2] = -thrust_dir_world[2]
        
        thrust_end = (drone_center_3d[0] + thrust_dir_world).reshape(-1)
        self.thrust_vector_line.setData(pos=np.array([drone_center_3d[0], thrust_end], dtype=np.float32))

        # 3. ĐƯỜNG BAY
        act_arr = np.array(self.flight_history_pos, dtype=np.float32)
        if len(act_arr) > 1: self.actual_path_line.setData(pos=act_arr[:, 0:3])

        plan_arr = np.array(self.planned_path_history, dtype=np.float32)
        if len(plan_arr) > 1: self.planned_path_line.setData(pos=plan_arr[:, 0:3])

        # 4. CAMERA & HUD UPDATE
        self.camera_ctrl.update(pos, data.orientation)

        self.lbl_alt.setText(f"Altitude (Z): {-pos[2]:.2f} m")
        self.lbl_pos_x.setText(f"Pos X: {pos[0]:.2f} m")
        self.lbl_pos_y.setText(f"Pos Y: {pos[1]:.2f} m")
        self.lbl_h_speed.setText(f"H-Speed: {h_speed:.2f} m/s")
        self.lbl_v_speed.setText(f"V-Speed (Vz): {v_speed:.2f} m/s")
        self.lbl_roll.setText(f"Roll: {euler[0]:.1f}°")
        self.lbl_pitch.setText(f"Pitch: {euler[1]:.1f}°")
        self.lbl_yaw.setText(f"Yaw: {euler[2]:.1f}°")
        self.lbl_thrust.setText(f"Thrust: {data.actual_thrust:.2f} N")
        self.lbl_pwms.setText(f"Motors: {[int(p) for p in data.pwms]}")
        self.lbl_loop.setText(f"Loop Time: {data.loop_time_ms:.2f} ms")
        self.lbl_mode.setText(f"Mode: {data.mode}")

    def generate_report(self) -> None:
        if len(self.flight_history_pos) == 0: return
        data = np.array(self.flight_history_pos)
        times, speeds, alts = data[:, 3], data[:, 4], -data[:, 5]

        idx = 1
        while os.path.exists(f"logs/flight_{idx:03d}.csv"): idx += 1
        csv_name = f"logs/flight_{idx:03d}.csv"
        np.savetxt(csv_name, data, delimiter=",", header="X,Y,Z_3d,Time,Speed,Z_ned", comments="")

        plt.figure(figsize=(10, 5))
        plt.subplot(1, 2, 1)
        plt.plot(times, alts, 'b-', label='Altitude (m)')
        plt.title('Flight Altitude Profile'); plt.xlabel('Time (s)'); plt.ylabel('Meters'); plt.grid(True)

        plt.subplot(1, 2, 2)
        plt.plot(times, speeds, 'r-', label='Speed (m/s)')
        plt.title('Flight Horizontal Speed Profile'); plt.xlabel('Time (s)'); plt.ylabel('m/s'); plt.grid(True)

        img_name = f"logs/reports/report_flight_{idx:03d}.png"
        plt.tight_layout(); plt.savefig(img_name); plt.close()
        print(f"📄 [FLIGHT RECORDER] Đã lưu báo cáo: {csv_name} & {img_name}")

    def closeEvent(self, event) -> None:
        self.sim_thread.stop()
        event.accept()
