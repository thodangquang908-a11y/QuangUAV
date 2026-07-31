import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, 
                             QLabel, QPushButton, QGridLayout, QGroupBox, QProgressBar)
from PyQt5.QtCore import QTimer
from interfaces.shared_state import shared_bus

class GCSPanelWindow(QMainWindow):
    """Cửa sổ 1: GCS Panel Telemetry HUD & Controls."""
    def __init__(self, flight_controller) -> None:
        super().__init__()
        self.fc = flight_controller
        
        self.setWindowTitle("QuangUAV - Ground Control Station (GCS Panel)")
        self.resize(380, 720)
        self.move(30, 50)
        
        self.init_ui()

        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.update_gui_30hz)
        self.render_timer.start(33)

    def init_ui(self) -> None:
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # System Health
        health_box = QGroupBox("🛡️ SYSTEM HEALTH & ENVIRONMENT")
        health_grid = QGridLayout()

        self.lbl_battery = QLabel("Battery: 100.0%")
        self.pbar_battery = QProgressBar(); self.pbar_battery.setValue(100)
        self.lbl_gps = QLabel("GPS: 3D FIX (18 Sats)")
        self.lbl_ekf = QLabel("EKF Status: OK (GOOD)")
        self.lbl_cpu = QLabel("CPU Load: 12.5%")
        self.lbl_flight_time = QLabel("Flight Time: 0.0 s")

        for lbl in [self.lbl_battery, self.lbl_gps, self.lbl_ekf, self.lbl_cpu, self.lbl_flight_time]:
            lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #34495e;")

        health_grid.addWidget(self.lbl_battery, 0, 0); health_grid.addWidget(self.pbar_battery, 0, 1)
        health_grid.addWidget(self.lbl_gps, 1, 0);     health_grid.addWidget(self.lbl_ekf, 1, 1)
        health_grid.addWidget(self.lbl_cpu, 2, 0);     health_grid.addWidget(self.lbl_flight_time, 2, 1)
        health_box.setLayout(health_grid)
        layout.addWidget(health_box)

        # Telemetry HUD
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
        layout.addWidget(hud_box)

        # Commands
        ctrl_box = QGroupBox("🕹️ FLIGHT COMMANDS")
        ctrl_layout = QVBoxLayout()
        
        self.btn_takeoff = QPushButton("🚀 ARM & TAKEOFF")
        self.btn_takeoff.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold; padding: 8px;")
        self.btn_takeoff.clicked.connect(self.cmd_takeoff)

        self.btn_pause = QPushButton("⏸️ PAUSE SIM")
        self.btn_pause.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold; padding: 8px;")
        self.btn_pause.clicked.connect(self.cmd_pause)
        
        ctrl_layout.addWidget(self.btn_takeoff)
        ctrl_layout.addWidget(self.btn_pause)
        ctrl_box.setLayout(ctrl_layout)
        layout.addWidget(ctrl_box)
        layout.addStretch()

    def cmd_takeoff(self) -> None:
        shared_bus.paused = False
        self.btn_pause.setText("⏸️ PAUSE SIM")
        self.fc.arm_and_takeoff(target_z=-10.0)

    def cmd_pause(self) -> None:
        shared_bus.paused = not shared_bus.paused
        self.btn_pause.setText("▶️ RESUME SIM" if shared_bus.paused else "⏸️ PAUSE SIM")

    def update_gui_30hz(self) -> None:
        data = shared_bus.get_snapshot()
        pos = data["position"]
        euler = np.degrees(data["orientation"])
        vel = data["velocity"]
        h_speed = float(np.linalg.norm(vel[0:2]))
        v_speed = float(-vel[2])

        self.lbl_battery.setText(f"Battery: {data['battery_pct']:.1f}%")
        self.pbar_battery.setValue(int(data['battery_pct']))
        self.lbl_cpu.setText(f"CPU Load: {data['cpu_load']:.1f}%")
        self.lbl_flight_time.setText(f"Flight Time: {data['sim_time']:.1f} s")

        self.lbl_alt.setText(f"Altitude (Z): {-pos[2]:.2f} m")
        self.lbl_pos_x.setText(f"Pos X: {pos[0]:.2f} m")
        self.lbl_pos_y.setText(f"Pos Y: {pos[1]:.2f} m")
        self.lbl_h_speed.setText(f"H-Speed: {h_speed:.2f} m/s")
        self.lbl_v_speed.setText(f"V-Speed (Vz): {v_speed:.2f} m/s")
        self.lbl_roll.setText(f"Roll: {euler[0]:.1f}°")
        self.lbl_pitch.setText(f"Pitch: {euler[1]:.1f}°")
        self.lbl_yaw.setText(f"Yaw: {euler[2]:.1f}°")
        self.lbl_thrust.setText(f"Thrust: {data['actual_thrust']:.2f} N")
        self.lbl_pwms.setText(f"Motors: {[int(p) for p in data['pwms']]}")
        self.lbl_loop.setText(f"Loop Time: {data['loop_time_ms']:.2f} ms")
        self.lbl_mode.setText(f"Mode: {data['mode']}")
