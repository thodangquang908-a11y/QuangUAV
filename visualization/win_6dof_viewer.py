import numpy as np
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QComboBox, QLabel
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph.opengl as gl
from visualization.camera_controller import CameraController
from interfaces.shared_state import shared_bus

class SixDOFViewerWindow(QMainWindow):
    """Cửa sổ 2: 6DoF Viewer 3D - Tối ưu vị trí OSD Label chống chồng đè chữ."""
    MAX_POINTS = 2000

    def __init__(self, flight_controller) -> None:
        super().__init__()
        self.fc = flight_controller
        
        self.setWindowTitle("QuangUAV - 6DoF Simulator Viewer 3D (Gazebo Style)")
        self.resize(850, 720)
        self.move(430, 50)
        
        self.flight_history_pos = []
        self.planned_path_history = []

        self.init_ui()
        self.init_3d_scene()
        self.camera_ctrl = CameraController(self.view3d)

        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.update_gui_30hz)
        self.render_timer.start(33)

    def init_ui(self) -> None:
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        self.combo_cam = QComboBox()
        self.combo_cam.addItems(["1. Free Camera", "2. Follow Camera", "3. Top-down View", "4. Cockpit FPV", "5. Orbit Camera 360°"])
        self.combo_cam.currentIndexChanged.connect(self.on_camera_changed)
        layout.addWidget(self.combo_cam)

        legend_lbl = QLabel("🟢 Planned 3D Path | 🟡 Actual Path | 🔴 Target Waypoint | 🔴🟢🔵 Body Axes (Xb, Yb, Zb)")
        legend_lbl.setStyleSheet("background-color: #1e272e; color: #f5f6fa; font-weight: bold; font-size: 11px; padding: 5px; border-radius: 4px;")
        legend_lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(legend_lbl)

        self.view3d = gl.GLViewWidget()
        self.view3d.setCameraPosition(distance=35, elevation=35, azimuth=45)
        layout.addWidget(self.view3d)

    def init_3d_scene(self) -> None:
        self.view3d.setBackgroundColor((0.05, 0.05, 0.1, 1.0))
        
        grid = gl.GLGridItem()
        grid.setSize(80, 80, 1); grid.setSpacing(2, 2, 1)
        self.view3d.addItem(grid)

        world_axes = gl.GLAxisItem()
        world_axes.setSize(x=5.0, y=5.0, z=5.0)
        self.view3d.addItem(world_axes)

        zero_pos = np.zeros((1, 3), dtype=np.float32)
        line_zeros = np.zeros((2, 3), dtype=np.float32)

        self.home_marker   = gl.GLScatterPlotItem(pos=zero_pos, color=(0.0, 0.4, 1.0, 1.0), size=14.0)
        self.target_marker = gl.GLScatterPlotItem(pos=zero_pos, color=(1.0, 0.0, 0.0, 1.0), size=18.0)
        self.drone_center  = gl.GLScatterPlotItem(pos=zero_pos, color=(0.0, 1.0, 1.0, 1.0), size=18.0)

        # Body Axes (Red: Xb, Green: Yb, Blue: Zb)
        self.body_axis_x = gl.GLLinePlotItem(pos=line_zeros, color=(1.0, 0.1, 0.1, 1.0), width=5.0)
        self.body_axis_y = gl.GLLinePlotItem(pos=line_zeros, color=(0.1, 1.0, 0.1, 1.0), width=5.0)
        self.body_axis_z = gl.GLLinePlotItem(pos=line_zeros, color=(0.1, 0.6, 1.0, 1.0), width=5.0)

        self.planned_path_line = gl.GLLinePlotItem(pos=line_zeros, color=(0.0, 1.0, 0.4, 0.9), width=2.5)
        self.actual_path_line  = gl.GLLinePlotItem(pos=line_zeros, color=(1.0, 0.8, 0.0, 1.0), width=3.5)

        self.drone_arms = gl.GLLinePlotItem(pos=np.zeros((4, 3), dtype=np.float32), color=(0.9, 0.9, 0.9, 1.0), width=4.0)
        self.propellers = gl.GLScatterPlotItem(pos=np.zeros((4, 3), dtype=np.float32), color=(0.0, 1.0, 0.5, 1.0), size=12.0)

        # 1. NHÃN DRONE & TARGET CÓ OFFSET KHOẢNG CÁCH (KHÔNG TRÙNG NHAU)
        self.txt_drone_label  = gl.GLTextItem(pos=[0, 0, 0], text="DRONE", color=(0, 255, 255, 255))
        self.txt_target_label = gl.GLTextItem(pos=[0, 0, 0], text="TARGET", color=(255, 100, 100, 255))
        
        # 2. CỘT CẤU TRÚC CAO ĐỘ (ALTITUDE TOWER) ĐẶT RIÊNG Ở MÉP NGOÀI BẢN ĐỒ X=-20, Y=-20
        self.txt_z_0m  = gl.GLTextItem(pos=[-20, -20, 0.2], text="-- 0m (GND)", color=(180, 180, 180, 200))
        self.txt_z_5m  = gl.GLTextItem(pos=[-20, -20, 5.0], text="-- 5m", color=(180, 180, 180, 200))
        self.txt_z_10m = gl.GLTextItem(pos=[-20, -20, 10.0], text="-- 10m (Ref Z)", color=(0, 255, 120, 255))
        self.txt_z_15m = gl.GLTextItem(pos=[-20, -20, 15.0], text="-- 15m", color=(180, 180, 180, 200))

        # Cột chỉ dẫn Z nét đứt đặt ngoài biên
        z_tower_line = np.array([[-20, -20, 0], [-20, -20, 20]], dtype=np.float32)
        self.alt_tower = gl.GLLinePlotItem(pos=z_tower_line, color=(0.4, 0.4, 0.5, 0.6), width=2.0)

        for item in [self.home_marker, self.target_marker, self.drone_center,
                      self.body_axis_x, self.body_axis_y, self.body_axis_z,
                      self.planned_path_line, self.actual_path_line,
                      self.drone_arms, self.propellers,
                      self.txt_drone_label, self.txt_target_label,
                      self.txt_z_0m, self.txt_z_5m, self.txt_z_10m, self.txt_z_15m,
                      self.alt_tower]:
            self.view3d.addItem(item)

    def on_camera_changed(self, index: int) -> None:
        self.camera_ctrl.set_mode(index)

    def update_gui_30hz(self) -> None:
        data = shared_bus.get_snapshot()
        pos = data["position"]
        if np.any(np.isnan(pos)) or np.any(np.isinf(pos)): return

        self.flight_history_pos.append([pos[0], pos[1], -pos[2]])
        if len(self.flight_history_pos) > self.MAX_POINTS: self.flight_history_pos.pop(0)

        des_pos = data["desired_pos"]
        self.planned_path_history.append([des_pos[0], des_pos[1], -des_pos[2]])
        if len(self.planned_path_history) > self.MAX_POINTS: self.planned_path_history.pop(0)

        drone_center_3d = np.array([[pos[0], pos[1], -pos[2]]], dtype=np.float32)
        self.drone_center.setData(pos=drone_center_3d)

        # CẬP NHẬT NHÃN VĂN BẢN VỚI KHOẢNG LÙI OFFSET RÕ RÀNG
        euler_deg = np.degrees(data["orientation"])
        # Drone label: Đẩy cao lên trên +1.8m, lệch phải +1.5m
        self.txt_drone_label.setData(
            pos=[pos[0] + 1.5, pos[1] + 1.5, -pos[2] + 1.8],
            text=f"UAV ({pos[0]:.1f}, {pos[1]:.1f}, {-pos[2]:.1f}m) | R:{euler_deg[0]:.0f}° P:{euler_deg[1]:.0f}°"
        )

        t_pos = data["target_pos"]
        self.target_marker.setData(pos=np.array([[t_pos[0], t_pos[1], -t_pos[2]]], dtype=np.float32))
        # Target label: Đẩy xuống phía dưới -1.8m, lệch trái -1.5m
        self.txt_target_label.setData(
            pos=[t_pos[0] - 1.5, t_pos[1] - 1.5, -t_pos[2] - 1.8],
            text=f"Target ({t_pos[0]:.1f}, {t_pos[1]:.1f}, {-t_pos[2]:.1f}m)"
        )

        phi, theta, psi = data["orientation"][0], data["orientation"][1], data["orientation"][2]
        L = 0.35
        arm_nodes = np.array([[L*0.707, L*0.707, 0], [-L*0.707, -L*0.707, 0], 
                              [L*0.707, -L*0.707, 0], [-L*0.707, L*0.707, 0]], dtype=np.float32)

        c_p, s_p = np.cos(phi), np.sin(phi)
        c_t, s_t = np.cos(theta), np.sin(theta)
        c_s, s_s = np.cos(psi), np.sin(psi)
        R = np.array([[c_t*c_s, s_p*s_t*c_s - c_p*s_s, c_p*s_t*c_s + s_p*s_s],
                      [c_t*s_s, s_p*s_t*s_s + c_p*c_s, c_p*s_t*s_s - s_p*c_s],
                      [-s_t,    s_p*c_t,               c_p*c_t]], dtype=np.float32)

        props_world = (R @ arm_nodes.T).T
        props_world[:, 2] = -props_world[:, 2]
        props_abs = drone_center_3d + props_world

        self.drone_arms.setData(pos=np.array([props_abs[0], props_abs[1], props_abs[2], props_abs[3]], dtype=np.float32))
        self.propellers.setData(pos=props_abs.astype(np.float32))

        axis_len = 1.2
        xb_world = R @ np.array([axis_len, 0, 0], dtype=np.float32); xb_world[2] = -xb_world[2]
        yb_world = R @ np.array([0, axis_len, 0], dtype=np.float32); yb_world[2] = -yb_world[2]
        zb_world = R @ np.array([0, 0, -axis_len], dtype=np.float32); zb_world[2] = -zb_world[2]

        self.body_axis_x.setData(pos=np.array([drone_center_3d[0], drone_center_3d[0] + xb_world], dtype=np.float32))
        self.body_axis_y.setData(pos=np.array([drone_center_3d[0], drone_center_3d[0] + yb_world], dtype=np.float32))
        self.body_axis_z.setData(pos=np.array([drone_center_3d[0], drone_center_3d[0] + zb_world], dtype=np.float32))

        act_arr = np.array(self.flight_history_pos, dtype=np.float32)
        if len(act_arr) > 1: self.actual_path_line.setData(pos=act_arr)

        plan_arr = np.array(self.planned_path_history, dtype=np.float32)
        if len(plan_arr) > 1: self.planned_path_line.setData(pos=plan_arr)

        self.camera_ctrl.update(pos, data["orientation"])
