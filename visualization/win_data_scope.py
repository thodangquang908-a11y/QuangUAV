import collections
import numpy as np
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
from interfaces.shared_state import shared_bus

class DataScopeWindow(QMainWindow):
    """Cửa sổ 3: Data Scope - Tích hợp Legend Box Chú Thích Đường Đồ Thị Chuyên Nghiệp."""
    BUFFER_SIZE = 200

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("QuangUAV - Real-Time Data Scope (PX4 Telemetry Plots)")
        self.resize(520, 720)
        self.move(1290, 50)

        self.time_buf    = collections.deque(maxlen=self.BUFFER_SIZE)
        self.alt_buf     = collections.deque(maxlen=self.BUFFER_SIZE)
        self.des_alt_buf = collections.deque(maxlen=self.BUFFER_SIZE)
        
        self.roll_buf    = collections.deque(maxlen=self.BUFFER_SIZE)
        self.pitch_buf   = collections.deque(maxlen=self.BUFFER_SIZE)
        self.yaw_buf     = collections.deque(maxlen=self.BUFFER_SIZE)
        
        self.m1_buf      = collections.deque(maxlen=self.BUFFER_SIZE)
        self.m2_buf      = collections.deque(maxlen=self.BUFFER_SIZE)
        self.m3_buf      = collections.deque(maxlen=self.BUFFER_SIZE)
        self.m4_buf      = collections.deque(maxlen=self.BUFFER_SIZE)

        self.init_ui()

        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.update_gui_30hz)
        self.render_timer.start(33)

    def init_ui(self) -> None:
        pg.setConfigOption('background', '#111118')
        pg.setConfigOption('foreground', '#dcdde1')
        pg.setConfigOptions(antialias=False)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # 1. Altitude Plot + Legend
        self.plot_alt = pg.PlotWidget(title="Altitude Profile (m)")
        self.plot_alt.showGrid(x=True, y=True, alpha=0.3)
        self.legend_alt = self.plot_alt.addLegend(offset=(-10, 10))
        self.curve_alt = self.plot_alt.plot(pen=pg.mkPen('#00a8ff', width=2), name="Actual Altitude")
        self.curve_des_alt = self.plot_alt.plot(pen=pg.mkPen('#4cd137', width=1.5, style=Qt.DashLine), name="Target Altitude")
        layout.addWidget(self.plot_alt)

        # 2. Attitude Angles Plot + Legend
        self.plot_euler = pg.PlotWidget(title="Attitude Angles (deg)")
        self.plot_euler.showGrid(x=True, y=True, alpha=0.3)
        self.legend_euler = self.plot_euler.addLegend(offset=(-10, 10))
        self.curve_roll  = self.plot_euler.plot(pen=pg.mkPen('#e84118', width=1.5), name="Roll")
        self.curve_pitch = self.plot_euler.plot(pen=pg.mkPen('#fbc531', width=1.5), name="Pitch")
        self.curve_yaw   = self.plot_euler.plot(pen=pg.mkPen('#9c88ff', width=1.5), name="Yaw")
        layout.addWidget(self.plot_euler)

        # 3. Motor PWM Outputs Plot + Legend
        self.plot_motors = pg.PlotWidget(title="Motor PWM Outputs (µs)")
        self.plot_motors.showGrid(x=True, y=True, alpha=0.3)
        self.legend_motors = self.plot_motors.addLegend(offset=(-10, 10))
        self.curve_m1 = self.plot_motors.plot(pen=pg.mkPen('#00a8ff', width=1.5), name="M1 (Front-Right)")
        self.curve_m2 = self.plot_motors.plot(pen=pg.mkPen('#fbc531', width=1.5), name="M2 (Rear-Left)")
        self.curve_m3 = self.plot_motors.plot(pen=pg.mkPen('#4cd137', width=1.5), name="M3 (Front-Left)")
        self.curve_m4 = self.plot_motors.plot(pen=pg.mkPen('#e84118', width=1.5), name="M4 (Rear-Right)")
        layout.addWidget(self.plot_motors)

    def update_gui_30hz(self) -> None:
        data = shared_bus.get_snapshot()
        t = float(data["sim_time"])
        if t <= 0: return

        pos = data["position"]
        euler = np.degrees(data["orientation"])
        pwms = data["pwms"]

        self.time_buf.append(t)
        self.alt_buf.append(float(-pos[2]))
        self.des_alt_buf.append(float(-data["desired_pos"][2]))
        
        self.roll_buf.append(float(euler[0]))
        self.pitch_buf.append(float(euler[1]))
        self.yaw_buf.append(float(euler[2]))
        
        self.m1_buf.append(float(pwms[0]))
        self.m2_buf.append(float(pwms[1]))
        self.m3_buf.append(float(pwms[2]))
        self.m4_buf.append(float(pwms[3]))

        t_arr = np.array(self.time_buf)
        self.curve_alt.setData(t_arr, np.array(self.alt_buf))
        self.curve_des_alt.setData(t_arr, np.array(self.des_alt_buf))

        self.curve_roll.setData(t_arr, np.array(self.roll_buf))
        self.curve_pitch.setData(t_arr, np.array(self.pitch_buf))
        self.curve_yaw.setData(t_arr, np.array(self.yaw_buf))

        self.curve_m1.setData(t_arr, np.array(self.m1_buf))
        self.curve_m2.setData(t_arr, np.array(self.m2_buf))
        self.curve_m3.setData(t_arr, np.array(self.m3_buf))
        self.curve_m4.setData(t_arr, np.array(self.m4_buf))
