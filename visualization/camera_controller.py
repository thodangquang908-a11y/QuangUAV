import time
import numpy as np
from PyQt5.QtGui import QVector3D

class CameraController:
    """Smart Camera 3D Controller: Tích hợp Smooth Follow, Orbit Mode & FPV."""
    MODE_FREE = 0
    MODE_FOLLOW = 1
    MODE_TOP = 2
    MODE_FPV = 3
    MODE_ORBIT = 4

    def __init__(self, gl_view) -> None:
        self.view = gl_view
        self.mode = self.MODE_FREE

    def set_mode(self, mode: int) -> None:
        self.mode = mode

    def update(self, drone_pos: np.ndarray, drone_euler: np.ndarray) -> None:
        x, y, z_3d = float(drone_pos[0]), float(drone_pos[1]), float(-drone_pos[2])
        pitch, yaw = float(drone_euler[1]), float(drone_euler[2])

        if self.mode == self.MODE_FOLLOW:
            # Smooth Follow Behind
            self.view.opts['center'] = QVector3D(x, y, z_3d)
            self.view.setCameraPosition(distance=12, elevation=20, azimuth=np.degrees(yaw) - 90)
        elif self.mode == self.MODE_TOP:
            # Top-down View
            self.view.opts['center'] = QVector3D(x, y, z_3d)
            self.view.setCameraPosition(distance=25, elevation=90, azimuth=0)
        elif self.mode == self.MODE_FPV:
            # Cockpit FPV Camera
            self.view.opts['center'] = QVector3D(x + 0.8 * np.cos(yaw), y + 0.8 * np.sin(yaw), z_3d)
            self.view.setCameraPosition(distance=2, elevation=-np.degrees(pitch), azimuth=np.degrees(yaw) - 90)
        elif self.mode == self.MODE_ORBIT:
            # Giai đoạn 5.1: Orbit Mode quay 360 độ tự động quanh drone
            azimuth = (time.time() * 25.0) % 360.0
            self.view.opts['center'] = QVector3D(x, y, z_3d)
            self.view.setCameraPosition(distance=15, elevation=25, azimuth=azimuth)
