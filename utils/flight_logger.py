import csv
import os
import numpy as np

class FlightLogger:
    """Ghi nhật ký bay tốc độ cao ra file CSV để vẽ đồ thị và phân tích sau chuyến bay."""
    def __init__(self, filepath: str = "flight_data.csv") -> None:
        self.filepath = filepath
        self.file = open(self.filepath, mode='w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        header = [
            "Time_s", "True_X", "True_Y", "True_Z", "Est_X", "Est_Y", "Est_Z",
            "True_Roll", "True_Pitch", "True_Yaw", "Est_Roll", "Est_Pitch", "Est_Yaw",
            "PWM_M1", "PWM_M2", "PWM_M3", "PWM_M4", "Flight_Mode"
        ]
        self.writer.writerow(header)

    def log_step(self, t: float, true_state: dict, est_state: dict, pwms: np.ndarray, mode: str) -> None:
        row = [
            f"{t:.4f}",
            f"{true_state['position'][0]:.4f}", f"{true_state['position'][1]:.4f}", f"{true_state['position'][2]:.4f}",
            f"{est_state['position'][0]:.4f}",  f"{est_state['position'][1]:.4f}",  f"{est_state['position'][2]:.4f}",
            f"{true_state['orientation'][0]:.4f}", f"{true_state['orientation'][1]:.4f}", f"{true_state['orientation'][2]:.4f}",
            f"{est_state['orientation'][0]:.4f}",  f"{est_state['orientation'][1]:.4f}",  f"{est_state['orientation'][2]:.4f}",
            f"{pwms[0]:.3f}", f"{pwms[1]:.3f}", f"{pwms[2]:.3f}", f"{pwms[3]:.3f}", mode
        ]
        self.writer.writerow(row)

    def close(self) -> None:
        self.file.close()
        print(f"📁 Đã xuất nhật ký chuyến bay ra file: {os.path.abspath(self.filepath)}")
