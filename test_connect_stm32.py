import serial
import time
from datetime import datetime

PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    print(f"✅ Đã kết nối {PORT} @ {BAUDRATE}")
    print("Đang chờ dữ liệu từ STM32... (Ctrl+C để dừng)\n")

    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            try:
                text = data.decode('utf-8', errors='ignore')
                if text.strip():
                    print(f"[{now}] {text}", end='')
            except:
                print(f"[{now}] HEX: {data.hex(' ')}")
        time.sleep(0.005)

except serial.SerialException as e:
    print(f"❌ Lỗi: {e}")
except KeyboardInterrupt:
    print("\nĐã dừng.")
    ser.close()
