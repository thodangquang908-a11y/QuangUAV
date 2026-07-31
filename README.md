# QuangUAV - 6DoF Quadcopter Flight Simulator

A Python-based High-Fidelity 6-Degree-of-Freedom (6DoF) Quadcopter Flight Simulator and Ground Control Station (GCS). Designed following the architecture of professional flight stacks like PX4 and Gazebo.

## 🚀 Features
* **Decoupled Architecture:** 400Hz Physics Engine & Controller running independently from 30Hz GUI via an Atomic Shared State Bus.
* **Flight Controller:** Cascade PID Controller with Attitude, Rate, and Position loops.
* **Trajectory Generator:** Automated 3D Figure-8 (Lissajous curve) continuous path generation.
* **Motor Mixer:** Quadcopter X-frame matrix mixer with motor saturation limits.
* **3-Window System Dashboard:**
  * **GCS Panel:** Telemetry HUD, Flight Commands, and System Health.
  * **6DoF Viewer 3D:** Real-time Gazebo-style 3D rendering with OpenGL, Body Axes, and Trajectory paths.
  * **Data Scope:** Real-time PyQtGraph plotting for Altitude, Attitude Angles, and Motor PWMs.

## 🛠️ Tech Stack
* **Language:** Python 3.x
* **GUI & Graphics:** PyQt5, PyQtGraph, PyOpenGL
* **Math & Physics:** NumPy, SciPy

## 🏃‍♂️ How to Run
```bash
# Clone the repository
git clone [https://github.com/ThoDangQuang/QuangUAV.git](https://github.com/ThoDangQuang/QuangUAV.git)
cd QuangUAV

# Run the simulation
python3 examples/run_gcs_3d.py

# 1. Tạo file .gitignore
cat << 'EOF' > .gitignore
__pycache__/
*.py[cod]
*$py.class
venv/
env/
.env
.vscode/
.idea/
logs/
*.log
*.bin
*.csv
