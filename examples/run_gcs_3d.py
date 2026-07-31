import os
import sys

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["PYOPENGL_PLATFORM"] = "glx"

from PyQt5.QtWidgets import QApplication
from flight_controller.flight_controller import FlightController
from physics.rigid_body_6dof import RigidBody6DoF
from visualization.sim_thread import SimulationThread

from visualization.win_gcs_panel import GCSPanelWindow
from visualization.win_6dof_viewer import SixDOFViewerWindow
from visualization.win_data_scope import DataScopeWindow

def main():
    app = QApplication(sys.argv)
    
    fc = FlightController(mass=1.2)
    physics = RigidBody6DoF(mass=1.2, arm_length=0.25)
    
    sim_thread = SimulationThread(fc, physics)
    
    win_gcs   = GCSPanelWindow(fc)
    win_6dof  = SixDOFViewerWindow(fc)
    win_scope = DataScopeWindow()
    
    win_gcs.show()
    win_6dof.show()
    win_scope.show()
    
    sim_thread.start()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
