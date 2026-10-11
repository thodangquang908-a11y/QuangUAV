import os
import sys
import numpy as np

# Ensure project root is on sys.path when running tests
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from controllers.control_allocation import ControlAllocator
from flight_controller.flight_controller import FlightController


def _dummy_true_state():
    return {
        "position": np.zeros(3),
        "velocity": np.zeros(3),
        "orientation": np.zeros(3),
        "angular_rate": np.zeros(3),
    }


def _fc_with_fixed_u(total_thrust: float, torques: np.ndarray) -> FlightController:
    fc = FlightController(mass=1.2)

    def fake_compute(target_state, current_state, dt, sim_time):
        return np.array([total_thrust, torques[0], torques[1], torques[2]])

    fc.runtime.compute_control_signals = fake_compute
    return fc


def test_roll_mapping_consistent_between_allocators():
    """Positive roll torque should increase M2 & M3, decrease M1 & M4 for both mixers."""
    T = 20.0
    taus = np.array([0.5, 0.0, 0.0])

    # FlightController mixer (inline)
    fc = _fc_with_fixed_u(T, taus)
    _, _, pwms_fc = fc.update_sensors_and_estimate(_dummy_true_state(), {}, 0.0)

    # ControlAllocator mixer
    ca = ControlAllocator()
    pwms_ca = ca.allocate(T, taus)

    # For positive roll: M2 and M3 should be higher than M1 and M4
    assert pwms_fc[1] > pwms_fc[0]
    assert pwms_fc[2] > pwms_fc[3]

    assert pwms_ca[1] > pwms_ca[0]
    assert pwms_ca[2] > pwms_ca[3]


def test_yaw_mapping_is_opposite_between_allocators():
    """Detect that FlightController and ControlAllocator use opposite sign for yaw mapping."""
    T = 20.0
    taus = np.array([0.0, 0.0, 0.2])

    fc = _fc_with_fixed_u(T, taus)
    _, _, pwms_fc = fc.update_sensors_and_estimate(_dummy_true_state(), {}, 0.0)

    ca = ControlAllocator()
    pwms_ca = ca.allocate(T, taus)

    # FlightController inline mixer: positive yaw increases M3 & M4
    assert pwms_fc[2] > pwms_fc[0]
    assert pwms_fc[3] > pwms_fc[1]

    # ControlAllocator math: positive yaw increases M1 & M2 (opposite)
    assert pwms_ca[0] > pwms_ca[2]
    assert pwms_ca[1] > pwms_ca[3]
