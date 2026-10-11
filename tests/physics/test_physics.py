import numpy as np
import pytest
from physics.rigid_body_6dof import RigidBody6DoF


def test_hover_stable():
    rb = RigidBody6DoF(mass=1.2)
    rb.reset(initial_pos=np.array([0.0, 0.0, -5.0]))
    dt = 0.01
    # total thrust equals weight -> should hover (no accel)
    thrust = rb.mass * rb.g
    for _ in range(200):
        rb.step(dt, total_thrust=thrust, body_torques=np.zeros(3))
    state = rb.get_state_dict()
    # position should remain near -5.0 and orientation near zero
    assert abs(state['position'][2] + 5.0) < 0.05
    assert np.allclose(state['orientation'], np.zeros(3), atol=0.05)


def test_free_fall():
    rb = RigidBody6DoF(mass=1.2)
    rb.reset(initial_pos=np.array([0.0, 0.0, -5.0]))
    dt = 0.01
    # zero thrust = free fall -> z should increase towards 0 (ground)
    for _ in range(100):
        rb.step(dt, total_thrust=0.0, body_torques=np.zeros(3))
    state = rb.get_state_dict()
    assert state['position'][2] > -5.0


def test_constant_thrust_rises():
    rb = RigidBody6DoF(mass=1.2)
    rb.reset(initial_pos=np.array([0.0, 0.0, -5.0]))
    dt = 0.01
    # thrust greater than weight should cause z to decrease (move up away from ground)
    thrust = rb.mass * rb.g * 1.2
    for _ in range(100):
        rb.step(dt, total_thrust=thrust, body_torques=np.zeros(3))
    state = rb.get_state_dict()
    assert state['position'][2] < -5.0


def test_roll_torque_direction():
    rb = RigidBody6DoF(mass=1.2)
    rb.reset(initial_pos=np.array([0.0, 0.0, -1.0]))
    dt = 0.01
    # apply positive roll torque -> roll angle should increase (positive)
    torque = np.array([0.01, 0.0, 0.0])
    for _ in range(200):
        rb.step(dt, total_thrust=rb.mass * rb.g, body_torques=torque)
    state = rb.get_state_dict()
    assert state['orientation'][0] > 0.0


def test_pitch_torque_direction():
    rb = RigidBody6DoF(mass=1.2)
    rb.reset(initial_pos=np.array([0.0, 0.0, -1.0]))
    dt = 0.01
    # apply positive pitch torque -> pitch angle should increase (positive)
    torque = np.array([0.0, 0.01, 0.0])
    for _ in range(200):
        rb.step(dt, total_thrust=rb.mass * rb.g, body_torques=torque)
    state = rb.get_state_dict()
    assert state['orientation'][1] > 0.0


def test_yaw_torque_direction():
    rb = RigidBody6DoF(mass=1.2)
    rb.reset(initial_pos=np.array([0.0, 0.0, -1.0]))
    dt = 0.01
    # apply positive yaw torque -> yaw angle should change (sign depends on convention)
    torque = np.array([0.0, 0.0, 0.02])
    for _ in range(200):
        rb.step(dt, total_thrust=rb.mass * rb.g, body_torques=torque)
    state = rb.get_state_dict()
    assert abs(state['orientation'][2]) > 0.0
