"""
The analytic theory is the reference; the code has to reproduce it.

Every test here checks a numerical result against something that was
derived by hand, so a regression in the engine shows up as a disagreement
with the mathematics rather than a changed number.
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flyingscreen import acoustics
from flyingscreen import components as comp
from flyingscreen import mission as mission_mod
from flyingscreen import sizing, sweep, tether
from flyingscreen.dynamics import (build_vehicle, quat_to_rot, rot_to_quat,
                                   hover_linearisation, rotor_forces,
                                   initial_state, derivatives)
from flyingscreen.params import DEFAULTS, clamp_params, derived
from flyingscreen.simulate import simulate_flight


def base():
    return dict(DEFAULTS)


# ---------------------------------------------------------------------------
# Layer 1: momentum theory
# ---------------------------------------------------------------------------
def test_induced_velocity_matches_momentum_theory():
    p = base()
    d = derived(p)
    m = 3.0
    T = m * p["g"]
    v_i = math.sqrt(T / (2.0 * p["rho"] * d.A))
    # T = 2 rho A v_i^2 must hold identically
    assert 2.0 * p["rho"] * d.A * v_i ** 2 == pytest.approx(T, rel=1e-12)


def test_rotor_point_reproduces_ideal_power_when_profile_drag_is_zero():
    """With every loss and correction switched off, the disk must be ideal."""
    p = base()
    p["Cd0"] = 1e-9
    p["kappa_ind"] = 1.0
    p["ige_enable"] = 0
    p["interference_enable"] = 0
    rp = comp.rotor_point(p, 10.0)
    assert rp.FM_effective == pytest.approx(1.0, rel=1e-5)
    assert rp.P_ideal == pytest.approx(10.0 * rp.v_induced, rel=1e-12)
    assert rp.kappa_total == pytest.approx(1.0, rel=1e-12)


def test_thrust_and_power_scale_as_omega_squared_and_cubed():
    p = base()
    rp = comp.rotor_point(p, 8.0)
    assert rp.k_T * rp.omega ** 2 == pytest.approx(rp.T, rel=1e-12)
    assert rp.k_P * rp.omega ** 3 == pytest.approx(rp.P_shaft, rel=1e-12)
    assert rp.k_Q * rp.omega ** 2 == pytest.approx(rp.Q, rel=1e-12)


# ---------------------------------------------------------------------------
# Layer 2: the closed-form closure
# ---------------------------------------------------------------------------
def test_coefficients_match_hand_calculation():
    """The worked example in the derivation: alpha 0.6366, beta 0.2271.

    The example states the usable specific energy directly, so the landing
    reserve is set to zero here.
    """
    p = base()
    p["soc_reserve"] = 0.0
    c = sizing.coefficients(p, area_p=0.0)
    assert c.alpha == pytest.approx(1 - 0.2 - 2 * p["g"] / 120.0, rel=1e-12)
    assert c.alpha == pytest.approx(0.6366, abs=5e-4)
    assert c.beta == pytest.approx(0.2271, abs=5e-4)


def test_landing_reserve_divides_the_usable_specific_energy():
    p = base()
    a = sizing.coefficients(dict(p, soc_reserve=0.0), area_p=0.0)
    b = sizing.coefficients(dict(p, soc_reserve=0.2), area_p=0.0)
    assert b.beta == pytest.approx(a.beta / 0.8, rel=1e-12)
    assert b.M0 == pytest.approx(
        derived(p).m_fix + (a.M0 - derived(p).m_fix) / 0.8, rel=1e-12)


def test_fold_location_and_height():
    p = base()
    p["soc_reserve"] = 0.0
    c = sizing.coefficients(p, area_p=0.0)
    w = sizing.payload_wall(p)
    assert w["m_star"] == pytest.approx(4 * c.alpha ** 2 / (9 * c.beta ** 2), rel=1e-12)
    # with p["soc_reserve"] = 0 as in the worked example
    assert w["M0_max"] == pytest.approx(4 * c.alpha ** 3 / (27 * c.beta ** 2), rel=1e-12)
    assert w["m_star"] == pytest.approx(3.493, abs=2e-3)
    assert w["M0_max"] == pytest.approx(0.741, abs=2e-3)


def test_roots_satisfy_the_closure_equation():
    p = base()
    p["m_screen"] = 0.25
    c = sizing.coefficients(p, area_p=0.0)
    res = sizing.solve_power_closure(c.alpha, c.beta, c.M0, 1.5)
    assert res.feasible
    for m in (res.m_light, res.m_heavy):
        assert c.alpha * m - c.beta * m ** 1.5 == pytest.approx(c.M0, rel=1e-8)
    assert res.m_light < res.m_star < res.m_heavy


def test_no_solution_above_the_wall():
    p = base()
    w = sizing.payload_wall(p)
    p["m_screen"] = p["m_screen"] + (w["m_screen_max"] - p["m_screen"]) + 0.05
    assert not sizing.solve_fixed_area(p).feasible


def test_exactly_at_the_wall_the_two_roots_merge():
    p = base()
    w = sizing.payload_wall(p)
    p["m_screen"] = w["m_screen_max"] - 1e-9
    r = sizing.solve_fixed_area(p)
    assert r.feasible
    assert r.closure["m_light"] == pytest.approx(r.closure["m_heavy"], rel=1e-3)
    assert r.m == pytest.approx(w["m_star"], rel=1e-3)


def test_wall_scales_as_inverse_square_of_endurance():
    p = base()
    a = sizing.payload_wall(p)["M0_max"]
    q = dict(p); q["t_f"] = 2 * p["t_f"]
    b = sizing.payload_wall(q)["M0_max"]
    assert b == pytest.approx(a / 4.0, rel=1e-9)


def test_sensitivity_exponents_are_the_analytic_ones():
    s = sizing.sensitivities(base())
    assert s["t_f"] == pytest.approx(-2.0, abs=1e-3)
    assert s["e_b_wh_kg"] == pytest.approx(2.0, abs=1e-3)
    assert s["FM"] == pytest.approx(2.0, abs=1e-3)
    assert s["eta"] == pytest.approx(2.0, abs=1e-3)
    assert s["D_rotor"] == pytest.approx(2.0, abs=1e-3)
    assert s["N_rotors"] == pytest.approx(1.0, abs=1e-3)
    assert s["rho"] == pytest.approx(1.0, abs=1e-3)


def test_eight_hour_wall_is_about_ten_grams():
    p = base()
    p["t_f"] = 8 * 3600.0
    w = sizing.payload_wall(p)
    assert 0.005 < w["M0_max"] < 0.02


# ---------------------------------------------------------------------------
# Constant disk loading removes the fold
# ---------------------------------------------------------------------------
def test_constant_disk_loading_is_linear_and_matches_the_fraction_formula():
    p = base()
    dp = sizing.solve_constant_dl(p)
    assert dp.feasible
    c = dp.coeffs
    m_expected = c["M0"] / (1 - c["f_s"] - c["gamma_m"] - c["gamma_b"])
    assert dp.m == pytest.approx(m_expected, rel=1e-9)
    assert dp.closure["has_fold"] is False


def test_constant_disk_loading_diverges_when_fractions_reach_one():
    p = base()
    p["t_f"] = 200000.0            # gamma_b grows without bound with time
    dp = sizing.solve_constant_dl(p)
    assert not dp.feasible
    assert "diverges" in dp.reason


def test_power_law_exponent_follows_three_minus_p_over_two():
    p = base()
    for pp in (0.0, 0.5, 1.0, 1.5):
        c = sizing.coefficients(p, area_p=pp)
        assert c.q == pytest.approx((3.0 - pp) / 2.0, rel=1e-12)


def test_fold_disappears_once_area_grows_at_least_as_fast_as_mass():
    p = base()
    p["m_screen"] = 0.2
    anchor = sizing.coefficients(p, area_p=0.0).anchor_mass
    assert sweep.endurance_wall(p, 0.0, anchor) is not None
    assert sweep.endurance_wall(p, 1.4, anchor) is None


# ---------------------------------------------------------------------------
# Numerical closure agrees with the closed form
# ---------------------------------------------------------------------------
def test_fixed_point_iteration_lands_on_the_analytic_root():
    p = base()
    p["m_screen"] = 0.2
    dp = sizing.solve_fixed_area(p)
    tr = sizing.fixed_point_trace(p, m0=0.5, iters=4000)
    assert tr["status"] in ("converged", "slow")
    assert tr["m_final"] == pytest.approx(dp.m, rel=1e-4)


def test_fixed_point_diverges_when_the_design_does_not_exist():
    p = base()
    p["t_f"] = 4 * 3600.0
    assert not sizing.solve_fixed_area(p).feasible
    tr = sizing.fixed_point_trace(p, m0=1.0, iters=4000)
    assert tr["status"] == "diverging"


def test_stability_criterion_agrees_with_being_on_the_light_branch():
    p = base()
    p["m_screen"] = 0.2
    dp = sizing.solve_fixed_area(p)
    assert dp.stable and dp.dmdm < 1.0
    c = sizing.coefficients(p, area_p=0.0)
    assert 1.5 * c.beta * math.sqrt(dp.m) < c.alpha


# ---------------------------------------------------------------------------
# Layer 3: components
# ---------------------------------------------------------------------------
def test_first_principles_closure_actually_closes():
    p = base()
    p["m_screen"] = 0.25
    p["t_f"] = 1800.0
    r = comp.close_first_principles(p)
    assert r["feasible"]
    b = r["breakdown"]
    total = b["m_fix"] + b["m_frame"] + b["m_prop"] + b["m_bat"]
    assert total == pytest.approx(b["m"], rel=1e-6)


def test_arm_wall_thickness_satisfies_the_stress_it_was_sized_for():
    p = base()
    arm = comp.arm_structure(p, 3.0)
    assert arm.sigma <= p["sigma_allow"] * 1e6 * (1 + 1e-9) or arm.driver != "stress"
    assert arm.wall >= p["wall_min"] - 1e-15


def test_bigger_rotors_lower_disk_loading_and_power():
    p = base()
    a = comp.breakdown(p, 2.5)
    q = dict(p); q["D_rotor"] = 0.6
    b = comp.breakdown(q, 2.5)
    assert b.disk_loading < a.disk_loading
    assert b.P_hover_elec < a.P_hover_elec


def test_surface_effects_help_and_fade_with_distance():
    """
    Cheeseman and Bennett go as (R / 4z)^2, so a small rotor at head height
    gets a few percent, not a few tens of percent.
    """
    p = base()
    near = comp.surface_effects(dict(p, ceiling_height=2.4))
    far = comp.surface_effects(dict(p, ceiling_height=6.0))
    assert near["k_surface"] > far["k_surface"] > 1.0
    assert near["k_surface"] < 1.20, "surface relief is being overstated"
    off = comp.surface_effects(dict(p, ige_enable=0))
    assert off["k_surface"] == 1.0


def test_surface_effect_lowers_induced_power():
    p = dict(base(), D_rotor=0.49, ceiling_height=2.4)
    with_ige = comp.rotor_point(p, 3.6)
    without = comp.rotor_point(dict(p, ige_enable=0), 3.6)
    assert with_ige.P_induced < without.P_induced
    assert with_ige.kappa_total < without.kappa_total


def test_rotor_interference_bites_only_when_rotors_are_close():
    p = base()
    tight = comp.interference_factor(dict(p, rotor_gap=1.02))
    loose = comp.interference_factor(dict(p, rotor_gap=1.5))
    assert tight["gap_ratio"] < loose["gap_ratio"]
    assert tight["kappa_int"] > loose["kappa_int"]
    assert loose["kappa_int"] == pytest.approx(1.0, abs=1e-12)
    assert tight["kappa_int"] < 1.08
    assert comp.interference_factor(dict(p, interference_enable=0))["kappa_int"] == 1.0


def test_room_air_is_recirculated_quickly():
    """The machine turns the room over in under a minute, so the still-air
    inflow assumption stops being true early in any session."""
    p = base()
    r = comp.recirculation(p, 0.754, 2.79)
    assert r["exchange_time"] < 60.0
    assert r["volume_flow"] == pytest.approx(0.754 * 2.79, rel=1e-12)


# ---------------------------------------------------------------------------
# Acoustics
# ---------------------------------------------------------------------------
def test_sound_power_to_pressure_conversion_is_eight_dB():
    """L_p(1 m) = L_WA - 10 log10(2 pi) over a reflecting plane."""
    got = acoustics.spl_at(83.0, 1.0, None)["total"]
    assert got == pytest.approx(83.0 - 7.98, abs=0.05)


def test_a_weighting_matches_the_standard_at_reference_points():
    assert acoustics.a_weighting(1000.0) == pytest.approx(0.0, abs=0.05)
    assert acoustics.a_weighting(100.0) == pytest.approx(-19.1, abs=0.4)
    assert acoustics.a_weighting(31.5) == pytest.approx(-39.4, abs=0.6)
    assert acoustics.a_weighting(10000.0) == pytest.approx(-2.5, abs=0.4)


def test_tip_speed_is_the_sixth_power_lever():
    a = acoustics.rotor_noise(50.0, 3.6, 4, 82.9)
    b = acoustics.rotor_noise(25.0, 3.6, 4, 82.9)
    assert a["spl_1m"] - b["spl_1m"] == pytest.approx(60 * math.log10(2), abs=1e-9)


def test_reverberant_field_does_not_fall_off_with_distance():
    room = acoustics.room_constant(5.0, 4.0, 2.4, 0.20)
    near = acoustics.spl_at(59.0, 1.0, room["R"])["total"]
    far = acoustics.spl_at(59.0, 3.5, room["R"])["total"]
    # In a room, tripling the distance is worth a couple of dB, not ten.
    assert 0.0 < near - far < 4.0
    free_near = acoustics.spl_at(59.0, 1.0, None)["total"]
    free_far = acoustics.spl_at(59.0, 3.5, None)["total"]
    assert free_near - free_far == pytest.approx(20 * math.log10(3.5), abs=1e-9)


def test_a_more_absorbent_room_is_quieter():
    r_hard = acoustics.room_constant(5.0, 4.0, 2.4, 0.10)["R"]
    r_soft = acoustics.room_constant(5.0, 4.0, 2.4, 0.45)["R"]
    assert r_soft > r_hard
    assert acoustics.spl_at(59.0, 2.0, r_soft)["total"] < \
        acoustics.spl_at(59.0, 2.0, r_hard)["total"]


def test_blade_passage_frequency():
    assert acoustics.blade_passage_frequency(963.0, 2) == pytest.approx(32.1, abs=0.1)


def test_acoustic_anchor_reproduces_published_drone_noise():
    """
    The declared sound power of four DJI aircraft, converted to pressure at
    1 m, must come back out of the model within the spread of the data.
    """
    cases = [  # mass, N, D, hover rpm, declared L_WA
        (0.249, 4, 0.152, 9000, 79.0),
        (0.595, 4, 0.183, 7000, 82.0),
        (0.895, 4, 0.239, 5500, 83.0),
    ]
    for m, N, D, rpm, LWA in cases:
        p = base()
        p["N_rotors"] = N
        p["D_rotor"] = D
        omega = rpm * 2 * math.pi / 60.0
        rp = comp.rotor_point(p, m * p["g"] / N, omega=omega)
        expected = LWA - 10 * math.log10(2 * math.pi)      # L_p at 1 m
        assert rp.spl_1m == pytest.approx(expected, abs=1.5)


def test_profile_drag_rises_at_low_reynolds_number():
    """A design pushed to very low tip speed must not get Cd0 for free."""
    p = base()
    fast = comp.rotor_point(p, 5.0, omega=600.0)
    slow = comp.rotor_point(p, 5.0, omega=90.0)
    assert slow.reynolds < fast.reynolds
    assert slow.cd0_effective > fast.cd0_effective
    assert slow.cd0_effective <= 3.0 * p["Cd0"] + 1e-12


def test_higher_solidity_lowers_tip_speed_and_noise():
    """
    More blade area holds the same thrust at a lower tip speed, and noise
    goes as the sixth power of tip speed, so solidity is the strongest lever
    on how loud the machine is.
    """
    p = base()
    p["tip_speed_auto"] = 1
    thin = comp.rotor_point(p, 3.2)
    q = dict(p); q["n_blades"] = 3; q["blade_AR"] = 6.0
    fat = comp.rotor_point(q, 3.2)
    assert fat.sigma > thin.sigma
    assert fat.v_tip < thin.v_tip
    assert fat.spl_1m < thin.spl_1m - 5.0


def test_motor_mass_grows_sublinearly_with_torque():
    """Cooling improves with size, so torque density is not constant."""
    p = base()
    m1 = comp.motor_mass_for_torque(p, 0.2)
    m2 = comp.motor_mass_for_torque(p, 2.0)
    assert m2 > m1
    assert m2 < 10.0 * m1
    q = dict(p); q["motor_scale_exp"] = 0.0
    assert comp.motor_mass_for_torque(q, 2.0) == \
        pytest.approx(10.0 * comp.motor_mass_for_torque(q, 0.2), rel=1e-9)


def test_rotor_diameter_has_an_interior_optimum():
    """
    Induced power wants big rotors, structure and motor torque want small
    ones, so a real minimum sits between the extremes rather than at an edge.
    """
    p = base()
    p["m_screen"] = 0.35
    best, best_m = None, float("inf")
    results = {}
    for D in (0.30, 0.40, 0.50, 0.60, 0.70):
        q = dict(p); q["D_rotor"] = D
        r = comp.close_first_principles(q)
        results[D] = r["m"] if r["feasible"] else None
        if r["feasible"] and r["m"] < best_m:
            best, best_m = D, r["m"]
    assert best is not None
    assert best not in (0.30, 0.70), "optimum sits on the edge: %s" % results


def test_guards_scale_with_circumference_not_area():
    p = base()
    a = comp.arm_structure(p, 2.0)
    q = dict(p); q["D_rotor"] = 2 * p["D_rotor"]
    b = comp.arm_structure(q, 2.0)
    assert b.m_guards == pytest.approx(2.0 * a.m_guards, rel=1e-9)


def test_optimal_tip_speed_beats_an_arbitrary_one():
    # Automatic tip speed would silently override the manual value, so the
    # comparison is only meaningful with it switched off.
    p = base()
    p["tip_speed_auto"] = 0
    best = comp.optimal_tip_speed(p, 6.0)
    assert best["feasible"] == 1.0
    for v in (40.0, 60.0, 90.0, 140.0):
        q = dict(p); q["tip_speed"] = v
        rp = comp.rotor_point(q, 6.0)
        assert rp.v_tip == pytest.approx(v, rel=1e-9)
        if not rp.stalled:
            assert best["P_shaft"] <= rp.P_shaft + 1e-9


# ---------------------------------------------------------------------------
# Layers 4 and 5: dynamics
# ---------------------------------------------------------------------------
def test_quaternion_rotation_roundtrip():
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = rng.standard_normal(4)
        q /= np.linalg.norm(q)
        R = quat_to_rot(q)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-10)
        q2 = rot_to_quat(R)
        assert np.allclose(quat_to_rot(q2), R, atol=1e-9)


def test_hover_allocation_gives_weight_and_no_torque():
    p = base()
    p["m_screen"] = 0.25
    veh = build_vehicle(p, 1.8)
    Om = np.full(veh.N, math.sqrt(veh.m * veh.g / (veh.N * veh.k_T)))
    T, tau = rotor_forces(veh, Om)
    assert T == pytest.approx(veh.m * veh.g, rel=1e-9)
    assert np.allclose(tau, 0.0, atol=1e-9)


def test_freefall_with_no_thrust():
    p = base()
    p["m_screen"] = 0.25
    veh = build_vehicle(p, 1.8)
    x = initial_state(veh, np.zeros(3))
    x[13:13 + veh.N] = 0.0
    dx = derivatives(veh, 0.0, x, np.zeros(veh.N), np.zeros(2),
                     np.zeros(3), np.zeros(3))
    assert dx[5] == pytest.approx(-veh.g, rel=1e-12)


def test_energy_state_falls_at_the_electrical_power():
    p = base()
    p["m_screen"] = 0.25
    veh = build_vehicle(p, 1.8)
    x = initial_state(veh, np.zeros(3))
    from flyingscreen.dynamics import electrical_power
    dx = derivatives(veh, 0.0, x, x[13:13 + veh.N], np.zeros(2),
                     np.zeros(3), np.zeros(3))
    assert dx[13 + veh.N] == pytest.approx(-electrical_power(veh, x[13:13 + veh.N]),
                                           rel=1e-12)


def test_hover_model_is_controllable():
    p = base()
    p["m_screen"] = 0.25
    veh = build_vehicle(p, 1.8)
    lin = hover_linearisation(veh)
    assert lin["controllable"]


def test_screen_drag_is_orientation_dependent():
    from flyingscreen.dynamics import aero_forces
    p = base()
    p["m_screen"] = 0.25
    veh = build_vehicle(p, 1.8)
    veh.A_frame = 0.0          # isolate the screen from the airframe drag
    R = np.eye(3)
    face_on, _, _ = aero_forces(veh, R, np.array([5.0, 0.0, 0.0]), np.zeros(2))
    edge_on, _, _ = aero_forces(veh, R, np.array([0.0, 5.0, 0.0]), np.zeros(2))
    assert np.linalg.norm(face_on) > 5.0 * np.linalg.norm(edge_on)


# ---------------------------------------------------------------------------
# Layer 6: control and simulation
# ---------------------------------------------------------------------------
def test_hovering_next_to_a_still_person_is_tracked_tightly():
    p = base()
    p["m_screen"] = 0.25
    p["t_f"] = 1800.0
    p["tip_speed"] = 75.0
    r = simulate_flight(p, 1.9, mission_kind="standing", t_window=6.0)
    assert r.ok
    assert r.e_track_rms < 0.05
    assert r.overhead == pytest.approx(1.0, abs=0.25)


def test_walking_costs_more_power_than_standing():
    p = base()
    p["m_screen"] = 0.25
    p["t_f"] = 1800.0
    p["tip_speed"] = 75.0
    still = simulate_flight(p, 1.9, mission_kind="standing", t_window=6.0)
    moving = simulate_flight(p, 1.9, mission_kind="walk_loop", t_window=6.0)
    assert still.ok and moving.ok
    assert moving.P_mean > still.P_mean


def test_mission_reference_feedforward_is_consistent():
    p = base()
    human = mission_mod.make_human(p, "walk_loop")
    ref = mission_mod.reference(p, human, 3.0)
    h = 1e-4
    r1 = mission_mod.desired_position(p, human, 3.0 + h)
    r0 = mission_mod.desired_position(p, human, 3.0 - h)
    assert np.allclose(ref["v"], (r1 - r0) / (2 * h), atol=1e-4)


# ---------------------------------------------------------------------------
# Tether
# ---------------------------------------------------------------------------
def test_tether_removes_the_endurance_term():
    p = base()
    a = tether.solve_tethered(p)
    q = dict(p); q["t_f"] = 100 * p["t_f"]
    b = tether.solve_tethered(q)
    assert a["feasible"] and b["feasible"]
    assert a["m"] == pytest.approx(b["m"], rel=1e-12)


def test_tether_conductor_mass_scales_as_length_squared_over_voltage_squared():
    p = base()
    p["tether_support"] = 1.0
    base_c = tether.tether_constants(p)
    q = dict(p); q["tether_len"] = 2 * p["tether_len"]
    assert tether.tether_constants(q)["k_mass_per_W"] == \
        pytest.approx(4 * base_c["k_mass_per_W"], rel=1e-12)
    q = dict(p); q["tether_V"] = 2 * p["tether_V"]
    assert tether.tether_constants(q)["k_mass_per_W"] == \
        pytest.approx(base_c["k_mass_per_W"] / 4, rel=1e-12)


def test_tether_beats_the_battery_at_long_endurance():
    p = base()
    p["t_f"] = 8 * 3600.0
    assert not sizing.solve_fixed_area(p).feasible
    assert tether.solve_tethered(p)["feasible"]


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
def test_specific_energy_conversion():
    p = base()
    p["e_b_wh_kg"] = 250.0
    p["dod"] = 1.0
    assert derived(p).e_b == pytest.approx(900000.0, rel=1e-12)


def test_clamp_keeps_rotor_count_even_and_in_range():
    p = clamp_params(dict(DEFAULTS, N_rotors=7.4))
    assert p["N_rotors"] % 2 == 0
    assert 4 <= p["N_rotors"] <= 12


# ---------------------------------------------------------------------------
# Regressions from the 10 September 2026 review
# ---------------------------------------------------------------------------
from flyingscreen import closure, payloads
from flyingscreen import params as params_mod
from flyingscreen.control import control_authority
from flyingscreen.dynamics import hover_speeds


def desk1w():
    return payloads.apply(
        params_mod.preset_params("DESK-1/W, full 1920x1080 desktop"), "pi_thinclient")


def test_component_root_search_finds_a_narrow_positive_island():
    # Near the fold the positive stretch of the component residual is
    # narrower than the old sampling ratio of 1.35, which reported "no mass
    # closes" for a design with two roots.
    p = dict(desk1w(), t_f=4280.05020145)
    r = comp.close_first_principles(p)
    assert r["feasible"] and r["status"] == "root_found"
    assert r["m"] == pytest.approx(4.5677791206, rel=1e-7)
    assert abs(r["residual"]) < 1e-9


def test_unsuccessful_root_search_is_not_called_a_proof():
    p = dict(desk1w(), t_f=6 * 3600.0)
    r = comp.close_first_principles(p)
    assert not r["feasible"]
    assert "not a proof" in r["reason"]


def test_torque_authority_is_the_balanced_differential_without_one_over_n():
    p = base()
    p["m_screen"] = 0.25
    veh = build_vehicle(p, 1.8)
    a = control_authority(veh)
    om2 = veh.m * veh.g / (veh.N * veh.k_T)
    delta = min(om2, veh.omega_max ** 2 - om2)
    for axis, key in ((1, "tau_roll"), (2, "tau_pitch"), (3, "tau_yaw")):
        assert a[key] == pytest.approx(delta * np.sum(np.abs(veh.alloc[axis])), rel=1e-6)


def test_hexacopter_losing_an_opposite_pair_loses_a_wrench_direction():
    p = base()
    p["N_rotors"] = 6.0
    p["m_screen"] = 0.25
    veh = build_vehicle(p, 1.8)
    rows = veh.alloc[:, [1, 2, 4, 5]]
    rows = rows / np.linalg.norm(rows, axis=1, keepdims=True)
    assert np.linalg.matrix_rank(rows) == 3


def test_screen_inertia_uses_the_panel_axes():
    # Panel of width w along body y and height h along body z: the slab adds
    # m w^2 / 12 more to J_xx than to J_yy, whatever the offsets.
    p = base()
    p["m_gimbal"] = 0.0
    arm = comp.arm_structure(p, 2.0)
    j0 = comp.inertia(dict(p, m_screen=0.0), 2.0, arm, 0.1)
    j1 = comp.inertia(dict(p, m_screen=0.3), 2.0, arm, 0.1)
    d_xx, d_yy = j1["Jxx"] - j0["Jxx"], j1["Jyy"] - j0["Jyy"]
    assert d_xx - d_yy == pytest.approx(0.3 * p["screen_w"] ** 2 / 12.0, rel=1e-9)


def test_forward_boom_gives_a_product_of_inertia_and_a_trimmed_hover():
    p = base()
    p["m_screen"] = 0.25
    p["screen_forward"] = 0.3
    p["screen_above"] = 1
    veh = build_vehicle(p, 1.9)
    assert abs(veh.J[0, 2]) > 1e-4
    assert np.allclose(veh.J, veh.J.T)
    om = hover_speeds(veh)
    wrench = veh.alloc @ om ** 2
    assert wrench[0] == pytest.approx(veh.m * veh.g, rel=1e-9)
    assert np.allclose(wrench[1:], 0.0, atol=1e-9)
    assert np.ptp(om) > 1e-3


def test_states_and_references_are_compared_at_the_same_instant():
    p = base()
    p["m_screen"] = 0.25
    p["tip_speed"] = 75.0
    r = simulate_flight(p, 1.9, mission_kind="standing", t_window=1.0, settle=0.0)
    assert r.t[0] == 0.0
    assert r.telemetry["e_track"][0] == pytest.approx(0.0, abs=1e-12)


def test_governor_keeps_out_of_a_head_that_walks_toward_it():
    # The keep-out boundary moves with the person. Testing n . v_g instead of
    # n . (v_g - c_dot) lets a stationary reference be walked into.
    p = base()
    radius, dt = 0.8, 0.004
    target = np.array([1.0, 0.0, 0.0])
    gov = mission_mod.ReferenceGovernor(p, target, v_max=2.5, a_max=5.0,
                                        tracking_margin=0.0)
    c = np.array([-0.5, 0.0, 0.0])
    cv = np.array([0.6, 0.0, 0.0])
    worst = float("inf")
    for _ in range(int(3.0 / dt)):
        gov.step(target, np.zeros(3), dt, keep_out=(c, radius),
                 center_velocity=cv, center_acceleration=np.zeros(3))
        c = c + cv * dt
        worst = min(worst, float(np.linalg.norm(gov.r - c)) - radius)
        assert np.linalg.norm(gov.a) <= 5.0 + 1e-6
        assert np.linalg.norm(gov.v) <= 2.5 + 1e-6
        assert gov.feasible
    assert worst > -1e-3


def test_closure_verifies_the_battery_it_reports():
    p = base()
    p["m_screen"] = 0.25
    p["tip_speed"] = 75.0
    p["t_f"] = 1200.0
    run = closure.close_with_simulation(p, mission_kind="standing", t_window=3.0,
                                        settle=1.0, keep_telemetry=False)
    assert run.converged and run.battery_ok
    assert run.energy_margin >= 0.0 and run.power_margin >= 0.0
    assert run.reserve_end >= p["soc_reserve"] - 1e-9
    assert run.sim_calls == len(run.trace)
    assert run.trace[-1]["kind"] == "verify"
    for t in run.trace:
        assert t["residual"] == pytest.approx(t["m_bat"] - t["m_bat_required"], abs=1e-15)


def test_tethered_component_closure_reports_what_it_checked():
    r = tether.close_tethered_first_principles(base())
    c = r["checks"]
    assert r["feasible"] == (c["converged"] and c["current_density_ok"])
    assert r["current_density"] <= tether.J_MAX_A_MM2 * (1 + 1e-9)
    b = comp.propulsion(base(), r["m"])
    p = base()
    need = (b.P_elec_max + derived(p).P_aux) / p["p_b_w_kg"]
    assert r["m_reserve"] >= need * (1.0 - 1e-9)


def test_a_carried_power_sized_tether_keeps_its_fold():
    p = base()
    p["tether_support"] = 1.0
    p["tether_len"] = 60.0
    p["tether_V"] = 24.0
    r = tether.solve_tethered(p)
    assert r["fold_retained"] and r["beta_tether"] > 0.0
    assert math.isfinite(r["closure"]["M0_max"])


def test_moving_a_panel_away_sharpens_pixels_but_shrinks_text():
    panel = payloads.BY_KEY["laptop156"]
    near, far = payloads.readability(panel, 0.6), payloads.readability(panel, 1.2)
    assert far["pixels_per_degree"] > near["pixels_per_degree"]
    assert far["cap_arcmin_at_100"] == pytest.approx(0.5 * near["cap_arcmin_at_100"], rel=1e-3)
    assert far["logical_w"] < near["logical_w"]


def test_inner_map_slope_is_below_one_for_the_shipped_submodels():
    # The inner-map uniqueness proposition needs 0 <= S'(m) <= 1 - eps. Check
    # it on the component models over the whole range the engine searches.
    p = desk1w()

    def S(m):
        dm = comp._dry_mass(p, m)
        return dm["m_frame"] + dm["m_prop"]

    h = 1e-4
    slopes = [(S(m * (1 + h)) - S(m * (1 - h))) / (2 * h * m)
              for m in np.geomspace(0.8, 50.0, 80)]
    assert min(slopes) >= 0.0
    assert max(slopes) < 0.9
