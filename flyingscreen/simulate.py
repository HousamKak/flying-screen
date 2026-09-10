"""
The nonlinear simulator: integrate the coupled ODEs and account for energy.

Rather than integrating a whole work session step by step, the simulator
runs a representative window of the mission at full fidelity, measures the
mean electrical power over that window once the transient has decayed, and
extrapolates to the mission duration:

    E_mission = P_mean * t_f

P_mean comes from the controller working against drag, gusts and human
motion, through the hover-calibrated power law of the dynamics module. That
law has no inflow correction, so the figure is a near-hover estimate: it is
appropriate for following a person indoors and is not a model of climbs,
descents or aggressive manoeuvres.

Timing convention: every quantity recorded at step k refers to time
t_k = k dt, and compares the state x(t_k) with the references evaluated at
t_k. The controller is sampled at t_k and holds its command over the step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from . import mission as mission_mod
from .params import derived
from .control import (CascadedController, control_authority, desired_attitude,
                      screen_tilt_error)
from .dynamics import (E3, Vehicle, build_vehicle, derivatives, initial_state,
                       normalize_quat, quat_to_rot, euler_from_rot,
                       electrical_power, hover_speeds, rotor_clearance)


@dataclass
class SimResult:
    ok: bool
    reason: str = ""
    t: List[float] = field(default_factory=list)
    telemetry: Dict[str, List[float]] = field(default_factory=dict)
    trajectory: Dict[str, List[List[float]]] = field(default_factory=dict)
    P_mean: float = 0.0
    P_hover_ref: float = 0.0        # steady hover of this same rotor
    P_ideal_lumped: float = 0.0     # what the closed-form layer assumes
    P_peak: float = 0.0             # over the whole window, start included
    P_peak_settled: float = 0.0     # after the settling interval only
    energy_window: float = 0.0
    E_mission: float = 0.0
    e_track_rms: float = 0.0     # against the reference the controller was given
    e_track_max: float = 0.0
    e_track_max_all: float = 0.0   # over the whole window, start included
    tracking_margin: float = 0.0   # the governor's keep-out allowance for it
    margin_holds: bool = True      # e_track_max_all <= tracking_margin in this run
    e_lag_rms: float = 0.0       # against where the screen ideally belongs
    e_lag_max: float = 0.0
    tilt_rms_deg: float = 0.0
    tilt_max_deg: float = 0.0
    screen_tilt_rms_deg: float = 0.0
    screen_tilt_max_deg: float = 0.0
    saturation_fraction: float = 0.0
    keepout_fraction: float = 0.0   # time the governor's keep-out constraint acted
    clipped_fraction: float = 0.0   # time any governor limit (caps or keep-out) acted
    governor_infeasible_fraction: float = 0.0
    keepout_radius: float = 0.0     # reference keep-out radius about the head [m]
    started_inside: float = 0.0     # how far the ideal start lay inside it   [m]
    ref_keepout_margin_min: float = 0.0   # governed reference distance minus that radius
    ref_accel_max: float = 0.0      # peak acceleration the mission asks for [m/s^2]
    lam_needed: float = 0.0         # thrust margin that peak alone implies   [-]
    # Two clearance measures. The envelope gap is the distance from the head
    # to the sphere of radius `reach` around the centre of mass, which is
    # conservative. The rotor clearance is the distance from the head point to
    # the nearest oriented rotor disk, which is what a blade could touch.
    min_human_distance: float = 0.0     # envelope gap, conservative   [m]
    rotor_clearance_min: float = 0.0    # nearest rotor disk to the eye [m]
    soc_end: float = 1.0
    endurance_est: float = 0.0
    overhead: float = 1.0          # P_mean / ideal hover power
    constraints_ok: bool = True
    violations: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    config: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        d = dict(self.__dict__)
        return d


def simulate_flight(p: Dict[str, float], m: float, m_bat: Optional[float] = None,
                    mission_kind: str = "pacing", t_window: float = 30.0,
                    dt: float = 0.004, sample_hz: float = 25.0,
                    settle: float = 3.0, seed: int = 12345,
                    veh: Optional[Vehicle] = None,
                    governor: Optional[Dict[str, float]] = None,
                    substeps: int = 1) -> SimResult:
    """
    Integrate a window of the mission with RK4 at fixed step.

    Passing `governor` (a dict of v_max, a_max and optionally
    tracking_margin) puts a rate limited reference with a keep-out
    constraint between the human and the controller, which is what a real
    product does instead of chasing a face rigidly.
    """
    if veh is None:
        veh = build_vehicle(p, m, m_bat)
    human = mission_mod.make_human(p, mission_kind)
    wind = mission_mod.Wind(p, t_window, seed=seed)
    ctrl = CascadedController(p, veh)
    offset = np.asarray(veh.screen_offset, dtype=float)

    def ref_at(tt: float) -> Dict[str, Any]:
        return mission_mod.reference(p, human, tt, screen_offset=offset)

    ref0 = ref_at(0.0)
    N = veh.N
    n_steps = max(int(round(t_window / dt)), 1)
    stride = max(int(round(1.0 / (sample_hz * dt))), 1)
    d_safe = float(p["d_safe"])
    # The person's head, not their feet: the trajectory generator reports a
    # ground reference, and the thing we must not hit is at eye level.
    head_offset = np.array([0.0, 0.0, float(p["standoff_z"])])
    gov = None
    gov_cfg: Dict[str, float] = {}
    r_start = np.asarray(ref0["r"], dtype=float)
    started_inside = 0.0
    if governor:
        known = ("v_max", "a_max", "kp", "kd", "tracking_margin",
                 "yaw_rate_max", "yaw_acc_max", "j_max")
        gov_cfg = {k: float(v) for k, v in governor.items() if k in known}
        if "yaw_acc_max" not in gov_cfg:
            # Half the yaw authority the rotors have, the same reservation the
            # attitude gain rule makes on the other two axes.
            gov_cfg["yaw_acc_max"] = 0.5 * control_authority(veh)["alpha_yaw"]
        margin = float(gov_cfg.get("tracking_margin", 0.10))
        # If the ideal position is already inside the keep-out sphere (a
        # standoff shorter than the reach allows) the mission starts on the
        # sphere, radially out from the head. Starting inside and letting the
        # governor fling the reference out at a_max would measure the start,
        # not the design; the distance moved is reported instead.
        head0 = np.asarray(ref0["human"], dtype=float) + head_offset
        delta = r_start - head0
        dist = float(np.linalg.norm(delta))
        radius0 = d_safe + veh.reach + margin
        if dist < radius0:
            r_start = head0 + delta * (radius0 / max(dist, 1e-9))
            started_inside = radius0 - dist
        gov = mission_mod.ReferenceGovernor(p, r_start, yaw0=float(ref0["yaw"]),
                                            **gov_cfg)
        gov.v = np.asarray(ref0["v"], dtype=float).copy()
        gov.a = np.asarray(ref0["a"], dtype=float).copy()
        gov.j = np.asarray(ref0["j"], dtype=float).copy()
        gov.yaw_rate = float(ref0["yaw_rate"])
    keep_radius = d_safe + veh.reach + (gov.tracking_margin if gov else 0.0)
    # Trim for the reference at t = 0: the attitude, body rate and thrust
    # that the controller would ask for with zero error.
    F0 = veh.m * (np.asarray(ref0["a"], dtype=float) + veh.g * E3)
    R0, w0, _ = desired_attitude(veh, F0, ref0)
    x = initial_state(veh, r_start, v0=np.asarray(ref0["v"], dtype=float),
                      R0=R0, w0=w0, T0=float(np.linalg.norm(F0)))

    ts: List[float] = []
    tel: Dict[str, List[float]] = {k: [] for k in
                                   ("P", "soc", "e_track", "tilt_deg",
                                    "screen_tilt_deg", "T", "thrust_ratio",
                                    "omega_mean", "v_speed", "sat",
                                    "rotor_clearance")}
    traj = {"r": [], "r_h": [], "r_d": [], "euler": []}

    P_acc, P_cnt, P_peak, P_peak_all = 0.0, 0, 0.0, 0.0
    e_acc, e_cnt, e_max, e_all = 0.0, 0, 0.0, 0.0
    eraw_acc, eraw_max = 0.0, 0.0
    tilt_acc, tilt_max = 0.0, 0.0
    st_acc, st_max = 0.0, 0.0
    sat_cnt = 0
    blocked_cnt = 0
    clipped_cnt = 0
    infeasible_cnt = 0
    d_min = float("inf")
    clear_min = float("inf")
    ref_margin_min = float("inf")
    energy0 = x[13 + N]
    zero3 = np.zeros(3)

    def rhs(tt: float, xx: np.ndarray, u_om, u_g) -> np.ndarray:
        return derivatives(veh, tt, xx, u_om, u_g, wind(tt), zero3)

    a_ref_max = 0.0
    for k in range(n_steps):
        t = k * dt
        ref = ref_at(t)
        r_wanted = np.asarray(ref["r"], dtype=float)
        r_head = np.asarray(ref["human"], dtype=float) + head_offset
        if gov is not None:
            # The governed state at t is the state before this update; the
            # update computes the acceleration at t and advances to t + dt.
            r_g, v_g = gov.r.copy(), gov.v.copy()
            y_g, yr_g = gov.yaw, gov.yaw_rate
            gov.step(ref["r"], ref["v"], dt,
                     keep_out=(r_head, keep_radius),
                     center_velocity=ref["human_v"],
                     center_acceleration=ref["human_a"],
                     a_raw=ref["a"], j_raw=ref["j"])
            gov.step_yaw(ref["yaw"], ref["yaw_rate"], dt)
            ref = dict(ref, r=r_g, v=v_g, a=gov.a.copy(), j=gov.j.copy(),
                       yaw=y_g, yaw_rate=yr_g, yaw_acc=gov.yaw_acc)
            blocked_cnt += int(gov.blocked)
            clipped_cnt += int(gov.clipped)
            infeasible_cnt += int(not gov.feasible)
            ref_margin_min = min(ref_margin_min,
                                 float(np.linalg.norm(r_g - r_head)) - keep_radius)
        a_ref_max = max(a_ref_max, float(np.linalg.norm(ref["a"])))

        # -- measure the state at t against the references at t ------------
        r = x[0:3]
        R = quat_to_rot(x[6:10])
        err = float(np.linalg.norm(np.asarray(ref["r"]) - r))
        # What the reader sees: distance from where the screen ideally
        # belongs, which the governor deliberately lets go during a turn.
        err_raw = float(np.linalg.norm(r_wanted - r))
        if err > 25.0:
            return SimResult(ok=False, reason="tracking error exceeded 25 m at t = %.2f s: "
                                              "the design cannot follow this mission" % t)
        P = electrical_power(veh, x[13:13 + N])
        tilt = math.acos(max(-1.0, min(1.0, float(R[2, 2]))))
        st = screen_tilt_error(veh, x)
        d_min = min(d_min, float(np.linalg.norm(r - r_head)) - veh.reach)
        clear = rotor_clearance(veh, r, R, r_head)
        clear_min = min(clear_min, clear)
        P_peak_all = max(P_peak_all, P)
        e_all = max(e_all, err)

        out = ctrl(t, x, ref, dt)
        if t >= settle:
            P_acc += P; P_cnt += 1
            P_peak = max(P_peak, P)
            e_acc += err * err; e_cnt += 1; e_max = max(e_max, err)
            eraw_acc += err_raw * err_raw
            eraw_max = max(eraw_max, err_raw)
            tilt_acc += tilt * tilt; tilt_max = max(tilt_max, tilt)
            st_acc += st * st; st_max = max(st_max, st)
            if out.saturated:
                sat_cnt += 1

        if k % stride == 0:
            ts.append(t)
            tel["P"].append(P)
            tel["soc"].append(float(x[13 + N] / veh.E_max) if veh.E_max > 0 else 0.0)
            tel["e_track"].append(err)
            tel["tilt_deg"].append(math.degrees(tilt))
            tel["screen_tilt_deg"].append(math.degrees(st))
            T = float(veh.alloc[0] @ np.clip(x[13:13 + N], 0, None) ** 2)
            tel["T"].append(T)
            tel["thrust_ratio"].append(T / (veh.m * veh.g))
            tel["omega_mean"].append(float(np.mean(x[13:13 + N])))
            tel["v_speed"].append(float(np.linalg.norm(x[3:6])))
            tel["sat"].append(1.0 if out.saturated else 0.0)
            tel["rotor_clearance"].append(clear)
            traj["r"].append([float(v) for v in r])
            traj["r_h"].append([float(v) for v in np.asarray(ref["human"])])
            traj["r_d"].append([float(v) for v in np.asarray(ref["r"])])
            roll, pitch, yaw = euler_from_rot(R)
            traj["euler"].append([math.degrees(roll), math.degrees(pitch),
                                  math.degrees(yaw)])

        # -- advance to t + dt with the command held -----------------------
        # The controller is sampled once per dt; `substeps` RK4 steps of
        # dt / substeps integrate the plant under that held command, which
        # refines the integration without changing the sampled-data loop.
        u_om, u_g = out.Om_cmd, out.tau_g
        h = dt / max(int(substeps), 1)
        for j in range(max(int(substeps), 1)):
            tj = t + j * h
            k1 = rhs(tj, x, u_om, u_g)
            k2 = rhs(tj + 0.5 * h, x + 0.5 * h * k1, u_om, u_g)
            k3 = rhs(tj + 0.5 * h, x + 0.5 * h * k2, u_om, u_g)
            k4 = rhs(tj + h, x + h * k3, u_om, u_g)
            x = normalize_quat(x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4))
        if not np.all(np.isfinite(x)):
            return SimResult(ok=False, reason="integration diverged: the controller "
                                              "lost the vehicle at t = %.2f s" % t)

    P_mean = P_acc / max(P_cnt, 1)
    d = derived(p)
    # Baseline is steady hover of *this* rotor, so the overhead measures what
    # the mission costs above hovering rather than a mismatch between the
    # lumped figure of merit and the one the blade model produces.
    om_trim = hover_speeds(veh)
    P_hover_ref = float(np.sum(veh.k_P * om_trim ** 3)) / veh.eta + d.P_aux
    P_ideal_lumped = (m * veh.g) ** 1.5 / (float(p["FM"]) * float(p["eta"]) *
                                           math.sqrt(2.0 * veh.rho * d.A)) + d.P_aux
    t_f = float(p["t_f"])
    E_mission = P_mean * t_f
    energy_window = float(energy0 - x[13 + N])

    res = SimResult(
        ok=True, t=ts, telemetry=tel, trajectory=traj, P_mean=P_mean,
        P_hover_ref=P_hover_ref, P_ideal_lumped=P_ideal_lumped,
        P_peak=P_peak_all, P_peak_settled=P_peak, energy_window=energy_window,
        E_mission=E_mission,
        e_track_rms=math.sqrt(e_acc / max(e_cnt, 1)), e_track_max=e_max,
        e_track_max_all=e_all,
        tracking_margin=gov.tracking_margin if gov is not None else 0.0,
        margin_holds=bool(gov is None or e_all <= gov.tracking_margin),
        e_lag_rms=math.sqrt(eraw_acc / max(e_cnt, 1)), e_lag_max=eraw_max,
        tilt_rms_deg=math.degrees(math.sqrt(tilt_acc / max(e_cnt, 1))),
        tilt_max_deg=math.degrees(tilt_max),
        screen_tilt_rms_deg=math.degrees(math.sqrt(st_acc / max(e_cnt, 1))),
        screen_tilt_max_deg=math.degrees(st_max),
        saturation_fraction=sat_cnt / max(e_cnt, 1),
        ref_accel_max=a_ref_max,
        keepout_fraction=blocked_cnt / max(n_steps, 1),
        clipped_fraction=clipped_cnt / max(n_steps, 1),
        governor_infeasible_fraction=infeasible_cnt / max(n_steps, 1),
        keepout_radius=keep_radius if gov is not None else 0.0,
        started_inside=started_inside,
        ref_keepout_margin_min=ref_margin_min if gov is not None else float("nan"),
        lam_needed=math.hypot(veh.g, a_ref_max) / veh.g,
        min_human_distance=d_min,
        rotor_clearance_min=clear_min,
        soc_end=max(0.0, 1.0 - E_mission / veh.E_max) if veh.E_max > 0 else 0.0,
        endurance_est=veh.E_max / max(P_mean, 1e-9),
        overhead=P_mean / max(P_hover_ref, 1e-9),
        config=dict(mission=mission_kind, t_window=t_window, dt=dt, settle=settle,
                    substeps=max(int(substeps), 1),
                    seed=seed, sample_hz=sample_hz, governor=gov_cfg,
                    integrator="RK4, fixed step, zero-order-hold control",
                    power_model="hover-calibrated k_P Omega^3, no inflow correction"),
    )
    if res.saturation_fraction > 0.02:
        res.warnings.append(
            "motors saturate %.0f percent of the time: this mission peaks at %.1f m/s^2, "
            "which needs a thrust margin of at least %.2f before any control authority "
            "is left over, against the %.2f set"
            % (100.0 * res.saturation_fraction, res.ref_accel_max,
               res.lam_needed, float(p["lam"])))
    if res.screen_tilt_max_deg > float(p["theta_readable"]):
        res.violations.append("screen tilts %.1f deg, beyond the %.0f deg readability limit"
                              % (res.screen_tilt_max_deg, float(p["theta_readable"])))
    if res.rotor_clearance_min < d_safe:
        res.violations.append("nearest rotor disk came within %.3f m of the eye, inside "
                              "the %.2f m safety distance"
                              % (res.rotor_clearance_min, d_safe))
    elif res.min_human_distance < d_safe:
        res.warnings.append("the conservative envelope sphere came within %.3f m of the "
                            "eye; the rotor disks themselves stayed %.3f m away"
                            % (res.min_human_distance, res.rotor_clearance_min))
    if res.started_inside > 0.0:
        res.warnings.append("the ideal display position lies %.2f m inside the keep-out "
                            "radius of %.2f m about the head: the mission was started on "
                            "the sphere, and the governor holds the reference there"
                            % (res.started_inside, res.keepout_radius))
    if res.governor_infeasible_fraction > 0.0:
        res.violations.append("the reference governor could not satisfy its speed, "
                              "acceleration and keep-out limits together for %.1f percent "
                              "of the window" % (100.0 * res.governor_infeasible_fraction))
    # A tolerance of 0.01 percentage points: a battery sized by the hover
    # formula and flown standing still lands on the reserve to within the
    # windowed power estimate, and that is agreement, not a violation.
    if res.soc_end < float(p["soc_reserve"]) - 1e-4:
        res.violations.append("mission ends at %.1f percent state of charge, below the "
                              "%.0f percent reserve" % (100 * res.soc_end,
                                                        100 * float(p["soc_reserve"])))
    res.constraints_ok = not res.violations
    res.warnings = res.violations + res.warnings
    return res
