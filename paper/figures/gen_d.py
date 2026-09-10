"""
Figure data for the control, engine, numerics and verification sections
(figures sec12_*, sec14_*, sec15_*, sec16_*).

Writes results/fig/d_*.dat tables and results/fig/d_macros.tex.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from _common import (GOV, SIM, base_params, default_out,  # noqa: E402
                     write_macros, write_table)


def f(x: float, n: int = 2) -> str:
    return ("%%.%df" % n) % x


def _attainable(veh, n_dir: int = 240):
    """
    The set of (tau_x, tau_y) the rotors can produce with thrust mg and zero
    yaw moment, by its support function: one linear program per direction.
    """
    from scipy.optimize import linprog
    A = np.asarray(veh.alloc) * veh.omega_hover ** 2
    lim = (veh.omega_max / veh.omega_hover) ** 2
    pts = []
    for th in np.linspace(0.0, 2.0 * math.pi, n_dir, endpoint=False):
        c = -(math.cos(th) * A[1] + math.sin(th) * A[2])
        res = linprog(c, A_eq=A[[0, 3]], b_eq=[veh.m * veh.g, 0.0],
                      bounds=[(0.0, lim)] * veh.N, method="highs")
        if res.success:
            pts.append(A[1:3] @ res.x)
    pts = np.array(pts)
    # Keep the distinct vertices, ordered by angle about the centroid.
    uniq = []
    for q in pts:
        if not any(np.linalg.norm(q - u) < 1e-6 for u in uniq):
            uniq.append(q)
    uniq = np.array(uniq)
    cen = uniq.mean(axis=0)
    ang = np.arctan2(uniq[:, 1] - cen[1], uniq[:, 0] - cen[0])
    uniq = uniq[np.argsort(ang)]
    return np.vstack([uniq, uniq[:1]])


def _rk4_boundary(center: float = -1.4, n: int = 720):
    """Boundary of {z : |R(z)| <= 1} for classical RK4, traced along rays."""
    R = lambda z: 1 + z + z * z / 2 + z ** 3 / 6 + z ** 4 / 24
    out = []
    for phi in np.linspace(0.0, 2.0 * math.pi, n, endpoint=False):
        d = complex(math.cos(phi), math.sin(phi))
        lo, hi = 0.0, None
        r = 0.0
        while r < 5.0:
            r += 0.01
            if abs(R(center + r * d)) > 1.0:
                hi = r
                break
            lo = r
        if hi is None:
            continue
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if abs(R(center + mid * d)) > 1.0:
                hi = mid
            else:
                lo = mid
        z = center + lo * d
        out.append((z.real, z.imag))
    out.append(out[0])
    return out


def generate(out_dir: str) -> None:
    from scipy.optimize import brentq, minimize_scalar

    from flyingscreen import closure as CL
    from flyingscreen import components as C
    from flyingscreen import mission as M
    from flyingscreen import sizing
    from flyingscreen.control import control_authority
    from flyingscreen.dynamics import build_vehicle
    from flyingscreen.simulate import simulate_flight

    p = base_params()
    base = C.close_first_principles(p)
    m, mb = base["m"], base["m_bat"]
    veh = build_vehicle(p, m, mb)
    mac = {}

    # -- attainable torque sets: quadrotor design and its hexacopter twin ----
    quad = _attainable(veh)
    write_table(out_dir, "d_attain_quad", ["tx", "ty"], quad.tolist())
    aq = control_authority(veh)
    # The hexacopter of tab:hex: the same payload with its rotors
    # re-optimised under the same limits as the quadrotor (make_results.py,
    # section hex), so that this figure and the table describe one machine.
    import json
    import os
    from _common import PAPER
    from flyingscreen.params import clamp_params
    with open(os.path.join(PAPER, "results", "results.json"), encoding="utf-8") as fh:
        hx = json.load(fh)["hex"]["rows"]["hex"]
    q6 = clamp_params(dict(p, N_rotors=6.0, **hx["x"]))
    b6 = C.close_first_principles(q6)
    veh6 = build_vehicle(q6, b6["m"], b6["m_bat"])
    hexa = _attainable(veh6)
    write_table(out_dir, "d_attain_hex", ["tx", "ty"], hexa.tolist())
    ah = control_authority(veh6)
    mac.update(fdQuadRoll=f(aq["tau_roll"]), fdQuadPitch=f(aq["tau_pitch"]),
               fdQuadOld=f(aq["tau_roll"] / 4.0),
               fdHexRoll=f(ah["tau_roll"]), fdHexPitch=f(ah["tau_pitch"]),
               fdHexGross=f(b6["m"], 3), fdQuadGross=f(m, 3))

    # -- turnaround: top view and distances ---------------------------------
    s = simulate_flight(p, m, m_bat=mb, mission_kind="turnaround",
                        governor=GOV, **SIM)
    human = M.make_human(p, "turnaround")
    off = np.asarray(veh.screen_offset)
    zc = float(p["standoff_z"])
    rows = []
    for k, t in enumerate(s.t):
        hr = np.asarray(s.trajectory["r_h"][k])
        head = hr + np.array([0.0, 0.0, zc])
        raw = np.asarray(M.reference(p, human, t, screen_offset=off)["r"])
        gov = np.asarray(s.trajectory["r_d"][k])
        vv = np.asarray(s.trajectory["r"][k])
        rows.append([t, head[0], head[1], raw[0], raw[1], gov[0], gov[1],
                     vv[0], vv[1], np.linalg.norm(raw - head),
                     np.linalg.norm(gov - head), np.linalg.norm(vv - head),
                     np.linalg.norm(raw - vv), s.telemetry["rotor_clearance"][k]])
    write_table(out_dir, "d_turn",
                ["t", "hx", "hy", "rawx", "rawy", "govx", "govy", "vehx", "vehy",
                 "draw", "dgov", "dveh", "elag", "clear"], rows)
    # The reversal inside the plotted window: heading passes pi/2 where the
    # pacing velocity changes sign, w t = 3 pi / 2.
    hum = human
    t_turn = 1.5 * math.pi / hum.w
    arr = np.array(rows)
    dz = float(np.mean([np.asarray(s.trajectory["r"][k])[2] for k in range(len(s.t))])) - zc
    rho = s.keepout_radius
    mac.update(fdKeepRadius=f(rho, 3), fdKeepSlice=f(math.sqrt(max(rho ** 2 - dz ** 2, 0.0)), 3),
               fdEnvRadius=f(float(p["d_safe"]) + veh.reach, 3),
               fdTurnT=f(t_turn, 2), fdTurnHeadX=f(float(np.interp(t_turn, arr[:, 0], arr[:, 1])), 3),
               fdTurnHeadY=f(float(np.interp(t_turn, arr[:, 0], arr[:, 2])), 3),
               fdTurnPeriod=f(2 * math.pi / hum.w, 2), fdStandoff=f(float(p["standoff"]), 2))

    # -- root search near the fold ------------------------------------------
    near = dict(p, t_f=4280.05020145)
    F = lambda mm: mm - (C._dry_mass(near, mm)["dry"]
                         + C.required_battery(near, mm)["m_bat"])
    wide = [(mm, F(mm)) for mm in np.geomspace(0.5, 40.0, 420)]
    write_table(out_dir, "d_root_wide", ["m", "F"], wide)
    grid = []
    x = 1e-3
    while x <= 40.0:
        if x >= 0.5:
            grid.append((x, F(x)))
        x *= 1.35
    write_table(out_dir, "d_root_grid", ["m", "F"], grid)
    lo_s = max(g[0] for g in grid if g[0] < 4.56)
    hi_s = min(g[0] for g in grid if g[0] > 4.62)
    opt = minimize_scalar(lambda z: -F(math.exp(z)),
                          bounds=(math.log(lo_s), math.log(hi_s)), method="bounded",
                          options={"xatol": 1e-12})
    m_pk, F_pk = math.exp(opt.x), -opt.fun
    r1 = brentq(F, lo_s, m_pk, xtol=1e-12)
    r2 = brentq(F, m_pk, hi_s, xtol=1e-12)
    zoom = [(mm, F(mm)) for mm in np.linspace(4.52, 4.66, 420)]
    write_table(out_dir, "d_root_zoom", ["m", "F"], zoom)
    mac.update(fdRootLo=f(r1, 4), fdRootHi=f(r2, 4), fdPeakM=f(m_pk, 4),
               fdPeakF=f(1e3 * F_pk, 2), fdGridLo=f(lo_s, 3), fdGridHi=f(hi_s, 3),
               fdGridLoF=f(F(lo_s), 3), fdGridHiF=f(F(hi_s), 3),
               fdIslandPct=f(100 * (r2 - r1) / r1, 1))

    # -- battery loop: secant against plain iteration ------------------------
    cw = dict(t_window=8.0, dt=SIM["dt"], settle=SIM["settle"], seed=SIM["seed"])
    run = CL.close_with_simulation(p, mission_kind="pacing", governor=GOV,
                                   keep_telemetry=False, **cw)
    sec = [(k, abs(t["residual"])) for k, t in enumerate(run.trace) if t["kind"] == "update"]
    mu = run.trace[0]["m_bat"]
    d = CL.derived(p)
    plain = []
    for k in range(9):
        mm = CL.gross_mass_for_battery(p, mu)
        sim = simulate_flight(p, mm, m_bat=mu, mission_kind="pacing", governor=GOV, **cw)
        req = CL._requirement(p, sim, d.e_b)["m_req"]
        plain.append((k, abs(req - mu)))
        mu = req
    write_table(out_dir, "d_conv_secant", ["k", "gap"], sec)
    write_table(out_dir, "d_conv_plain", ["k", "gap"], plain)
    rate = plain[2][1] / plain[1][1] if plain[1][1] > 0 else float("nan")
    mac.update(fdPlainRate=f(rate, 3), fdConvWindow=f(cw["t_window"], 0))

    # -- RK4 stability region and the design's h lambda --------------------
    write_table(out_dir, "d_rk4", ["re", "im"], _rk4_boundary())
    h = SIM["dt"]
    tau_m = float(p["tau_motor"])
    KR, Kw = float(p["K_R"]), float(p["K_w"])
    Kp, Kd = float(p["Kp_pos"]), float(p["Kd_pos"])
    pts = [("motor", -h / tau_m, 0.0)]
    for name, a1, a0 in (("att", Kw, KR), ("pos", Kd, Kp), ("gov", 8.0, 16.0)):
        disc = complex(a1 * a1 - 4 * a0) ** 0.5
        for sgn in (1, -1):
            lam = (-a1 + sgn * disc) / 2
            pts.append((name, h * lam.real, h * lam.imag))
    write_table(out_dir, "d_rk4_pts", ["kind", "re", "im"],
                [(i, r, im) for i, (_, r, im) in enumerate(pts)])
    for name, key in (("motor", "Motor"), ("att", "Att"), ("pos", "Pos"), ("gov", "Gov")):
        sel = [q for q in pts if q[0] == name]
        mac["fdHl%sRe" % key] = f(sel[0][1], 4)
        mac["fdHl%sIm" % key] = f(abs(sel[0][2]), 4)

    # -- loop bandwidths ------------------------------------------------------
    arm = C.breakdown(p, m, m_bat=mb).arm
    mac.update(fdWpos=f(math.sqrt(Kp), 2), fdWgov=f(4.0, 2), fdWatt=f(math.sqrt(KR), 2),
               fdWmot=f(1.0 / tau_m, 1), fdWarm=f(2 * math.pi * arm["f_bending"], 0),
               fdWnyq=f(math.pi / h, 0), fdRatioPA=f(math.sqrt(KR / Kp), 1),
               fdRatioAM=f((1.0 / tau_m) / math.sqrt(KR), 1),
               fdKR=f(KR, 1), fdKp=f(Kp, 2), fdTauMms=f(1e3 * tau_m, 0),
               fdH=f(1e3 * h, 0))

    # -- motor calibration: the design's motor ------------------------------
    pr = C.propulsion(p, m)
    mac.update(fdMotorQ=f(pr.Q_max, 3), fdMotorM=f(pr.m_motor_each, 4))

    write_macros(out_dir, "d_macros", mac)


if __name__ == "__main__":
    generate(default_out())
