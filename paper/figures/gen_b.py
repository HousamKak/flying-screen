"""
Figure data for the fold (sec06), scaling (sec07) and co-design (sec13)
sections. Writes results/fig/b_*.dat.

The worked example of sec05 (alpha = 0.6366, beta = 0.2271) is used for the
purely analytic pictures of the fold; the engine supplies everything that
depends on a real design.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np

from _common import (GOV, PAPER, SIM, base_params, default_out,  # noqa: F401
                     write_table)

A_EX, B_EX = 0.6366, 0.2271          # worked example of sec05
M0_EX = 0.45                          # a feasible load for the pictures


def _roots_example(M0: float):
    from scipy.optimize import brentq
    F = lambda m: A_EX * m - B_EX * m ** 1.5
    ms = 4 * A_EX ** 2 / (9 * B_EX ** 2)
    lo = brentq(lambda m: F(m) - M0, 1e-12, ms)
    hi = brentq(lambda m: F(m) - M0, ms, 100.0)
    return lo, hi


def cobweb(out: str) -> None:
    """Staircases of m_{k+1} = Phi(m_k) for the worked example."""
    Phi = lambda m: M0_EX + (1 - A_EX) * m + B_EX * m ** 1.5

    def stairs(m0: float, n: int, cap: float, from_axis: bool = True):
        pts = [(m0, 0.0 if from_axis else m0)]
        m = m0
        for _ in range(n):
            y = Phi(m)
            pts.append((m, min(y, cap)))
            if y > cap:
                break
            pts.append((y, y))
            m = y
        return pts

    lo, hi = _roots_example(M0_EX)
    write_table(out, "b_cobweb_in", ["x", "y"], stairs(0.25, 14, 9.0))
    write_table(out, "b_cobweb_mid", ["x", "y"], stairs(hi - 0.35, 14, 9.0, False))
    write_table(out, "b_cobweb_out", ["x", "y"], stairs(hi + 0.35, 8, 9.0, False))
    write_table(out, "b_cobweb_roots", ["m", "F"], [(lo, lo), (hi, hi)])


def slowdown(out: str) -> None:
    """Rate 1 - F'(m_-) and iteration count against M0 / M0max."""
    Mm = 4 * A_EX ** 3 / (27 * B_EX ** 2)
    rows = []
    for r in np.concatenate([np.linspace(0.05, 0.9, 35), 1 - np.logspace(-1, -5, 25)]):
        M0 = r * Mm
        lo, _ = _roots_example(M0)
        rate = 1 - (A_EX - 1.5 * B_EX * math.sqrt(lo))
        m, k = 0.0, 0
        while abs(m - lo) > 1e-6 * lo and k < 10 ** 6:
            m = M0 + (1 - A_EX) * m + B_EX * m ** 1.5
            k += 1
        rows.append((r, 1 - r, rate, k))
    write_table(out, "b_slowdown", ["ratio", "gap", "rate", "iters"], rows)


def sensitivities(out: str) -> None:
    from flyingscreen import sizing
    from flyingscreen.params import DEFAULTS
    s = sizing.sensitivities(dict(DEFAULTS))
    order = ["t_f", "e_b_wh_kg", "FM", "eta", "D_rotor", "N_rotors", "rho",
             "f_s", "lam", "S_m", "g"]
    write_table(out, "b_sens", ["i", "S"],
                [(i, s[k]) for i, k in enumerate(order) if k in s])


def wall_tf(out: str) -> None:
    """Critical load and required load against endurance, lumped model."""
    from flyingscreen import sizing
    p = base_params()
    rows = []
    for t in np.geomspace(300.0, 8 * 3600.0, 70):
        c = sizing.coefficients(dict(p, t_f=float(t)), area_p=0.0)
        Mm = 4 * c.alpha ** 3 / (27 * c.beta ** 2)
        rows.append((t / 60.0, Mm, c.M0))
    write_table(out, "b_walltf", ["tmin", "Mmax", "M0"], rows)


def theta(out: str) -> None:
    """Theta_p(m) / t_f against m / m0 at the defaults, fold-mass anchor."""
    from scipy.optimize import minimize_scalar
    from flyingscreen import sizing
    from flyingscreen.params import DEFAULTS
    p = dict(DEFAULTS)
    t_f = float(p["t_f"])
    anchor = sizing.coefficients(p, area_p=0.0).anchor_mass
    ps = [0.0, 0.5, 0.75, 1.0, 1.25]
    cs = [sizing.coefficients(p, area_p=pp, anchor_mass=anchor) for pp in ps]

    def th(c, m):
        v = (c.alpha * m - c.m_fix) * c.e_b / (c.c_P * m ** c.q + c.P_aux)
        return v / t_f if v > 0 else float("nan")

    rows = []
    for x in np.geomspace(0.3, 300.0, 400):
        m = x * anchor
        rows.append([x] + [th(c, m) for c in cs])
    write_table(out, "b_theta", ["x"] + ["p%d" % i for i in range(len(ps))], rows)
    peaks = []
    for pp, c in zip(ps, cs):
        if pp < 1.0:
            opt = minimize_scalar(lambda lm: -th(c, math.exp(lm)) if th(c, math.exp(lm)) == th(c, math.exp(lm)) else 0.0,
                                  bounds=(math.log(c.m_fix / c.alpha * 1.0001), math.log(1e6)),
                                  method="bounded", options=dict(xatol=1e-10))
            peaks.append((pp, math.exp(opt.x) / anchor, -opt.fun))
    write_table(out, "b_theta_peaks", ["p", "x", "t"], peaks)
    c1 = cs[ps.index(1.0)]
    write_table(out, "b_theta_lim", ["x", "t"],
                [(0.3, c1.alpha * c1.e_b / c1.c_P / t_f), (300.0, c1.alpha * c1.e_b / c1.c_P / t_f)])


def pwall(out: str) -> None:
    """Endurance wall and wall mass against the disk-area exponent."""
    from scipy.optimize import minimize_scalar
    from flyingscreen import sizing, sweep
    from flyingscreen.params import DEFAULTS
    p = dict(DEFAULTS)
    t_f = float(p["t_f"])
    anchor = sizing.coefficients(p, area_p=0.0).anchor_mass
    rows = []
    for pp in list(np.arange(-0.25, 0.975, 0.025)) + [0.975, 0.985, 0.99]:
        pp = float(pp)
        # The bisection of sweep.endurance_wall evaluates the fold mass
        # (alpha / q beta)^(1/(q-1)), which overflows as q -> 1. The direct
        # maximisation of Theta_p is the same supremum and does not.
        try:
            t = sweep.endurance_wall(p, area_p=pp, anchor=anchor)
        except OverflowError:
            t = None
        c = sizing.coefficients(p, area_p=pp, anchor_mass=anchor)

        def neg(lm):
            m = math.exp(lm)
            return -(c.alpha * m - c.m_fix) * c.e_b / (c.c_P * m ** c.q + c.P_aux)
        opt = minimize_scalar(neg, bounds=(math.log(c.m_fix / c.alpha), math.log(1e12)),
                              method="bounded", options=dict(xatol=1e-12))
        rows.append((pp, (t or float("nan")) / t_f, -opt.fun / t_f,
                     math.exp(opt.x) / anchor))
    write_table(out, "b_pwall", ["p", "t", "tdirect", "x"], rows)
    c1 = sizing.coefficients(p, area_p=1.0, anchor_mass=anchor)
    write_table(out, "b_pwall_lim", ["p", "t"], [(1.0, c1.alpha * c1.e_b / c1.c_P / t_f)])


def fixed_point(out: str) -> None:
    """H(mu) sampled with the simulator, and the engine's secant iterates."""
    from flyingscreen import closure
    from flyingscreen.params import derived
    from flyingscreen.simulate import simulate_flight
    p = base_params()
    e_b = derived(p).e_b
    for mission, tag in (("turnaround", "turn"), ("standing", "stand")):
        rows = []
        for mu in np.linspace(0.17, 0.35, 5):
            mu = float(mu)
            m = closure.gross_mass_for_battery(p, mu)
            s = simulate_flight(p, m, m_bat=mu, mission_kind=mission,
                                governor=GOV, **SIM)
            rows.append((mu, closure._requirement(p, s, e_b)["m_req"]))
        write_table(out, "b_H_" + tag, ["mu", "H"], rows)
    with open(os.path.join(PAPER, "results", "results.json"), encoding="utf-8") as fh:
        R = json.load(fh)
    for mission, tag in (("turnaround", "turn"), ("standing", "stand")):
        tr = R["coupled"]["rows"][mission]["trace"]
        write_table(out, "b_trace_" + tag, ["k", "mu", "H", "verify"],
                    [(t["iteration"], t["m_bat"], t["m_bat_required"],
                      1 if t["kind"] == "verify" else 0) for t in tr])
        # Staircase for the plot: (mu_k, mu_k) -> (mu_k, H_k) -> (mu_{k+1}, ...)
        st = []
        for t in tr:
            if t["kind"] != "update":
                continue
            st.append((t["m_bat"], t["m_bat"]))
            st.append((t["m_bat"], t["m_bat_required"]))
        write_table(out, "b_trace_path_" + tag, ["x", "y"], st)


def inner(out: str) -> None:
    """The inner map of the component models at the design battery."""
    from flyingscreen import components as C
    from flyingscreen.params import derived
    p = base_params()
    r = C.close_first_principles(p)
    mu = r["m_bat"]
    m_fix = derived(p).m_fix
    S = lambda m: (lambda d: d["m_frame"] + d["m_prop"])(C._dry_mass(p, m))
    rows = []
    for m in np.linspace(0.4, 3.2, 57):
        h = 1e-4 * m
        rows.append((m, m_fix + S(m) + mu, S(m), (S(m + h) - S(m - h)) / (2 * h)))
    write_table(out, "b_inner", ["m", "phi", "S", "slope"], rows)
    write_table(out, "b_inner_fp", ["m", "phi"], [(r["m"], r["m"])])


def kappa(out: str) -> None:
    """Closure curves with the mission overhead applied to all power."""
    from flyingscreen import sizing
    p = base_params()
    c = sizing.coefficients(p, area_p=0.0)
    aux = c.M0 - c.m_fix
    ks = [1.0, 1.1, 1.25]
    top = (c.alpha / c.beta) ** 2
    rows = []
    for m in np.linspace(0.0, top, 240):
        rows.append([m] + [c.alpha * m - k * c.beta * m ** 1.5 for k in ks]
                    + [c.m_fix + k * aux for k in ks])
    write_table(out, "b_kappa", ["m", "F0", "F1", "F2", "M0", "M1", "M2"], rows)
    pts = []
    for k in ks:
        ms = 4 * c.alpha ** 2 / (9 * (k * c.beta) ** 2)
        pts.append((k, ms, c.alpha * ms / 3.0, c.m_fix + k * aux))
    write_table(out, "b_kappa_pts", ["k", "mstar", "Mmax", "M0"], pts)


def generate(out_dir: str) -> None:
    for f in (cobweb, slowdown, sensitivities, wall_tf, theta, pwall, inner,
              kappa, fixed_point):
        f(out_dir)


if __name__ == "__main__":
    generate(default_out())
