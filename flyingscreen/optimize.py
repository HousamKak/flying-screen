"""
Layer 7: choose the machine.

    min over p of   w_m m + w_P P + w_N SPL + w_s span + w_e e_track
    subject to      the mass closure existing,
                    T_max >= lambda m g            (by construction),
                    blade loading below stall,
                    SOC at the end above the reserve,
                    downwash and noise limits,
                    span below the room limit.

Constraints are handled by an exterior penalty so that the search still has
a gradient to follow in the infeasible region, which matters here because
large parts of the design space genuinely do not close.

The objective uses the first-principles closure, which is milliseconds per
evaluation.  Tracking error, which needs the simulator, is optionally
folded in at the end by re-scoring the best few candidates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from . import components as comp
from .params import clamp_params, describe
from .simulate import simulate_flight

# Variables the optimiser is allowed to move, with their bounds taken from
# the schema unless overridden by the caller.
DESIGN_VARS = ["D_rotor", "N_rotors", "ct_sigma_design", "lam", "arm_radius",
               "blade_AR", "n_blades", "tip_speed", "e_b_wh_kg", "standoff"]

DEFAULT_VARS = ["D_rotor", "N_rotors", "ct_sigma_design", "lam"]

DEFAULT_WEIGHTS = dict(w_m=1.0, w_P=0.004, w_N=0.05, w_span=0.6, w_track=2.0)

DEFAULT_LIMITS = dict(span_max=1.6, spl_max=72.0, downwash_max=11.0,
                      m_max=12.0, lam_margin=1.35)


@dataclass
class Candidate:
    x: Dict[str, float]
    ok: bool
    J: float
    penalty: float
    m: float = 0.0
    P_total: float = 0.0
    spl: float = 0.0
    span: float = 0.0
    downwash: float = 0.0
    m_bat: float = 0.0
    endurance: float = 0.0
    lam_required: float = 0.0
    reason: str = ""
    violations: List[str] = field(default_factory=list)
    detail: Dict[str, Any] = field(default_factory=dict)


def evaluate(p: Dict[str, float], weights: Dict[str, float],
             limits: Dict[str, float]) -> Candidate:
    """Score one design with the first-principles closure."""
    q = clamp_params(p)
    r = comp.close_first_principles(q)
    if not r["feasible"]:
        return Candidate(x={}, ok=False, J=1e6, penalty=1e6,
                         reason=r.get("reason", "does not close"))
    b = r["breakdown"]
    m, P = b["m"], b["P_total"]
    spl, span, dw = b["spl_1m"], b["span"], b["downwash"]

    viol: List[str] = []
    pen = 0.0

    def over(value: float, limit: float, scale: float, label: str) -> None:
        nonlocal pen
        if value > limit * (1.0 + 2e-3):
            pen += ((value - limit) / scale) ** 2
            viol.append("%s %.3g exceeds %.3g" % (label, value, limit))

    # The thrust margin is not free to shrink: it has to cover the
    # acceleration the mission demands, the screen drag at following speed
    # and the wind, plus the differential thrust the attitude loop spends.
    v_rel = float(q["v_follow"]) + float(q["wind_mean"]) + float(q["wind_gust"])
    D_screen = 0.5 * float(q["rho"]) * float(q["Cd_screen_n"]) * \
        float(q["screen_w"]) * float(q["screen_h"]) * v_rel * v_rel
    F_h = m * float(q["a_follow"]) + D_screen
    lam_req = math.hypot(m * float(q["g"]), F_h) / (m * float(q["g"])) * \
        float(limits.get("lam_margin", 1.35))
    if float(q["lam"]) < lam_req * (1.0 - 2e-3):
        pen += ((lam_req - float(q["lam"])) / 0.1) ** 2
        viol.append("thrust margin %.3g below the %.3g this mission needs"
                    % (float(q["lam"]), lam_req))

    over(span, float(limits["span_max"]), 0.2, "span")
    over(spl, float(limits["spl_max"]), 3.0, "SPL at 1 m")
    over(dw, float(limits["downwash_max"]), 2.0, "downwash")
    over(m, float(limits["m_max"]), 1.0, "gross mass")
    hover = b["rotor"]["hover"]
    over(hover["Ct_sigma"], float(q["ct_sigma_max"]), 0.02, "blade loading")
    over(hover["sigma"], float(q["sigma_max"]), 0.03, "solidity")
    if hover["reynolds"] < 40000:
        pen += ((40000 - hover["reynolds"]) / 8000.0) ** 2
        viol.append("blade Reynolds %.0f below 40000" % hover["reynolds"])
    mx = b["rotor"]["max"]
    over(mx["Ct_sigma"], float(q["ct_sigma_max"]), 0.02, "blade loading at T_max")
    if b["m_bat"] / max(m, 1e-9) > 0.7:
        pen += ((b["m_bat"] / m - 0.7) / 0.05) ** 2
        viol.append("battery fraction %.2f" % (b["m_bat"] / m))

    J = (float(weights["w_m"]) * m
         + float(weights["w_P"]) * P
         + float(weights["w_N"]) * spl
         + float(weights["w_span"]) * span)
    return Candidate(x={}, ok=len(viol) == 0, J=J + 100.0 * pen, penalty=pen,
                     m=m, P_total=P, spl=spl, span=span, downwash=dw,
                     m_bat=b["m_bat"], endurance=b["endurance"],
                     violations=viol, detail=b, lam_required=lam_req)


def optimise(p: Dict[str, float], variables: Optional[List[str]] = None,
             weights: Optional[Dict[str, float]] = None,
             limits: Optional[Dict[str, float]] = None,
             maxiter: int = 45, popsize: int = 14, seed: int = 7,
             refine_with_sim: bool = False,
             mission_kind: str = "pacing",
             t_window: float = 8.0) -> Dict[str, Any]:
    """Differential evolution over the chosen design variables."""
    from scipy.optimize import differential_evolution

    variables = variables or list(DEFAULT_VARS)
    weights = {**DEFAULT_WEIGHTS, **(weights or {})}
    limits = {**DEFAULT_LIMITS, **(limits or {})}

    specs = [describe(k) for k in variables]
    bounds = [(float(s["lo"]), float(s["hi"])) for s in specs]

    history: List[float] = []

    def unpack(xv) -> Dict[str, float]:
        q = dict(p)
        for k, s, v in zip(variables, specs, xv):
            q[k] = float(v)
        return clamp_params(q)      # handles integer and even-only fields

    def obj(xv) -> float:
        c = evaluate(unpack(xv), weights, limits)
        return float(c.J)

    def cb(xk, convergence=None):
        history.append(float(obj(xk)))
        return False

    res = differential_evolution(
        obj, bounds, maxiter=maxiter, popsize=popsize, seed=seed,
        tol=1e-6, mutation=(0.4, 1.0), recombination=0.85, polish=True,
        init="sobol", callback=cb)

    best_p = unpack(res.x)
    best = evaluate(best_p, weights, limits)
    best.x = {k: best_p[k] for k in variables}

    baseline = evaluate(clamp_params(p), weights, limits)

    out: Dict[str, Any] = dict(
        success=bool(res.success), iterations=int(res.nit),
        evaluations=int(res.nfev), history=history,
        variables=variables, bounds=bounds, weights=weights, limits=limits,
        best=dict(x=best.x, J=best.J, ok=best.ok, m=best.m, P_total=best.P_total,
                  spl=best.spl, span=best.span, downwash=best.downwash,
                  m_bat=best.m_bat, endurance=best.endurance,
                  lam_required=best.lam_required,
                  violations=best.violations, detail=best.detail),
        baseline=dict(J=baseline.J, ok=baseline.ok, m=baseline.m,
                      P_total=baseline.P_total, spl=baseline.spl,
                      span=baseline.span, violations=baseline.violations),
        params=best_p,
    )

    if refine_with_sim:
        sim = simulate_flight(best_p, best.m, m_bat=best.m_bat,
                              mission_kind=mission_kind, t_window=t_window)
        out["sim"] = dict(ok=sim.ok, reason=sim.reason, P_mean=sim.P_mean,
                          overhead=sim.overhead, e_track_rms=sim.e_track_rms,
                          e_track_max=sim.e_track_max,
                          screen_tilt_max_deg=sim.screen_tilt_max_deg,
                          saturation_fraction=sim.saturation_fraction,
                          warnings=sim.warnings)
        if sim.ok:
            out["best"]["J_with_tracking"] = best.J + float(weights["w_track"]) * sim.e_track_rms
    return out


def pareto(p: Dict[str, float], var: str = "D_rotor", n: int = 24,
           lo: Optional[float] = None, hi: Optional[float] = None) -> Dict[str, Any]:
    """
    One-dimensional trade study: sweep a single variable and report the
    whole vector of consequences, which is usually more informative than a
    scalar optimum.
    """
    s = describe(var)
    lo = float(s["lo"]) if lo is None else lo
    hi = float(s["hi"]) if hi is None else hi
    pts = []
    for i in range(n):
        v = lo + (hi - lo) * i / (n - 1)
        q = dict(p)
        q[var] = int(round(v)) if s.get("integer") else v
        q = clamp_params(q)
        r = comp.close_first_principles(q)
        if not r["feasible"]:
            pts.append(dict(value=q[var], feasible=False))
            continue
        b = r["breakdown"]
        pts.append(dict(value=q[var], feasible=True, m=b["m"], m_bat=b["m_bat"],
                        P_total=b["P_total"], spl=b["spl_1m"], span=b["span"],
                        downwash=b["downwash"], FM=b["FM_implied"],
                        disk_loading=b["disk_loading"],
                        Ct_sigma=b["rotor"]["hover"]["Ct_sigma"],
                        rpm=b["rotor"]["hover"]["rpm"]))
    return dict(variable=var, label=s["label"], unit=s["unit"], points=pts)
