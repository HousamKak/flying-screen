"""
Feasibility maps.

Sweep any two parameters over a grid, solve the closure at every point and
classify the result.  The output is the picture the whole project is really
about: where in assumption space a flying screen exists at all, and how the
boundary moves when technology or geometry changes.

The analytic boundary for the fixed-area model is available in closed form,
so the map can be drawn with the exact curve overlaid on the numerical
classification, which is the check that the two agree.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from . import components as comp
from . import sizing
from .params import clamp_params, describe

METRICS = {
    "m": "Gross mass [kg]",
    "m_bat": "Battery mass [kg]",
    "P_total": "Hover power [W]",
    "disk_loading": "Disk loading [N/m^2]",
    "utilisation": "M0 / M0_max [-]",
    "D_rotor": "Rotor diameter [m]",
    "endurance_check": "Endurance [s]",
    "battery_fraction": "Battery / gross mass [-]",
    "power_loading": "Power loading [N/W]",
}


def _axis(spec: Dict[str, Any]) -> List[float]:
    lo, hi = float(spec["lo"]), float(spec["hi"])
    n = int(spec.get("n", 48))
    if spec.get("log"):
        lo = max(lo, 1e-12)
        return [lo * (hi / lo) ** (i / (n - 1)) for i in range(n)]
    return [lo + (hi - lo) * i / (n - 1) for i in range(n)]


def sweep2d(p: Dict[str, float], x: Dict[str, Any], y: Dict[str, Any],
            metric: str = "m", model: str = "fixed_area",
            first_principles: bool = False) -> Dict[str, Any]:
    """
    x and y are {key, lo, hi, n, log}.  Returns a grid of metric values with
    NaN wherever the design does not close, plus the feasibility mask.
    """
    xs, ys = _axis(x), _axis(y)
    Z: List[List[Optional[float]]] = []
    feas: List[List[int]] = []
    reasons: Dict[str, int] = {}

    for yv in ys:
        rowz: List[Optional[float]] = []
        rowf: List[int] = []
        for xv in xs:
            q = dict(p)
            q[x["key"]] = xv
            q[y["key"]] = yv
            q = clamp_params(q)
            try:
                if first_principles:
                    r = comp.close_first_principles(q)
                    if not r["feasible"]:
                        rowz.append(None); rowf.append(0)
                        reasons[r.get("reason", "?")] = reasons.get(r.get("reason", "?"), 0) + 1
                        continue
                    b = r["breakdown"]
                    vals = dict(m=b["m"], m_bat=b["m_bat"], P_total=b["P_total"],
                                disk_loading=b["disk_loading"],
                                D_rotor=float(q["D_rotor"]),
                                endurance_check=b["endurance"],
                                battery_fraction=b["m_bat"] / max(b["m"], 1e-9),
                                power_loading=b["m"] * float(q["g"]) / max(b["P_total"], 1e-9),
                                utilisation=float("nan"))
                    rowz.append(vals.get(metric)); rowf.append(1)
                else:
                    dp = sizing.solve(q, model)
                    if not dp.feasible:
                        rowz.append(None); rowf.append(0)
                        reasons[dp.reason] = reasons.get(dp.reason, 0) + 1
                        continue
                    util = dp.closure.get("utilisation")
                    vals = dict(m=dp.m, m_bat=dp.m_bat, P_total=dp.P_total,
                                disk_loading=dp.disk_loading, D_rotor=dp.D_rotor,
                                endurance_check=dp.endurance_check,
                                battery_fraction=dp.m_bat / max(dp.m, 1e-9),
                                power_loading=dp.power_loading,
                                utilisation=util if util is not None else float("nan"))
                    rowz.append(vals.get(metric)); rowf.append(1)
            except Exception as exc:      # keep the map, mark the hole
                rowz.append(None); rowf.append(0)
                reasons[str(exc)] = reasons.get(str(exc), 0) + 1
        Z.append(rowz)
        feas.append(rowf)

    return dict(x=xs, y=ys, z=Z, feasible=feas, metric=metric,
                metric_label=METRICS.get(metric, metric),
                x_key=x["key"], y_key=y["key"],
                x_label=describe(x["key"])["label"],
                y_label=describe(y["key"])["label"],
                x_unit=describe(x["key"])["unit"],
                y_unit=describe(y["key"])["unit"],
                model=model, reasons=reasons)


def payload_boundary(p: Dict[str, float], t_lo: float = 120.0,
                     t_hi: float = 28800.0, n: int = 120) -> Dict[str, Any]:
    """
    The closed-form wall m_screen_max(t_f), which the numerical map must
    reproduce.  This is the 1 / t_f^2 law made explicit.
    """
    ts, walls, screens = [], [], []
    for i in range(n):
        t = t_lo * (t_hi / t_lo) ** (i / (n - 1))
        q = dict(p)
        q["t_f"] = t
        w = sizing.payload_wall(q)
        ts.append(t)
        walls.append(w.get("M0_max"))
        screens.append(w.get("m_screen_max"))
    return dict(t=ts, M0_max=walls, m_screen_max=screens,
                law="M0_max is proportional to 1 / t_f^2 at fixed disk area")


def endurance_wall(p: Dict[str, float], area_p: float, anchor: float,
                   t_lo: float = 60.0, t_hi: float = 200000.0,
                   iters: int = 44) -> Optional[float]:
    """
    Longest endurance at which the closure still has a solution, for a given
    disk-area exponent.  Returns None when there is no wall at all, which is
    exactly what happens once p reaches 1.
    """
    def ok(t: float) -> bool:
        q = dict(p)
        q["t_f"] = t
        return bool(sizing.solve_power_law(q, area_p=area_p,
                                           anchor_mass=anchor).feasible)

    if not ok(t_lo):
        return t_lo
    if ok(t_hi):
        return None
    lo, hi = t_lo, t_hi
    for _ in range(iters):
        mid = math.sqrt(lo * hi)
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def scaling_sweep(p: Dict[str, float], p_lo: float = -0.25, p_hi: float = 1.75,
                  n: int = 25, endurance_factors: Optional[List[float]] = None
                  ) -> Dict[str, Any]:
    """
    Sweep the disk-area exponent.

    Every exponent is anchored at the same physical rotor, so at the current
    endurance they agree.  What separates them is what happens when more is
    asked: the mass at longer endurance, and whether an endurance wall
    exists at all.  That is the whole content of P ~ m^((3-p)/2).
    """
    factors = endurance_factors or [1.0, 2.0, 4.0, 8.0]
    anchor = sizing.coefficients(p, area_p=0.0).anchor_mass
    t0 = float(p["t_f"])
    out = []
    for i in range(n):
        pp = p_lo + (p_hi - p_lo) * i / (n - 1)
        c = sizing.coefficients(p, area_p=pp, anchor_mass=anchor)
        dp = sizing.solve_power_law(p, area_p=pp, anchor_mass=anchor)
        masses = []
        for f in factors:
            q = dict(p)
            q["t_f"] = t0 * f
            r = sizing.solve_power_law(q, area_p=pp, anchor_mass=anchor)
            masses.append(dict(factor=f, t_f=t0 * f,
                               m=r.m if r.feasible else None,
                               D_rotor=r.D_rotor if r.feasible else None,
                               feasible=bool(r.feasible)))
        out.append(dict(
            p=pp, q=c.q, feasible=bool(dp.feasible),
            m=dp.m if dp.feasible else None,
            m_bat=dp.m_bat if dp.feasible else None,
            D_rotor=dp.D_rotor if dp.feasible else None,
            P_total=dp.P_total if dp.feasible else None,
            disk_loading=dp.disk_loading if dp.feasible else None,
            has_fold=bool(c.q > 1.0),
            M0_max=(dp.closure or {}).get("M0_max"),
            t_wall=endurance_wall(p, pp, anchor),
            endurance_scan=masses,
        ))
    return dict(points=out, anchor=anchor, t_f=t0, factors=factors,
                note=("P is proportional to m^((3-p)/2). At p = 1 the exponent "
                      "becomes 1, the fold disappears and the endurance wall goes "
                      "to infinity; what replaces it is rotor diameter."))
