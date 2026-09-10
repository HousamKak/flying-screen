"""
Stage 6: close the battery loop around the real simulation.

    guess m_bat -> gross mass -> simulate the mission -> energy actually used
                -> required m_bat -> repeat

If the sequence settles, the mass and energy balance closes. That is not the
same as the design being acceptable: the final simulation is checked
separately for reserve, peak power, rotor clearance and screen attitude, and
the result reports both.

The expensive part is the simulation, so the loop is structured to need one
simulation per outer iteration: the inner mass closure (frame and propulsion
mass as functions of gross mass) is algebraic and converges in a few cheap
passes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from . import components as comp
from . import sizing
from .params import derived
from .simulate import simulate_flight


def gross_mass_for_battery(p: Dict[str, float], m_bat: float,
                           iters: int = 60) -> float:
    """
    Solve m = m_fix + m_frame(m) + m_prop(m) + m_bat for a fixed battery.

    Both submodels are close to linear in m with slope well below one, so
    plain iteration converges geometrically and needs no derivatives. The
    slope condition is what makes the root unique (see the inner-map
    proposition in the paper); it is checked numerically by the test suite
    for the shipped submodels, not assumed for arbitrary ones.
    """
    d = derived(p)
    N = float(p["N_rotors"])
    m = d.m_fix + m_bat
    for _ in range(iters):
        pr = comp.propulsion(p, m)
        arm = comp.arm_structure(
            p, m, m_tip=pr.m_motor_each + (pr.m_esc + pr.m_props) / N)
        m_new = d.m_fix + arm.m_frame + pr.m_prop_total + m_bat
        if abs(m_new - m) < 1e-10 * max(1.0, m):
            return m_new
        m = m_new
    return m


@dataclass
class ClosureRun:
    converged: bool
    diverged: bool
    status: str
    reason: str = ""
    trace: List[Dict[str, float]] = field(default_factory=list)
    m: float = 0.0
    m_bat: float = 0.0
    P_mean: float = 0.0
    E_mission: float = 0.0
    overhead: float = 1.0
    iterations: int = 0            # outer updates of the battery mass
    sim_calls: int = 0             # every simulation, verification included
    energy_margin: float = 0.0     # e_b (1 - reserve) m_bat - E_mission   [J]
    power_margin: float = 0.0      # p_b m_bat - P_peak                    [W]
    reserve_end: float = 0.0       # state of charge left after the mission
    battery_ok: bool = False       # both margins non-negative
    constraints_ok: bool = False   # final flight met its path constraints
    feasible: bool = False         # converged, battery_ok and constraints_ok
    sim: Dict[str, Any] = field(default_factory=dict)
    design: Dict[str, Any] = field(default_factory=dict)
    analytic: Dict[str, Any] = field(default_factory=dict)
    config: Dict[str, Any] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return dict(self.__dict__)


def _requirement(p: Dict[str, float], sim, e_b: float) -> Dict[str, float]:
    """Battery the flown mission requires: the larger of energy and power."""
    reserve = min(max(float(p["soc_reserve"]), 0.0), 0.9)
    E_mission = sim.P_mean * float(p["t_f"])
    m_energy = E_mission / (e_b * (1.0 - reserve))
    m_power = sim.P_peak / float(p["p_b_w_kg"])
    return dict(E_mission=E_mission, m_energy=m_energy, m_power=m_power,
                m_req=max(m_energy, m_power), reserve=reserve,
                limit="power" if m_power > m_energy else "energy")


def close_with_simulation(p: Dict[str, float], mission_kind: str = "pacing",
                          t_window: float = 12.0, dt: float = 0.004,
                          m_bat0: Optional[float] = None, max_iter: int = 8,
                          tol: float = 2e-3, relax: float = 1.0,
                          cap_ratio: float = 60.0,
                          keep_telemetry: bool = True,
                          governor: Optional[Dict[str, float]] = None,
                          settle: float = 3.0, seed: int = 12345) -> ClosureRun:
    """
    The residual pair from the derivation,

        R1 = m - (m_fix + m_str + m_prop + m_bat)
        R2 = m_bat - max(E_mission / (e_b (1 - reserve)), P_peak / p_b)

    driven to zero. R1 is solved exactly at every step by
    gross_mass_for_battery, so the outer loop only has to drive R2. Once R2
    is within tolerance the battery is rounded up to the requirement and the
    design is flown once more; the run is reported converged only when that
    final flight needs no more battery than it carries.
    """
    d = derived(p)
    t_f = float(p["t_f"])
    analytic = sizing.solve_fixed_area(p)
    config = dict(mission=mission_kind, t_window=t_window, dt=dt, settle=settle,
                  seed=seed, governor=dict(governor or {}), tol=tol,
                  max_iter=max_iter, t_f=t_f, soc_reserve=float(p["soc_reserve"]))

    if m_bat0 is None:
        m_bat0 = analytic.m_bat if analytic.feasible and analytic.m_bat > 0 else \
            max(0.25, d.m_fix)
    m_bat = float(m_bat0)
    cap = cap_ratio * max(m_bat0, 1e-3)

    trace: List[Dict[str, float]] = []
    calls = [0]
    warnings: List[str] = []
    status = "slow"
    converged = diverged = False
    reason = ""
    prev = None          # (m_bat, gap) of the previous iterate, for the secant step

    def fly(mb: float):
        calls[0] += 1
        mm = gross_mass_for_battery(p, mb)
        return mm, simulate_flight(p, mm, m_bat=mb, mission_kind=mission_kind,
                                   t_window=t_window, dt=dt, governor=governor,
                                   settle=settle, seed=seed)

    def record(it: int, mm: float, mb: float, sim, req: Dict[str, float],
               kind: str) -> None:
        trace.append(dict(iteration=it, kind=kind, m=mm, m_bat=mb,
                          m_bat_required=req["m_req"], limit=req["limit"],
                          residual=mb - req["m_req"],
                          energy_margin=d.e_b * (1.0 - req["reserve"]) * mb
                          - req["E_mission"],
                          power_margin=float(p["p_b_w_kg"]) * mb - sim.P_peak,
                          P_mean=sim.P_mean, P_peak=sim.P_peak,
                          E_mission=req["E_mission"], overhead=sim.overhead,
                          e_track_rms=sim.e_track_rms))

    sim = None
    m = gross_mass_for_battery(p, m_bat)
    for it in range(max_iter):
        m, sim = fly(m_bat)
        if not sim.ok:
            return ClosureRun(False, False, "failed", sim.reason, trace=trace,
                              m=m, m_bat=m_bat, sim_calls=calls[0],
                              analytic=analytic.to_dict(), config=config)
        req = _requirement(p, sim, d.e_b)
        record(it, m, m_bat, sim, req, "update")
        m_bat_req = req["m_req"]

        if abs(m_bat_req - m_bat) < tol * max(1.0, m_bat):
            converged = True
            break
        if m_bat_req > cap or not math.isfinite(m_bat_req):
            diverged = True
            status = "diverging"
            reason = ("battery mass ran away past %.1f kg: the energy the mission "
                      "needs grows faster than the mass added to carry it" % cap)
            break

        # Plain iteration on this map contracts only as fast as its slope,
        # which near the fold approaches one. A secant step on the same
        # residual costs no extra simulation and converges superlinearly.
        gap = m_bat_req - m_bat
        step = relax * gap
        if prev is not None:
            m_prev, gap_prev = prev
            dg = gap - gap_prev
            dm = m_bat - m_prev
            if abs(dg) > 1e-12 and abs(dm) > 1e-12:
                secant = -gap * dm / dg
                # Only trust it while it stays a sane, same-direction move.
                if math.isfinite(secant) and secant * gap > 0 and \
                        abs(secant) < 6.0 * abs(gap) + 1e-9:
                    step = secant
        prev = (m_bat, gap)
        m_bat = max(m_bat + step, 1e-4)

    iterations = len(trace)
    if converged:
        # Round the battery up to what the last flight needed and verify. The
        # map has slope below one on the light branch, so each verification
        # overshoots less than the last; three attempts are ample.
        m_bat = max(m_bat, trace[-1]["m_bat_required"])
        for _ in range(4):
            m, sim = fly(m_bat)
            if not sim.ok:
                return ClosureRun(False, False, "failed", sim.reason, trace=trace,
                                  m=m, m_bat=m_bat, sim_calls=calls[0],
                                  analytic=analytic.to_dict(), config=config)
            req = _requirement(p, sim, d.e_b)
            record(len(trace), m, m_bat, sim, req, "verify")
            if req["m_req"] <= m_bat * (1.0 + 1e-9):
                status = "converged"
                break
            m_bat = req["m_req"] * (1.0 + 0.25 * tol)
        else:
            converged = False
            status = "slow"
            reason = "the verification flight kept needing more battery than it carried"
    elif not diverged and len(trace) >= 3:
        d1 = trace[-2]["m_bat_required"] - trace[-2]["m_bat"]
        d2 = trace[-1]["m_bat_required"] - trace[-1]["m_bat"]
        if abs(d1) > 1e-12 and abs(d2 / d1) >= 1.0 and abs(d2) > tol:
            diverged = True
            status = "diverging"
            reason = "the fixed point map is expanding: successive corrections grow"

    m = gross_mass_for_battery(p, m_bat)
    bd = comp.breakdown(p, m, m_bat=m_bat)
    warnings.extend(bd.warnings)
    if sim is not None:
        warnings.extend(sim.warnings)

    sim_dict: Dict[str, Any] = {}
    energy_margin = power_margin = reserve_end = 0.0
    battery_ok = constraints_ok = False
    if sim is not None:
        sim_dict = sim.to_dict()
        if not keep_telemetry:
            sim_dict.pop("telemetry", None)
            sim_dict.pop("trajectory", None)
            sim_dict.pop("t", None)
        last = trace[-1]
        energy_margin = last["energy_margin"]
        power_margin = last["power_margin"]
        reserve_end = 1.0 - sim.P_mean * t_f / max(d.e_b * m_bat, 1e-12)
        battery_ok = energy_margin >= -1e-9 * max(1.0, abs(last["E_mission"])) \
            and power_margin >= -1e-9
        constraints_ok = bool(sim.constraints_ok)

    return ClosureRun(
        converged=converged, diverged=diverged, status=status, reason=reason,
        trace=trace, m=m, m_bat=m_bat,
        P_mean=sim.P_mean if sim else 0.0,
        E_mission=(sim.P_mean * t_f) if sim else 0.0,
        overhead=sim.overhead if sim else 1.0,
        iterations=iterations, sim_calls=calls[0],
        energy_margin=energy_margin, power_margin=power_margin,
        reserve_end=reserve_end, battery_ok=battery_ok,
        constraints_ok=constraints_ok,
        feasible=bool(converged and battery_ok and constraints_ok),
        sim=sim_dict, design=bd.to_dict(), analytic=analytic.to_dict(),
        config=config, warnings=warnings,
    )
