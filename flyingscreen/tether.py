"""
The tethered closure: what happens when the energy stops being onboard.

Removing the battery deletes the term that carries the m^(3/2) and makes
the design fold.  What replaces it is a conductor whose mass is set by the
power it must carry and the voltage it carries it at:

    I  = P / V
    R  = rho_e * 2 L / A_c
    dV = I R <= drop * V        ->  A_c = 2 rho_e L P / (drop V^2)
    m_c = rho_m A_c * 2 L       ->  m_c = 4 rho_e rho_m L^2 P / (drop V^2)

so conductor mass scales as L^2 / V^2 and does not contain the mission
time. Endurance stops being a design variable; what is bought instead is a
length limit and a voltage.

The closure keeps the same algebraic shape,

    alpha m - beta_t m^(3/2) = M0_t

so for any positive beta_t the fold is still there: it has moved, not gone.
beta_t is smaller than the battery beta by orders of magnitude at these
lengths and voltages, which puts the fold far away. The m^(3/2) term
vanishes only when the carried cable mass stops depending on power, either
because the tether is supported from above or because the conductor sits at
its minimum practical gauge. The minimum-gauge regime is linear only while
the electrical requirement stays below that gauge, which the solver checks.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Any, Dict

from . import sizing
from .params import derived


def tether_constants(p: Dict[str, float]) -> Dict[str, float]:
    L = float(p["tether_len"])
    V = float(p["tether_V"])
    drop = float(p["tether_drop"])
    rho_e = float(p["tether_rho_e"])
    rho_m = float(p["tether_rho_m"])
    insul = float(p["tether_insul"])
    eta_dc = float(p["eta_dcdc"])
    # kg of tether per watt delivered to the aircraft bus
    k_mass_per_W = insul * 4.0 * rho_e * rho_m * L * L / (drop * V * V)
    # source power per watt used onboard
    k_source = 1.0 / (eta_dc * (1.0 - drop))
    return dict(L=L, V=V, drop=drop, k_mass_per_W=k_mass_per_W,
                k_source=k_source, insul=insul, eta_dc=eta_dc,
                rho_e=rho_e, rho_m=rho_m)


# Below about AWG 26 a conductor stops being a cable and starts being a
# fuse: handling, flex life and connector strain set a floor that the
# electrical calculation does not know about.
A_MIN_CONDUCTOR = 0.13e-6          # m^2


def solve_tethered(p: Dict[str, float]) -> Dict[str, Any]:
    d = derived(p)
    g = float(p["g"])
    tc = tether_constants(p)
    support = float(p["tether_support"])

    gamma_m = float(p["lam"]) * g / float(p["S_m"])
    alpha = 1.0 - float(p["f_s"]) - gamma_m
    c = g ** 1.5 / (float(p["FM"]) * float(p["eta"]) *
                    math.sqrt(2.0 * float(p["rho"]) * d.A))

    # mass carried per watt of onboard load
    k_carry = support * tc["k_mass_per_W"] * tc["k_source"]
    beta_t = k_carry * c
    M0_t = d.m_fix + k_carry * d.P_aux
    gauge_limited = False

    def area_needed(m_: float) -> float:
        P_src_ = (c * m_ ** 1.5 + d.P_aux) * tc["k_source"]
        return 2.0 * tc["rho_e"] * tc["L"] * P_src_ / (tc["drop"] * tc["V"] ** 2)

    res = sizing.solve_power_closure(alpha, beta_t, M0_t, 1.5)
    fold = dict(m_star=res.m_star, M0_max=res.M0_max, beta=beta_t)
    if res.feasible and res.m_light is not None and area_needed(res.m_light) < A_MIN_CONDUCTOR:
        # The tether is at its minimum practical gauge, so its mass no longer
        # depends on power and the closure becomes linear. That holds only
        # while the linear solution still needs less than the minimum gauge,
        # which is checked rather than assumed.
        m_teth_fixed = tc["insul"] * tc["rho_m"] * A_MIN_CONDUCTOR * 2.0 * tc["L"]
        lin = sizing.solve_power_closure(alpha, 0.0, d.m_fix + support * m_teth_fixed, 1.5)
        if lin.feasible and lin.m_light is not None and \
                area_needed(lin.m_light) <= A_MIN_CONDUCTOR:
            gauge_limited = True
            beta_t = 0.0
            M0_t = d.m_fix + support * m_teth_fixed
            res = lin

    out: Dict[str, Any] = dict(feasible=res.feasible, reason=res.reason,
                               alpha=alpha, beta_tether=beta_t, M0=M0_t,
                               closure=asdict(res), constants=tc,
                               support=support, gauge_limited=gauge_limited,
                               fold_retained=bool(beta_t > 0.0),
                               fold_power_limited=fold)
    if not res.feasible or res.m_light is None:
        return out

    m = res.m_light
    P_prop = c * m ** 1.5
    P_bus = P_prop + d.P_aux
    P_source = P_bus * tc["k_source"]
    I = P_source / tc["V"]
    A_c = max(2.0 * tc["rho_e"] * tc["L"] * P_source / (tc["drop"] * tc["V"] ** 2),
              A_MIN_CONDUCTOR)
    m_conductor = tc["rho_m"] * A_c * 2.0 * tc["L"]
    m_tether = tc["insul"] * m_conductor
    dia = 2.0 * math.sqrt(A_c / math.pi) * 1000.0        # per conductor, mm
    loss = P_source - P_bus

    out.update(dict(
        m=m, m_fix=d.m_fix, m_str=float(p["f_s"]) * m, m_prop=gamma_m * m,
        m_tether_carried=support * m_tether, m_tether_total=m_tether,
        P_prop=P_prop, P_bus=P_bus, P_source=P_source, P_loss=loss,
        current=I, conductor_area_mm2=A_c * 1e6, conductor_dia_mm=dia,
        tether_mass_per_m=m_tether / max(tc["L"], 1e-9),
        disk_loading=m * g / d.A,
        endurance="unlimited",
        M0_max=res.M0_max, m_star=res.m_star,
    ))
    return out


def close_tethered_first_principles(p: Dict[str, float],
                                    reserve_seconds: float = 90.0,
                                    iters: int = 300) -> Dict[str, Any]:
    """
    The tethered closure using the same component submodels as the battery
    one, so the two can be put side by side honestly.

    Off the wire the battery shrinks to a landing reserve. That reserve still
    has to deliver full power, so it is sized by the larger of its energy and
    its peak power, and it reintroduces a power-dependent mass term. This is
    a redesigned tethered aircraft: propulsion and structure are resized
    around the new mass, which is not the same thing as removing the battery
    from the battery design.

    Reported feasibility requires the mass iteration to have converged, the
    conductor current density to be sane, and the voltage delivered at the
    aircraft to exceed the bus voltage the motors were sized for.
    """
    from . import components as comp
    from .electrical import electrical_chain

    d = derived(p)
    N = float(p["N_rotors"])
    tc = tether_constants(p)
    support = float(p["tether_support"])
    L, V, drop = tc["L"], tc["V"], tc["drop"]
    reserve = min(max(float(p["soc_reserve"]), 0.0), 0.9)

    m = d.m_fix + 0.3
    m_teth = P_bus = P_src = A_c = m_res = 0.0
    res_limit = "energy"
    converged = False
    for _ in range(iters):
        pr = comp.propulsion(p, m)
        arm = comp.arm_structure(
            p, m, m_tip=pr.m_motor_each + (pr.m_esc + pr.m_props) / N)
        P_bus = N * pr.rotor_hover.P_elec + d.P_aux
        P_src = P_bus * tc["k_source"]
        # The conductor is sized by three requirements at once: voltage
        # drop, continuous current density (ampacity), and handling. The
        # largest wins, and its mass is what the aircraft carries.
        A_drop = 2.0 * tc["rho_e"] * L * P_src / (drop * V * V)
        A_amp = (P_src / V) / (J_MAX_A_MM2 * 1e6)
        A_c = max(A_drop, A_amp, A_MIN_CONDUCTOR)
        m_teth = tc["insul"] * tc["rho_m"] * A_c * 2.0 * L
        m_res_energy = reserve_seconds * P_bus / (d.e_b * (1.0 - reserve))
        m_res_power = (pr.P_elec_max + d.P_aux) / float(p["p_b_w_kg"])
        m_res = max(m_res_energy, m_res_power)
        res_limit = "power" if m_res_power > m_res_energy else "energy"
        m_new = (d.m_fix + arm.m_frame + pr.m_prop_total + m_res
                 + support * m_teth)
        if not math.isfinite(m_new) or m_new > 1e4:
            break
        if abs(m_new - m) < 1e-10 * max(1.0, m):
            m = m_new
            converged = True
            break
        m = m_new

    b = comp.breakdown(p, m, m_bat=m_res)
    current = P_src / V
    density = current / (A_c * 1e6)                  # A/mm^2
    v_delivered = V * (1.0 - drop)
    # Informational: the lowest bus voltage at which the motors can still be
    # wound with a practical KV. The delivered voltage only has to exceed
    # it, since an onboard buck converter takes up the rest; this is not a
    # design check, it states which converter topology is needed.
    try:
        chain = electrical_chain(p, m, m_res)
        usable = [o for o in chain["options"] if o["KV"] >= 45.0]
        v_bus = min(o["v_full"] for o in usable) if usable else float("nan")
    except Exception:
        v_bus = float("nan")
    sized_by = ("ampacity" if A_c == A_amp and A_amp > A_drop else
                "handling" if A_c == A_MIN_CONDUCTOR else "voltage drop")
    checks = dict(
        converged=converged,
        current_density_ok=bool(density <= J_MAX_A_MM2 * (1 + 1e-9)),
        bus_reachable=bool(math.isfinite(v_bus) and v_delivered >= v_bus),
        selv=bool(V <= 60.0),
    )
    feasible = checks["converged"] and checks["current_density_ok"]
    return dict(
        feasible=feasible, checks=checks, m=m, m_reserve=m_res,
        reserve_limit=res_limit, m_tether_total=m_teth,
        m_tether_carried=support * m_teth, P_bus=P_bus, P_source=P_src,
        P_loss=P_src - P_bus, current=current, current_density=density,
        v_delivered=v_delivered, v_bus=v_bus, conductor_sized_by=sized_by,
        conductor_area_mm2=A_c * 1e6,
        conductor_dia_mm=2.0 * math.sqrt(A_c / math.pi) * 1000.0,
        gauge_limited=A_c <= A_MIN_CONDUCTOR * (1 + 1e-9),
        spl_1m=b.spl_1m, downwash=b.downwash, disk_loading=b.disk_loading,
        span=b.span, reserve_seconds=reserve_seconds,
        breakdown=b.to_dict(), endurance="unlimited while connected",
    )


# Continuous current density for a thin insulated copper lead in free air.
# Chassis-wiring tables allow about 17 A/mm^2 for AWG 26 in free air; the
# value here leaves margin for a bundled, flexing, possibly coiled lead.
J_MAX_A_MM2 = 10.0


def compare(p: Dict[str, float]) -> Dict[str, Any]:
    """Battery design against tethered design at the same assumptions."""
    bat = sizing.solve_fixed_area(p)
    teth = solve_tethered(p)
    wall = sizing.payload_wall(p)
    return dict(
        battery=bat.to_dict(), tether=teth, wall=wall,
        note=("The tethered closure contains no mission time. Endurance stops "
              "being a design variable and becomes an operating choice."),
    )
