"""
Layer 1-2: momentum theory and algebraic mass closure.

This module implements the closed-form theory exactly as derived, so that
every numerical result elsewhere in the engine has an analytical reference
to be checked against.

The master sizing equation, for a rotor system whose total disk area is
held fixed while gross mass varies, is

    alpha * m  -  beta * m^(3/2)  =  M0

with

    alpha = 1 - f_s - lam g / S_m                       (thrust coupling)
    beta  = t_f g^(3/2) / (e_b FM eta sqrt(2 rho A))    (energy coupling)
    M0    = m_fix + P_aux t_f / e_b                     (effective fixed load)

where e_b is the energy the mission may use per kilogram of pack: cell
specific energy times depth of discharge times (1 - landing reserve).

The left hand side is not monotonic: it peaks and falls, so there is a hard
maximum on M0 beyond which no equilibrium mass exists at all.

    m*        = 4 alpha^2 / (9 beta^2)
    M0_max    = 4 alpha^3 / (27 beta^2)

More generally, if disk area is allowed to grow with gross mass as
A ~ m^p, hover power becomes P ~ m^q with q = (3 - p) / 2 and the closure is

    alpha * m - beta_q * m^q = M0

which folds only when q > 1, that is p < 1.  At p = 1 (constant disk
loading) the closure is linear and the fold disappears entirely, replaced by
the much gentler feasibility condition f_s + gamma_m + gamma_b < 1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict, field
from typing import Any, Dict, List, Optional

from .params import Derived, derived

SQRT2 = math.sqrt(2.0)


# ---------------------------------------------------------------------------
# Coefficients
# ---------------------------------------------------------------------------
@dataclass
class Coeffs:
    alpha: float          # 1 - f_s - lam g / S_m                    [-]
    beta: float           # energy coupling for the fixed-area model [kg^-1/2]
    M0: float             # effective fixed load                     [kg]
    q: float              # power-law exponent of the battery term   [-]
    beta_q: float         # coefficient in front of m^q              [kg^(1-q)]
    gamma_m: float        # lam g / S_m, propulsion mass fraction    [-]
    f_s: float            # structural mass fraction                 [-]
    A: float              # disk area used for beta                  [m^2]
    e_b: float            # usable specific energy                   [J/kg]
    m_fix: float          # fixed mass                               [kg]
    P_aux: float          # auxiliary electrical power               [W]
    m_bat_aux: float      # battery mass carried purely for P_aux    [kg]
    c_P: float            # hover power coefficient, P = c_P m^q     [W/kg^q]
    anchor_mass: float    # anchor for the A ~ m^p power law         [kg]


def _power_coefficient(p: Dict[str, float], d: Derived, A: float) -> float:
    """c in P_prop = c * m^(3/2) for a fixed disk area A."""
    g = float(p["g"])
    return g ** 1.5 / (float(p["FM"]) * float(p["eta"]) * math.sqrt(2.0 * float(p["rho"]) * A))


def coefficients(p: Dict[str, float], area_p: Optional[float] = None,
                 anchor_mass: Optional[float] = None) -> Coeffs:
    """alpha, beta, M0 and the generalised power-law coefficients."""
    d = derived(p)
    g = float(p["g"])
    gamma_m = float(p["lam"]) * g / float(p["S_m"])
    alpha = 1.0 - float(p["f_s"]) - gamma_m
    t_f = float(p["t_f"])
    # Energy available to the mission: the landing reserve is held back, the
    # same accounting as the component closure and the simulation loop use.
    e_b = d.e_b * (1.0 - min(max(float(p.get("soc_reserve", 0.0)), 0.0), 0.9))

    c_fixed = _power_coefficient(p, d, d.A)
    beta = t_f * c_fixed / e_b

    pp = float(p["area_p"]) if area_p is None else float(area_p)
    q = (3.0 - pp) / 2.0

    # Anchor the A ~ m^p law at a well defined mass so that every value of p
    # describes the same physical rotor at that anchor point.  Preference is
    # the fixed-area solution; if that does not exist we use the fold mass,
    # which is always defined.
    if anchor_mass is None:
        anchor_mass = _anchor_mass(alpha, beta, d.m_fix + d.P_aux * t_f / e_b)
    anchor_mass = max(anchor_mass, 1e-6)

    c_q = c_fixed * anchor_mass ** (pp / 2.0)
    beta_q = t_f * c_q / e_b

    M0 = d.m_fix + d.P_aux * t_f / e_b
    return Coeffs(
        alpha=alpha, beta=beta, M0=M0, q=q, beta_q=beta_q,
        gamma_m=gamma_m, f_s=float(p["f_s"]), A=d.A, e_b=e_b,
        m_fix=d.m_fix, P_aux=d.P_aux, m_bat_aux=d.P_aux * t_f / e_b,
        c_P=c_q, anchor_mass=anchor_mass,
    )


def _anchor_mass(alpha: float, beta: float, M0: float) -> float:
    """Fixed-area solution if it exists, else the fold mass."""
    if alpha <= 0.0 or beta <= 0.0:
        return max(M0, 1e-3)
    m_star = 4.0 * alpha * alpha / (9.0 * beta * beta)
    M0_max = 4.0 * alpha ** 3 / (27.0 * beta * beta)
    if M0 < M0_max:
        r = solve_power_closure(alpha, beta, M0, 1.5)
        if r.feasible and r.m_light is not None:
            return r.m_light
    return m_star


# ---------------------------------------------------------------------------
# Solving  alpha m - beta m^q = M0
# ---------------------------------------------------------------------------
@dataclass
class ClosureResult:
    feasible: bool
    m_light: Optional[float] = None     # physically meaningful branch  [kg]
    m_heavy: Optional[float] = None     # upper, unstable branch        [kg]
    m_star: Optional[float] = None      # fold mass                     [kg]
    M0_max: Optional[float] = None      # maximum supportable fixed load[kg]
    margin: Optional[float] = None      # M0_max - M0                   [kg]
    utilisation: Optional[float] = None # M0 / M0_max                   [-]
    has_fold: bool = True
    reason: str = ""


def solve_power_closure(alpha: float, beta: float, M0: float, q: float) -> ClosureResult:
    """
    Solve alpha m - beta m^q = M0 for m > 0.

    q > 1  : folds, up to two roots, the light one is the design.
    q == 1 : linear, unique root if alpha > beta.
    q < 1  : unique root, no fold (constant or growing disk loading).
    """
    if M0 <= 0.0:
        return ClosureResult(False, reason="fixed load must be positive")
    if alpha <= 0.0:
        return ClosureResult(
            False, has_fold=(q > 1.0), m_star=None, M0_max=0.0,
            reason="alpha <= 0: structure plus propulsion already consume the whole mass budget")

    def F(m: float) -> float:
        # m^q written as m * m^(q-1) so that a huge m with q close to one
        # does not overflow before the difference is formed.
        if m <= 0.0:
            return 0.0
        return m * (alpha - beta * math.exp((q - 1.0) * math.log(m)))

    # ---- linear case ------------------------------------------------------
    if abs(q - 1.0) < 1e-12:
        denom = alpha - beta
        if denom <= 0.0:
            return ClosureResult(False, has_fold=False,
                                 reason="f_s + gamma_m + gamma_b >= 1: mass diverges")
        m = M0 / denom
        return ClosureResult(True, m_light=m, has_fold=False, m_star=None,
                             M0_max=None, margin=None, utilisation=None)

    if beta <= 0.0:
        return ClosureResult(True, m_light=M0 / alpha, has_fold=False)

    # ---- folding case, q > 1 ---------------------------------------------
    if q > 1.0:
        # m* = (alpha / (q beta))^(1/(q-1)) in log space: as q -> 1+ the
        # exponent grows without bound and the fold recedes to infinity,
        # which is the p -> 1- limit of the endurance-wall proposition.
        log_m_star = math.log(alpha / (q * beta)) / (q - 1.0)
        if log_m_star > _LOG_M_CAP:
            # F is increasing up to the (unrepresentable) fold, and there
            # F(m) is close to alpha m, so the light root is near M0/alpha.
            # Bracket it from there outward: bisecting [1e-12, 1e250] in
            # linear space cannot resolve a root of order one.
            hi = 2.0 * M0 / alpha
            while F(hi) - M0 <= 0.0 and math.log(hi) < _LOG_M_CAP:
                hi *= 2.0
            m_light = _brentq(lambda m: F(m) - M0, 1e-12, hi)
            return ClosureResult(True, m_light=m_light, m_heavy=None,
                                 m_star=math.inf, M0_max=math.inf, margin=math.inf,
                                 utilisation=0.0, has_fold=True,
                                 reason="the fold lies beyond %.0e kg" % math.exp(_LOG_M_CAP))
        # The mirror case: with alpha < q beta the fold mass underflows as
        # q -> 1+, the critical load is zero, and no design exists.
        m_star = math.exp(log_m_star)
        M0_max = F(m_star)
        if M0 > M0_max:
            return ClosureResult(False, m_star=m_star, M0_max=M0_max,
                                 margin=M0_max - M0,
                                 utilisation=M0 / M0_max if M0_max > 0.0 else math.inf,
                                 has_fold=True,
                                 reason="no equilibrium mass exists: fixed load exceeds the fold")
        m_light = _brentq(lambda m: F(m) - M0, 1e-12, m_star)
        hi = m_star
        for _ in range(200):
            hi *= 1.7
            if F(hi) - M0 < 0.0:
                break
        m_heavy = _brentq(lambda m: F(m) - M0, m_star, hi)
        return ClosureResult(True, m_light=m_light, m_heavy=m_heavy, m_star=m_star,
                             M0_max=M0_max, margin=M0_max - M0,
                             utilisation=M0 / M0_max, has_fold=True)

    # ---- q < 1: monotone increasing beyond the minimum --------------------
    log_m_min = math.log(q * beta / alpha) / (1.0 - q)
    if log_m_min >= _LOG_M_CAP:
        # A root exists for every load when q < 1, but here it lies beyond
        # any mass the solver represents. Say so rather than overflow.
        return ClosureResult(True, m_light=math.inf, has_fold=False,
                             reason="the root lies beyond %.0e kg" % math.exp(_LOG_M_CAP))
    m_min = math.exp(log_m_min)
    lo = m_min
    hi = max(m_min * 2.0, 1.0)
    while F(hi) - M0 <= 0.0:
        if math.log(hi) >= _LOG_M_CAP:
            return ClosureResult(True, m_light=math.inf, has_fold=False,
                                 reason="the root lies beyond %.0e kg" % math.exp(_LOG_M_CAP))
        hi *= 1.7
    m = _brentq(lambda mm: F(mm) - M0, lo, hi)
    return ClosureResult(True, m_light=m, has_fold=False)


# Largest mass the closed-form solvers work with, as a logarithm. Masses
# beyond it are reported as infinite rather than overflowing.
_LOG_M_CAP = math.log(1e250)


def _brentq(f, a: float, b: float, tol: float = 1e-12, maxiter: int = 200) -> float:
    """Small dependency-free Brent solver (bisection fallback)."""
    fa, fb = f(a), f(b)
    if fa == 0.0:
        return a
    if fb == 0.0:
        return b
    if fa * fb > 0.0:
        # Shrink towards the endpoint with the smaller magnitude and retry.
        raise ValueError("root not bracketed: f(%g)=%g f(%g)=%g" % (a, fa, b, fb))
    for _ in range(maxiter):
        m = 0.5 * (a + b)
        fm = f(m)
        if fm == 0.0 or (b - a) < tol * max(1.0, abs(m)):
            return m
        if fa * fm < 0.0:
            b, fb = m, fm
        else:
            a, fa = m, fm
    return 0.5 * (a + b)


# ---------------------------------------------------------------------------
# Full design point
# ---------------------------------------------------------------------------
@dataclass
class DesignPoint:
    feasible: bool
    reason: str = ""
    model: str = "fixed_area"
    m: float = 0.0                # gross mass                    [kg]
    m_fix: float = 0.0
    m_str: float = 0.0
    m_prop: float = 0.0
    m_bat: float = 0.0
    m_bat_energy: float = 0.0     # battery mass required by energy
    m_bat_power: float = 0.0      # battery mass required by peak power
    battery_limit: str = "energy"
    P_prop: float = 0.0           # hover propulsive electrical power [W]
    P_aux: float = 0.0
    P_total: float = 0.0
    P_peak: float = 0.0           # electrical power at T_max         [W]
    E_required: float = 0.0       # J
    E_pack: float = 0.0           # J
    A: float = 0.0                # disk area used                    [m^2]
    D_rotor: float = 0.0          # rotor diameter implied            [m]
    disk_loading: float = 0.0     # N/m^2
    v_induced: float = 0.0        # m/s
    T_hover_per_rotor: float = 0.0
    T_max: float = 0.0
    power_loading: float = 0.0    # N/W, useful figure of merit
    hover_specific_power: float = 0.0  # W/kg
    endurance_check: float = 0.0  # s, energy / hover power
    dmdm: float = 0.0             # d(m_str+m_prop+m_bat)/dm, must be < 1
    stable: bool = True
    coeffs: Dict[str, float] = field(default_factory=dict)
    closure: Dict[str, Any] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _fill_point(p: Dict[str, float], c: Coeffs, m: float, model: str,
                A_of_m) -> DesignPoint:
    g = float(p["g"])
    rho = float(p["rho"])
    N = float(p["N_rotors"])
    A = A_of_m(m)
    P_prop = (m * g) ** 1.5 / (float(p["FM"]) * float(p["eta"]) * math.sqrt(2.0 * rho * A))
    P_total = P_prop + c.P_aux
    E_required = P_total * float(p["t_f"])
    m_bat_energy = E_required / c.e_b
    lam = float(p["lam"])
    P_peak = P_prop * lam ** 1.5
    m_bat_power = P_peak / float(p["p_b_w_kg"])
    m_bat = max(m_bat_energy, m_bat_power)
    m_str = c.f_s * m
    m_prop = c.gamma_m * m
    T_max = lam * m * g

    dp = DesignPoint(
        feasible=True, model=model, m=m, m_fix=c.m_fix, m_str=m_str,
        m_prop=m_prop, m_bat=m_bat, m_bat_energy=m_bat_energy,
        m_bat_power=m_bat_power,
        battery_limit="power" if m_bat_power > m_bat_energy else "energy",
        P_prop=P_prop, P_aux=c.P_aux, P_total=P_total, P_peak=P_peak,
        E_required=E_required, E_pack=m_bat * c.e_b, A=A,
        D_rotor=math.sqrt(4.0 * A / (math.pi * N)),
        disk_loading=m * g / A,
        v_induced=math.sqrt(m * g / (2.0 * rho * A)),
        T_hover_per_rotor=m * g / N, T_max=T_max,
        power_loading=(m * g) / max(P_total, 1e-9),
        hover_specific_power=P_total / max(m, 1e-9),
        endurance_check=m_bat * c.e_b / max(P_total, 1e-9),
        coeffs=asdict(c),
    )
    # local stability of the sizing fixed point: d/dm [m_str + m_prop + m_bat] < 1
    h = max(1e-6, 1e-6 * m)
    def battery_of(mm: float) -> float:
        AA = A_of_m(mm)
        PP = (mm * g) ** 1.5 / (float(p["FM"]) * float(p["eta"]) * math.sqrt(2.0 * rho * AA))
        return (PP + c.P_aux) * float(p["t_f"]) / c.e_b
    dmbat = (battery_of(m + h) - battery_of(m - h)) / (2.0 * h)
    dp.dmdm = c.f_s + c.gamma_m + dmbat
    dp.stable = dp.dmdm < 1.0

    # closure residual as a sanity check
    resid = m - (c.m_fix + m_str + m_prop + m_bat)
    if abs(resid) > 1e-6 * max(1.0, m):
        if dp.battery_limit == "power":
            dp.warnings.append(
                "battery is peak-power limited, so the pack is heavier than the energy "
                "closure alone requires; gross mass is not self-consistent unless resized")
        else:
            dp.warnings.append("mass closure residual %.3e kg" % resid)
    if dp.m_bat / max(m, 1e-9) > 0.6:
        dp.warnings.append("battery is more than 60 percent of gross mass")
    if not dp.stable:
        dp.warnings.append("sizing fixed point is unstable: d(m_str+m_prop+m_bat)/dm >= 1")
    return dp


def solve_fixed_area(p: Dict[str, float]) -> DesignPoint:
    """The p = 0 model: rotors do not grow with the aircraft."""
    c = coefficients(p, area_p=0.0)
    res = solve_power_closure(c.alpha, c.beta, c.M0, 1.5)
    if not res.feasible or res.m_light is None:
        return DesignPoint(feasible=False, reason=res.reason, model="fixed_area",
                           coeffs=asdict(c), closure=asdict(res), P_aux=c.P_aux,
                           m_fix=c.m_fix, A=c.A)
    dp = _fill_point(p, c, res.m_light, "fixed_area", lambda m: c.A)
    dp.closure = asdict(res)
    return dp


def solve_power_law(p: Dict[str, float], area_p: Optional[float] = None,
                    anchor_mass: Optional[float] = None) -> DesignPoint:
    """
    A ~ m^p with the exponent taken from the parameters (or overridden).

    The power law is anchored at a reference mass where A equals the stated
    rotor area, so every exponent describes the same physical rotor at that
    point.  Pass anchor_mass explicitly to hold the anchor fixed while the
    exponent is swept; otherwise it is re-derived per call and every
    exponent trivially reproduces the same design.
    """
    pp = float(p["area_p"]) if area_p is None else float(area_p)
    c = coefficients(p, area_p=pp, anchor_mass=anchor_mass)
    A_of_m = lambda m: c.A * (m / c.anchor_mass) ** pp
    res = solve_power_closure(c.alpha, c.beta_q, c.M0, c.q)
    if not res.feasible or res.m_light is None:
        return DesignPoint(feasible=False, reason=res.reason, model="power_law",
                           coeffs=asdict(c), closure=asdict(res), P_aux=c.P_aux,
                           m_fix=c.m_fix, A=c.A)
    dp = _fill_point(p, c, res.m_light, "power_law", A_of_m)
    dp.closure = asdict(res)
    return dp


def solve_constant_dl(p: Dict[str, float]) -> DesignPoint:
    """
    Constant disk loading: A = m g / DL exactly, so P = c_P m and the closure
    is linear.  Feasibility becomes f_s + gamma_m + gamma_b < 1.
    """
    c = coefficients(p, area_p=1.0)
    g = float(p["g"])
    DL = float(p["disk_loading"])
    c_P = g / (float(p["FM"]) * float(p["eta"])) * math.sqrt(DL / (2.0 * float(p["rho"])))
    gamma_b = c_P * float(p["t_f"]) / c.e_b
    denom = 1.0 - c.f_s - c.gamma_m - gamma_b
    cc = asdict(c)
    cc.update(dict(c_P=c_P, gamma_b=gamma_b, denom=denom, q=1.0, beta_q=gamma_b))
    if denom <= 0.0:
        return DesignPoint(
            feasible=False, model="constant_dl",
            reason="f_s + gamma_m + gamma_b = %.3f >= 1: mass diverges" % (1.0 - denom),
            coeffs=cc, m_fix=c.m_fix, P_aux=c.P_aux,
            closure=dict(feasible=False, has_fold=False, gamma_b=gamma_b, denom=denom))
    m = c.M0 / denom
    dp = _fill_point(p, c, m, "constant_dl", lambda mm: mm * g / DL)
    dp.coeffs = cc
    dp.closure = dict(feasible=True, has_fold=False, gamma_b=gamma_b, denom=denom,
                      m_light=m, gamma_sum=1.0 - denom)
    if dp.D_rotor > 1.2:
        dp.warnings.append(
            "rotor diameter of %.2f m per rotor: the binding constraint has moved from "
            "energy to geometry and safety" % dp.D_rotor)
    return dp


def solve(p: Dict[str, float], model: str = "fixed_area") -> DesignPoint:
    if model == "fixed_area":
        return solve_fixed_area(p)
    if model == "constant_dl":
        return solve_constant_dl(p)
    if model == "power_law":
        return solve_power_law(p)
    raise ValueError("unknown sizing model: %s" % model)


# ---------------------------------------------------------------------------
# Curves, walls and sensitivities
# ---------------------------------------------------------------------------
def closure_curve(p: Dict[str, float], model: str = "fixed_area",
                  n: int = 320, m_max: Optional[float] = None) -> Dict[str, Any]:
    """
    Sample F(m) = alpha m - beta m^q, the supportable fixed load as a function
    of gross mass.  This is the curve whose peak is the feasibility wall.
    """
    if model == "constant_dl":
        c = coefficients(p, area_p=1.0)
        g = float(p["g"])
        DL = float(p["disk_loading"])
        c_P = g / (float(p["FM"]) * float(p["eta"])) * math.sqrt(DL / (2.0 * float(p["rho"])))
        gamma_b = c_P * float(p["t_f"]) / c.e_b
        alpha, beta_q, q = c.alpha, gamma_b, 1.0
    else:
        pp = 0.0 if model == "fixed_area" else float(p["area_p"])
        c = coefficients(p, area_p=pp)
        alpha, beta_q, q = c.alpha, c.beta_q, c.q

    res = solve_power_closure(alpha, beta_q, c.M0, q)
    if q > 1.0 and alpha > 0.0 and beta_q > 0.0 and res.m_star is not None \
            and math.isfinite(res.m_star):
        m_star = res.m_star
        top = m_star * 2.4 if m_max is None else m_max
    else:
        base = res.m_light if res.m_light else max(c.M0 / max(alpha, 1e-6), 1.0)
        m_star = None
        top = base * 3.0 if m_max is None else m_max
    top = max(top, 1e-3)

    ms, Fs = [], []
    for i in range(n + 1):
        m = top * i / n
        ms.append(m)
        Fs.append(alpha * m - beta_q * (m ** q) if m > 0 else 0.0)
    return dict(
        model=model, m=ms, F=Fs, M0=c.M0, alpha=alpha, beta=beta_q, q=q,
        m_star=m_star, M0_max=res.M0_max, feasible=res.feasible,
        m_light=res.m_light, m_heavy=res.m_heavy, has_fold=res.has_fold,
        reason=res.reason,
    )


def payload_wall(p: Dict[str, float]) -> Dict[str, Any]:
    """
    Maximum fixed load and maximum *screen* mass at the current assumptions.

    M0 = m_fix + P_aux t_f / e_b, so the screen budget is what remains of
    M0_max after the auxiliary battery and the non-screen fixed mass.
    """
    c = coefficients(p, area_p=0.0)
    if c.alpha <= 0.0 or c.beta <= 0.0:
        return dict(feasible=False, M0_max=0.0, m_star=None,
                    reason="alpha <= 0" if c.alpha <= 0 else "beta <= 0")
    m_star = 4.0 * c.alpha ** 2 / (9.0 * c.beta ** 2)
    M0_max = 4.0 * c.alpha ** 3 / (27.0 * c.beta ** 2)
    non_screen_fix = c.m_fix - float(p["m_screen"])
    m_screen_max = M0_max - c.m_bat_aux - non_screen_fix
    return dict(
        feasible=M0_max > c.M0, M0=c.M0, M0_max=M0_max, m_star=m_star,
        m_screen_max=m_screen_max, m_bat_aux=c.m_bat_aux,
        non_screen_fix=non_screen_fix, alpha=c.alpha, beta=c.beta,
        utilisation=c.M0 / M0_max if M0_max > 0 else float("inf"),
    )


LEVERS = ["t_f", "e_b_wh_kg", "FM", "eta", "D_rotor", "N_rotors", "rho",
          "S_m", "f_s", "lam", "g", "dod"]


def sensitivities(p: Dict[str, float], levers: Optional[List[str]] = None,
                  rel: float = 1e-4) -> Dict[str, float]:
    """
    Logarithmic sensitivity d ln(M0_max) / d ln(x) for each lever.

    The analytic expectations are -2 for endurance, +2 for e_b, FM and eta,
    +1 for disk area (so +2 for diameter) and +1 for rho.  Anything that
    only touches alpha shows up with the cubic alpha dependence.
    """
    levers = levers or LEVERS
    base = payload_wall(p)
    if not base.get("M0_max"):
        return {}
    out: Dict[str, float] = {}
    for k in levers:
        x = float(p[k])
        if x == 0.0:
            continue
        hi = dict(p); hi[k] = x * (1.0 + rel)
        lo = dict(p); lo[k] = x * (1.0 - rel)
        Whi = payload_wall(hi).get("M0_max")
        Wlo = payload_wall(lo).get("M0_max")
        if not Whi or not Wlo or Whi <= 0 or Wlo <= 0:
            continue
        dlnx = math.log(1.0 + rel) - math.log(1.0 - rel)
        out[k] = (math.log(Whi) - math.log(Wlo)) / dlnx
    return out


# ---------------------------------------------------------------------------
# The naive fixed-point iteration: converge or run away
# ---------------------------------------------------------------------------
def fixed_point_trace(p: Dict[str, float], m0: Optional[float] = None,
                      iters: int = 150, model: str = "fixed_area",
                      relax: float = 1.0) -> Dict[str, Any]:
    """
    The iteration described in the derivation:

        m_{k+1} = m_fix + f_s m_k + gamma_m m_k + m_bat(m_k)

    Converging to a finite value is the design existing.  Running away is the
    mass spiral, discovered computationally rather than assumed.
    """
    c = coefficients(p, area_p=(1.0 if model == "constant_dl" else float(p["area_p"])
                                if model == "power_law" else 0.0))
    g = float(p["g"])
    if model == "constant_dl":
        DL = float(p["disk_loading"])
        A_of_m = lambda m: max(m, 1e-9) * g / DL
    elif model == "power_law":
        pp = float(p["area_p"])
        A_of_m = lambda m: c.A * (max(m, 1e-9) / c.anchor_mass) ** pp
    else:
        A_of_m = lambda m: c.A

    m = float(m0) if m0 is not None else c.m_fix
    hist = [m]
    diverged = False
    for _ in range(iters):
        A = A_of_m(m)
        P = (m * g) ** 1.5 / (float(p["FM"]) * float(p["eta"]) *
                              math.sqrt(2.0 * float(p["rho"]) * A)) + c.P_aux
        m_bat = P * float(p["t_f"]) / c.e_b
        m_next = c.m_fix + c.f_s * m + c.gamma_m * m + m_bat
        m = m + relax * (m_next - m)
        if not math.isfinite(m) or m > 1e7:
            diverged = True
            hist.append(float("inf"))
            break
        hist.append(m)
        if len(hist) > 2 and abs(hist[-1] - hist[-2]) < 1e-10 * max(1.0, m):
            break
    converged = (not diverged) and len(hist) > 2 and \
        abs(hist[-1] - hist[-2]) < 1e-6 * max(1.0, hist[-1])

    # Contraction factor of the map, estimated from the last two increments.
    rate = None
    if len(hist) > 3 and all(math.isfinite(h) for h in hist[-4:]):
        d1 = hist[-2] - hist[-3]
        d2 = hist[-1] - hist[-2]
        if abs(d1) > 1e-14:
            rate = abs(d2 / d1)
    if diverged:
        status = "diverging"
    elif converged:
        status = "converged"
    elif rate is not None and rate >= 1.0:
        status = "diverging"
    else:
        # Still creeping.  Near the fold the map is only marginally
        # contracting, so the mass spiral takes many iterations to reveal
        # itself; the analytic closure is the authority on feasibility.
        status = "slow"
    return dict(history=hist, converged=bool(converged), diverged=bool(diverged),
                status=status, rate=rate,
                m_final=hist[-1] if not diverged else None, model=model,
                m_start=hist[0], iterations=len(hist) - 1)
