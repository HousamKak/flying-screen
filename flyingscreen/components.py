"""
Layer 3: first-principles component sizing.

The lumped closure needs two hand-waved constants, the structural fraction
f_s and the propulsion specific thrust S_m.  This module derives both from
geometry and materials instead:

  * rotor       momentum theory plus a profile-power term, which *produces*
                the figure of merit rather than assuming it, together with
                k_T, k_Q, k_P, RPM, blade loading and an acoustic estimate
  * arm         thin-walled tube sized by bending stress, tip deflection and
                first bending frequency
  * motor       mass from the torque it must deliver at maximum thrust
  * inertia     J from motor and prop point masses on the arms, plus the
                central body and the screen slab

Everything here is a function of gross mass, so it plugs straight into the
mass closure as a replacement for f_s m and (lam g / S_m) m.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict, field
from typing import Any, Dict, List, Optional

from .params import derived

# Acoustic anchor, calibrated rather than guessed.
#
# EU 2019/945 makes manufacturers declare a sound *power* level L_WA, which
# converts to pressure at 1 m over a reflecting plane as L_p = L_WA - 8.0 dB.
# Running that back through the broadband exponents (60 log V_tip, 10 log T,
# 10 log N) for the DJI Mini 3, Air 2S, Mavic 3 and Mavic 2 Pro gives an
# anchor of 82.3 to 83.8 dB, mean 82.9, spread 1.4 dB across a 3.6x range of
# aircraft mass.  Still an empirical scaling that ranks designs rather than
# certifying them, but now tied to measured hardware.
_SPL_REF = 82.9
_VTIP_REF = 100.0
_T_REF = 5.0

# Blade profile drag rises as Reynolds number falls, and a design pushed to
# very low tip speed for quietness lands at Re well under 100k, where a
# constant Cd0 would be a free lunch that does not exist.
_RE_REF = 2.0e5
_NU_AIR = 1.5e-5            # kinematic viscosity, m^2/s
_CD0_RE_EXP = 0.20
_CD0_RE_CAP = 3.0


def profile_drag_coefficient(p: Dict[str, float], chord: float,
                             v_tip: float) -> float:
    """Cd0 corrected for the Reynolds number the blade actually sees."""
    Re = max(0.7 * v_tip * chord / _NU_AIR, 1.0)    # 0.7R is the usual station
    factor = min((_RE_REF / Re) ** _CD0_RE_EXP, _CD0_RE_CAP)
    return float(p["Cd0"]) * max(factor, 1.0)


# ---------------------------------------------------------------------------
# Corrections the plain actuator disk does not contain
# ---------------------------------------------------------------------------
def screen_sign(p: Dict[str, float]) -> float:
    """+1 when the screen sits above the rotor plane, -1 when it hangs below."""
    return 1.0 if float(p.get("screen_above", 0)) >= 0.5 else -1.0


def rotor_plane_height(p: Dict[str, float]) -> float:
    """
    Height of the rotor disks above the floor.

    The display is held at eye level either way; what moves is the rotor
    plane, and with it how close the blades come to the user's head and how
    much ground or ceiling effect the rotors see.
    """
    return float(p["standoff_z"]) - screen_sign(p) * float(p["r_cp"])


def geometry_relative_to_user(p: Dict[str, float]) -> Dict[str, Any]:
    """
    Where the spinning parts are, relative to the person reading the screen.

    This is the part of the design a purely aerodynamic model never sees: the
    screen is vertical and the rotor flow is vertical, so putting the panel
    above or below the disks costs almost nothing in power. What it changes
    is whether the blades are at head height, and whether the display sits
    between them and a face.
    """
    R = float(p["D_rotor"]) / 2.0
    N = float(p["N_rotors"])
    L = float(p["rotor_gap"]) * R / math.sin(math.pi / N)
    z_rotor = rotor_plane_height(p)
    z_screen = float(p["standoff_z"])
    eye = float(p["standoff_z"])            # the display is held at eye level
    standoff = float(p["standoff"])
    boom = float(p.get("screen_forward", 0.0))

    # The standoff is measured to the *screen*. The rotor centroid sits a boom
    # length further away, and the nearest blade tip reaches L + R back toward
    # the user from there. Measuring safety to the vehicle centre, as a naive
    # model does, flatters this by the whole reach of the machine.
    centroid_dist = standoff + boom
    horiz = centroid_dist - (L + R)
    dz = z_rotor - eye
    overhangs = horiz < 0.0
    tip_to_eye = math.hypot(max(horiz, 0.0), dz) if not overhangs else abs(dz)

    d_safe = float(p["d_safe"])
    # Boom length that would put the nearest tip at the safe distance.
    need = math.sqrt(max(d_safe ** 2 - dz ** 2, 0.0)) if abs(dz) < d_safe else 0.0
    boom_for_safety = max((L + R) + need - standoff, 0.0)
    # Standoff that would do the same with no boom at all.
    standoff_for_safety = (L + R) + need

    return dict(
        z_rotor=z_rotor, z_screen=z_screen, arm_L=L, boom=boom,
        screen_above=screen_sign(p) > 0,
        blade_height_vs_eye=dz,
        tip_to_eye=tip_to_eye,
        horizontal_tip_gap=horiz,
        overhangs_user=bool(overhangs),
        reach=L + R,
        centroid_distance=centroid_dist,
        screen_shields=bool(screen_sign(p) > 0),
        desk_clearance=z_rotor - 0.75,      # a desk is about 0.75 m
        d_safe=d_safe,
        safe=bool(tip_to_eye >= d_safe),
        boom_for_safety=boom_for_safety,
        standoff_for_safety=standoff_for_safety,
    )


def boom_structure(p: Dict[str, float], m_tip: float) -> Dict[str, float]:
    """
    The cantilever that carries the display forward of the rotors.

    Same thin-walled tube treatment as the arms, but loaded by the display
    mass under the manoeuvre limit rather than by rotor thrust, and sized by
    stiffness as much as strength because a wobbling screen is unreadable.
    """
    Lb = float(p.get("screen_forward", 0.0))
    if Lb <= 1e-6:
        return dict(L=0.0, mass=0.0, wall=0.0, f_bending=float("inf"),
                    tip_deflection=0.0)
    r = float(p["arm_radius"]) * 0.9
    sig = float(p["sigma_allow"]) * 1e6
    E = float(p["E_mod"]) * 1e9
    rho_m = float(p["rho_mat"])
    SF = float(p["SF_struct"])

    F = SF * m_tip * float(p["g"]) * float(p["lam"])
    M = F * Lb
    wall_stress = M / (math.pi * r * r * sig)
    I_stiff = F * Lb ** 3 / (3.0 * E * (Lb / 200.0))     # tip droop under L/200
    wall_stiff = I_stiff / (math.pi * r ** 3)
    wall = max(wall_stress, wall_stiff, float(p["wall_min"]))
    I = math.pi * r ** 3 * wall
    mass = rho_m * 2.0 * math.pi * r * wall * Lb * 1.25   # plus end fittings
    mu = rho_m * 2.0 * math.pi * r * wall + 3.0 * m_tip / Lb
    f = (1.875 ** 2 / (2.0 * math.pi)) * math.sqrt(E * I / max(mu * Lb ** 4, 1e-30))
    return dict(L=Lb, mass=mass, wall=wall, f_bending=f,
                tip_deflection=F * Lb ** 3 / (3.0 * E * I))


def surface_effects(p: Dict[str, float]) -> Dict[str, float]:
    """
    Ground and ceiling effect on induced power.

    Cheeseman and Bennett give the classic in-ground-effect result: with the
    wake constrained by a surface a distance z below the rotor,

        T_IGE / T_OGE |_P  =  1 / (1 - (R / 4z)^2)

    which at constant thrust is a reduction in induced power by the same
    factor.  A ceiling above the rotor restricts the *inflow* instead of the
    wake and is stronger at the same spacing, so the same functional form is
    used with its own coefficient.  That coefficient is the least certain
    number in this file; published values scatter badly.

    Both effects vanish quickly with distance, going as the square of
    (R / 4z), so for a small rotor at head height in an ordinary room they
    are worth a few percent, not a few tens of percent.  Saying so
    quantitatively is the point.
    """
    R = float(p["D_rotor"]) / 2.0
    if float(p.get("ige_enable", 0)) < 0.5:
        return dict(k_ground=1.0, k_ceiling=1.0, k_surface=1.0,
                    z_floor=float("nan"), z_ceiling=float("nan"),
                    enabled=0.0)

    # The rotor plane sits above the screen centre by the screen offset.
    z_rotor = rotor_plane_height(p)
    z_floor = max(z_rotor, 0.25 * R)
    z_ceiling = max(float(p["ceiling_height"]) - z_rotor, 0.25 * R)

    def factor(z: float, k: float) -> float:
        # Guard the singularity at z -> R/4: below about z = R the model is
        # outside its validity anyway, so clamp rather than diverge.
        x = min((R / (4.0 * z)) ** 2 * k, 0.45)
        return 1.0 / (1.0 - x)

    k_g = factor(z_floor, 1.0)
    k_c = factor(z_ceiling, float(p["ceiling_k"]))
    # Both raise thrust for the same power, so induced power falls by the
    # combined factor.  They are treated as independent, which is optimistic
    # when the machine is boxed in between the two.
    return dict(k_ground=k_g, k_ceiling=k_c, k_surface=k_g * k_c,
                z_floor=z_floor, z_ceiling=z_ceiling, enabled=1.0)


def interference_factor(p: Dict[str, float]) -> Dict[str, float]:
    """
    Induced power penalty from adjacent rotors working in each other's flow.

    Side-by-side rotors lose efficiency as the tip-to-tip gap closes.  The
    penalty is negligible past about a quarter diameter of clearance and
    reaches a few percent when the disks nearly touch; anchored at 1.00 at
    s/D = 0.25 and 1.07 at s/D = 0, interpolated on the square so it decays
    the way an induced-flow interaction should.
    """
    if float(p.get("interference_enable", 0)) < 0.5:
        return dict(kappa_int=1.0, gap_ratio=float("nan"), enabled=0.0)
    N = float(p["N_rotors"])
    R = float(p["D_rotor"]) / 2.0
    L = float(p["rotor_gap"]) * R / math.sin(math.pi / N)
    centre_spacing = 2.0 * L * math.sin(math.pi / N)     # adjacent hubs
    gap = centre_spacing - 2.0 * R                       # tip to tip
    s_over_D = gap / float(p["D_rotor"])
    if s_over_D >= 0.25:
        k = 1.0
    else:
        f = max(0.0, 1.0 - s_over_D / 0.25)
        k = 1.0 + 0.07 * f * f
    return dict(kappa_int=k, gap_ratio=s_over_D, gap=gap,
                centre_spacing=centre_spacing, enabled=1.0)


def recirculation(p: Dict[str, float], A_total: float,
                  v_induced: float) -> Dict[str, float]:
    """
    How fast the machine turns the room's air over.

    Not a correction, a diagnostic.  The rotors move rho A v_i of air per
    second; divided into the room volume that gives the time to circulate
    every cubic metre in the room once.  When that time is short the machine
    is flying in air it has already been through, and the momentum theory
    assumption of still, undisturbed inflow has quietly stopped being true.
    """
    vol_flow = A_total * v_induced
    volume = float(p["room_length"]) * float(p["room_width"]) * float(p["ceiling_height"])
    t_exchange = volume / max(vol_flow, 1e-9)
    return dict(volume_flow=vol_flow, room_volume=volume,
                exchange_time=t_exchange,
                mean_room_velocity=vol_flow / max(
                    float(p["room_length"]) * float(p["room_width"]), 1e-9))


@dataclass
class RotorPoint:
    T: float                 # thrust per rotor                 [N]
    R: float                 # radius                           [m]
    A_rotor: float           # disk area                        [m^2]
    omega: float             # angular speed                    [rad/s]
    rpm: float
    v_tip: float             # tip speed                        [m/s]
    v_induced: float         # induced velocity at the disk     [m/s]
    v_wake: float            # far wake velocity, 2 v_i         [m/s]
    sigma: float             # solidity                         [-]
    Ct: float                # thrust coefficient               [-]
    Ct_sigma: float          # blade loading                    [-]
    P_ideal: float           # T v_i                            [W]
    P_induced: float         # kappa T v_i                      [W]
    P_profile: float         # blade drag power                 [W]
    P_shaft: float           # total mechanical power           [W]
    P_elec: float            # after electrical efficiency      [W]
    FM_effective: float      # P_ideal / P_shaft                [-]
    Q: float                 # shaft torque                     [N.m]
    k_T: float               # T = k_T omega^2                  [N.s^2]
    k_Q: float               # Q = k_Q omega^2                  [N.m.s^2]
    k_P: float               # P = k_P omega^3                  [W.s^3]
    stalled: bool
    spl_1m: float            # indicative A-weighted SPL at 1 m [dB]
    chord: float             # blade chord, R / aspect ratio     [m]
    reynolds: float          # at the 0.7 radius station         [-]
    cd0_effective: float     # after the Reynolds correction     [-]
    kappa_total: float       # induced power factor actually used[-]
    k_surface: float         # ground and ceiling relief          [-]
    kappa_int: float         # rotor-to-rotor interference        [-]


def rotor_point(p: Dict[str, float], T: float, v_axial: float = 0.0,
                omega: Optional[float] = None) -> RotorPoint:
    """
    Performance of one rotor producing thrust T.

    Momentum theory with axial inflow v_axial (positive climb):

        v_i = -v_axial/2 + sqrt((v_axial/2)^2 + T / (2 rho A))

    Profile power uses the standard blade-element result

        P_profile = (1/8) rho A (omega R)^3 sigma Cd0
    """
    rho = float(p["rho"])
    R = float(p["D_rotor"]) / 2.0
    A = math.pi * R * R
    T = max(T, 1e-9)
    n_b = float(p["n_blades"])
    sigma = n_b / (math.pi * float(p["blade_AR"]))
    if omega is None:
        if float(p.get("tip_speed_auto", 0)) >= 0.5:
            # Hold the blade at its design loading, Ct/sigma = T / (rho A V^2 sigma).
            # This is how a rotor is actually sized, and it is what makes a
            # larger disk genuinely cheaper rather than merely slower.
            ctsig = max(float(p["ct_sigma_design"]), 1e-4)
            v_tip = math.sqrt(T / (rho * A * sigma * ctsig))
            v_tip = min(max(v_tip, 20.0), 280.0)
        else:
            v_tip = float(p["tip_speed"])
        omega = v_tip / R
    v_tip = omega * R

    half = 0.5 * v_axial
    v_i = -half + math.sqrt(half * half + T / (2.0 * rho * A))
    P_ideal = T * (v_i + v_axial)

    # Induced power carries three corrections beyond the ideal disk: the
    # non-uniform inflow penalty, the interference from neighbouring rotors,
    # and the relief from nearby surfaces.
    surf = surface_effects(p)
    inter = interference_factor(p)
    kappa_total = (float(p["kappa_ind"]) * inter["kappa_int"]
                   / max(surf["k_surface"], 1e-6))
    P_ind = kappa_total * P_ideal

    chord = R / float(p["blade_AR"])
    cd0 = profile_drag_coefficient(p, chord, v_tip)
    P_prof = 0.125 * rho * A * (v_tip ** 3) * sigma * cd0
    P_shaft = P_ind + P_prof
    P_elec = P_shaft / float(p["eta"])

    Ct = T / (rho * A * v_tip * v_tip) if v_tip > 0 else 0.0
    Ct_sigma = Ct / sigma if sigma > 0 else 0.0
    Q = P_shaft / omega if omega > 0 else 0.0

    spl = (_SPL_REF + 60.0 * math.log10(max(v_tip, 1.0) / _VTIP_REF)
           + 10.0 * math.log10(max(T, 1e-3) / _T_REF)
           + 10.0 * math.log10(max(float(p["N_rotors"]), 1.0)))

    return RotorPoint(
        T=T, R=R, A_rotor=A, omega=omega, rpm=omega * 60.0 / (2.0 * math.pi),
        v_tip=v_tip, v_induced=v_i, v_wake=2.0 * v_i, sigma=sigma, Ct=Ct,
        Ct_sigma=Ct_sigma, P_ideal=T * v_i, P_induced=P_ind, P_profile=P_prof,
        P_shaft=P_shaft, P_elec=P_elec,
        FM_effective=(T * v_i) / P_shaft if P_shaft > 0 else 0.0,
        Q=Q, k_T=T / omega ** 2 if omega > 0 else 0.0,
        k_Q=Q / omega ** 2 if omega > 0 else 0.0,
        k_P=P_shaft / omega ** 3 if omega > 0 else 0.0,
        stalled=Ct_sigma > float(p["ct_sigma_max"]), spl_1m=spl,
        chord=chord, reynolds=0.7 * v_tip * chord / _NU_AIR,
        cd0_effective=cd0, kappa_total=kappa_total,
        k_surface=surf["k_surface"], kappa_int=inter["kappa_int"],
    )


@dataclass
class ArmResult:
    L: float                 # arm length, hub to rotor centre  [m]
    wall: float              # wall thickness chosen            [m]
    wall_stress: float
    wall_stiff: float
    driver: str              # which requirement set the wall
    I: float                 # second moment of area            [m^4]
    sigma: float             # working stress at limit load     [Pa]
    tip_deflection: float    # under limit load                 [m]
    f_bending: float         # first bending frequency          [Hz]
    m_arm_each: float
    m_arms: float
    m_guards: float          # prop guards, scaled by disk area [kg]
    m_boom: float            # screen cantilever, if any        [kg]
    m_frame: float           # arms, hub, guards, boom, fixed   [kg]
    span: float              # tip to tip                       [m]


def arm_structure(p: Dict[str, float], m: float,
                  m_tip: float = 0.0) -> ArmResult:
    """
    Each arm is a thin-walled tube of radius r carrying the rotor at its end.

    Bending stress    sigma = M r / I,   I = pi r^3 t   for t << r
    Tip deflection    delta = F L^3 / (3 E I)
    First bending     f1 = (1.875^2 / 2 pi) sqrt(E I / (mu L^4))
    """
    N = float(p["N_rotors"])
    g = float(p["g"])
    R = float(p["D_rotor"]) / 2.0
    r = float(p["arm_radius"])
    sig_allow = float(p["sigma_allow"]) * 1e6
    E = float(p["E_mod"]) * 1e9
    rho_m = float(p["rho_mat"])
    SF = float(p["SF_struct"])

    # Arm length so adjacent disks clear each other by rotor_gap.
    L = float(p["rotor_gap"]) * R / math.sin(math.pi / N)

    F_limit = SF * float(p["lam"]) * m * g / N     # limit load at one rotor
    M_bend = F_limit * L

    wall_stress = M_bend / (math.pi * r * r * sig_allow)
    I_stiff = F_limit * L ** 3 / (3.0 * E * (L / 100.0))   # delta <= L/100
    wall_stiff = I_stiff / (math.pi * r ** 3)
    wall_min = float(p["wall_min"])
    wall = max(wall_stress, wall_stiff, wall_min)
    driver = ("stress" if wall == wall_stress else
              "stiffness" if wall == wall_stiff else "minimum gauge")

    I = math.pi * r ** 3 * wall
    m_arm_each = rho_m * 2.0 * math.pi * r * wall * L
    m_arms = N * m_arm_each
    # Guards are rings around each disk, so their mass follows circumference.
    m_guards = float(p["guard_mass_k"]) * N * 2.0 * math.pi * R
    m_boom = boom_structure(p, float(p["m_screen"]) + float(p["m_gimbal"]))["mass"]
    m_frame = (m_arms * (1.0 + float(p["hub_frac"])) + m_guards + m_boom
               + float(p["m_frame_fixed"]))

    mu = rho_m * 2.0 * math.pi * r * wall                  # mass per length
    # Tip mass lowers the bending frequency; Rayleigh correction.
    mu_eff = mu + 3.0 * m_tip / L if L > 0 else mu
    f_bend = (1.875 ** 2 / (2.0 * math.pi)) * math.sqrt(E * I / max(mu_eff * L ** 4, 1e-30))

    return ArmResult(
        L=L, wall=wall, wall_stress=wall_stress, wall_stiff=wall_stiff,
        driver=driver, I=I, sigma=M_bend * r / I,
        tip_deflection=F_limit * L ** 3 / (3.0 * E * I), f_bending=f_bend,
        m_arm_each=m_arm_each, m_arms=m_arms, m_guards=m_guards,
        m_boom=m_boom, m_frame=m_frame, span=2.0 * (L + R),
    )


def optimal_tip_speed(p: Dict[str, float], T: float,
                      lo: float = 8.0, hi: float = 260.0) -> Dict[str, float]:
    """
    Tip speed that maximises the figure of merit at thrust T.

    Induced power falls with tip speed only through the profile trade: the
    ideal induced power does not depend on RPM at all, while profile power
    grows as V_tip^3.  So the optimum is pressed down against the blade
    stall limit Ct/sigma, and this search respects that limit.
    """
    q = dict(p)
    q["tip_speed_auto"] = 0          # the search sets tip speed itself

    def at(v: float) -> RotorPoint:
        q["tip_speed"] = v
        return rotor_point(q, T)

    n = 120
    grid = [lo + (hi - lo) * i / n for i in range(n + 1)]
    pts = [at(v) for v in grid]
    ok = [i for i, rp in enumerate(pts) if not rp.stalled]
    if not ok:
        return dict(v_tip=float(p["tip_speed"]), feasible=0.0)
    i = min(ok, key=lambda j: pts[j].P_shaft)
    # Refine. Two cases: an interior minimum, bracketed by its neighbours and
    # found by golden section; or the minimum pressed against the stall
    # boundary, found by bisecting on the stall flag.
    if i - 1 >= 0 and pts[i - 1].stalled:
        a, b = grid[i - 1], grid[i]
        for _ in range(60):
            mid = 0.5 * (a + b)
            if at(mid).stalled:
                a = mid
            else:
                b = mid
        v_best = b
    else:
        a = grid[max(i - 1, 0)]
        b = grid[min(i + 1, n)]
        g = (math.sqrt(5.0) - 1.0) / 2.0
        c, d_ = b - g * (b - a), a + g * (b - a)
        fc, fd = at(c).P_shaft, at(d_).P_shaft
        for _ in range(80):
            if fc < fd:
                b, d_, fd = d_, c, fc
                c = b - g * (b - a)
                fc = at(c).P_shaft
            else:
                a, c, fc = c, d_, fd
                d_ = a + g * (b - a)
                fd = at(d_).P_shaft
        v_best = 0.5 * (a + b)
    rp = at(v_best)
    if rp.stalled or rp.P_shaft > pts[i].P_shaft:
        v_best, rp = grid[i], pts[i]
    return dict(v_tip=v_best, P_shaft=rp.P_shaft, FM=rp.FM_effective, rpm=rp.rpm,
                Ct_sigma=rp.Ct_sigma, feasible=1.0)


@dataclass
class PropulsionResult:
    m_motor_each: float
    m_motors: float
    m_esc: float
    m_props: float
    m_prop_total: float      # motors + ESCs + props            [kg]
    S_m_implied: float       # T_max / m_prop_total             [N/kg]
    Q_max: float             # torque at maximum thrust         [N.m]
    omega_max: float
    P_shaft_max: float
    P_elec_max: float
    rotor_hover: RotorPoint
    rotor_max: RotorPoint


MOTOR_REF_MASS = 0.10          # kg, where the quoted torque density applies


def motor_mass_for_torque(p: Dict[str, float], Q: float) -> float:
    """
    Motor mass from the torque it must hold.

    Torque density is not constant across sizes: a 30 g outrunner manages
    about 2 N.m/kg while a 200 g one manages nearer 4, because cooling
    improves with scale.  Taking

        Q = k (m / m_ref)^e * m

    and solving for m gives a mild but real advantage to the large slow
    rotors that a machine hovering beside a person actually wants.
    """
    k = float(p["motor_torque_density"])
    e = float(p["motor_scale_exp"])
    Q = max(Q, 1e-12)
    return (Q * MOTOR_REF_MASS ** e / k) ** (1.0 / (1.0 + e))


def propulsion(p: Dict[str, float], m: float) -> PropulsionResult:
    """Motors sized by the torque needed at T_max = lam m g."""
    N = float(p["N_rotors"])
    g = float(p["g"])
    T_hover = m * g / N
    T_max = float(p["lam"]) * m * g / N

    hov = rotor_point(p, T_hover)
    # At maximum thrust the rotor spins up: T ~ omega^2 at fixed geometry.
    omega_max = hov.omega * math.sqrt(T_max / max(T_hover, 1e-9))
    mx = rotor_point(p, T_max, omega=omega_max)

    m_motor_each = motor_mass_for_torque(p, mx.Q)
    m_motors = N * m_motor_each
    m_esc = N * mx.P_elec / float(p["esc_power_density"])
    m_prop_each = float(p["prop_mass_k"]) * float(p["D_rotor"]) ** 2.6 * (float(p["n_blades"]) / 2.0)
    m_props = N * m_prop_each
    total = m_motors + m_esc + m_props
    return PropulsionResult(
        m_motor_each=m_motor_each, m_motors=m_motors, m_esc=m_esc,
        m_props=m_props, m_prop_total=total,
        S_m_implied=(float(p["lam"]) * m * g) / max(total, 1e-9),
        Q_max=mx.Q, omega_max=omega_max, P_shaft_max=N * mx.P_shaft,
        P_elec_max=N * mx.P_elec, rotor_hover=hov, rotor_max=mx,
    )


def inertia(p: Dict[str, float], m: float, arm: ArmResult,
            m_tip_each: float) -> Dict[str, float]:
    """
    Full inertia tensor about the centre of mass, from one shared geometry.

    Body axes: x forward (toward the user, the screen normal), y left, z up,
    origin at the hub in the rotor plane. Every part is placed where it is:

      motors, ESCs and props   point masses at the rotor hubs
      arms                     uniform rods from hub to rotor, m L^2 / 3
      guards                   thin rings of radius R centred on each rotor
      hub, battery, avionics   a central disc of radius L/4
      display and gimbal       a thin slab in the body y-z plane at
                               (screen_forward, 0, +-r_cp)
      boom                     a uniform bar along x from the hub to the slab

    The tensor is first accumulated about the hub and then moved to the
    centre of mass by the parallel-axis theorem, so a display that is both
    forward and above the rotor plane produces the x-z product of inertia it
    physically has. The gimbal is lumped with the slab; at 85 g against a
    280 g panel on the same pivot this is a small error.
    """
    import numpy as np
    N = int(round(float(p["N_rotors"])))
    L = arm.L
    R_g = float(p["D_rotor"]) / 2.0
    d = derived(p)
    m_screen = float(p["m_screen"]) + float(p["m_gimbal"])
    fwd = float(p.get("screen_forward", 0.0))
    bm = boom_structure(p, m_screen)["mass"] if fwd > 1e-6 else 0.0
    m_guard_each = arm.m_guards / N
    m_body = max(m - N * m_tip_each - m_screen - arm.m_arms - arm.m_guards - bm,
                 0.05 * m)

    J = np.zeros((3, 3))
    first = np.zeros(3)          # first moment about the hub, sum m_i r_i

    def point(mass: float, r: np.ndarray) -> None:
        nonlocal J, first
        J += mass * (float(r @ r) * np.eye(3) - np.outer(r, r))
        first += mass * r

    for i in range(N):
        psi = 2.0 * math.pi * i / N + math.pi / N     # same layout as dynamics
        r_i = np.array([L * math.cos(psi), L * math.sin(psi), 0.0])
        point(m_tip_each + arm.m_arm_each / 3.0 + m_guard_each, r_i)
        first += (arm.m_arm_each / 2.0 - arm.m_arm_each / 3.0) * r_i
        J += m_guard_each * R_g ** 2 * np.diag([0.5, 0.5, 1.0])   # ring about itself

    r_body = 0.25 * L
    J += m_body * r_body ** 2 * np.diag([0.25, 0.25, 0.5])

    # The display slab: panel of width w along y and height h along z.
    w, h = float(p["screen_w"]), float(p["screen_h"])
    z = screen_sign(p) * float(p["r_cp"])
    r_s = np.array([fwd, 0.0, z])
    J += m_screen * np.diag([w * w + h * h, h * h, w * w]) / 12.0
    point(m_screen, r_s)
    if bm > 0.0:
        # A uniform bar from the hub along x: J = m L^2 / 3 about the hub.
        J += bm * fwd * fwd / 3.0 * np.diag([0.0, 1.0, 1.0])
        first += bm * np.array([fwd / 2.0, 0.0, 0.0])

    center = first / m
    J -= m * (float(center @ center) * np.eye(3) - np.outer(center, center))
    return dict(Jxx=float(J[0, 0]), Jyy=float(J[1, 1]), Jzz=float(J[2, 2]),
                Jxz=float(J[0, 2]),
                tensor=J.tolist(), center_of_mass=center.tolist(),
                screen_offset=(r_s - center).tolist(),
                m_body=m_body, A_screen=d.A_screen)


@dataclass
class Breakdown:
    """A complete first-principles description at a given gross mass."""
    m: float
    m_fix: float
    m_frame: float
    m_prop: float
    m_bat: float
    m_sum: float
    residual: float
    f_s_implied: float
    S_m_implied: float
    FM_implied: float
    P_hover_elec: float
    P_aux: float
    P_total: float
    endurance: float
    disk_loading: float
    downwash: float
    spl_1m: float
    span: float
    J: Dict[str, float]
    arm: Dict[str, Any]
    propulsion: Dict[str, Any]
    rotor: Dict[str, Any]
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def breakdown(p: Dict[str, float], m: float, m_bat: Optional[float] = None) -> Breakdown:
    """Evaluate every first-principles submodel at a stated gross mass."""
    d = derived(p)
    N = float(p["N_rotors"])
    pr = propulsion(p, m)
    m_tip_each = pr.m_motor_each + pr.m_esc / N + pr.m_props / N
    arm = arm_structure(p, m, m_tip=m_tip_each)
    J = inertia(p, m, arm, m_tip_each)

    P_hover = N * pr.rotor_hover.P_elec
    P_total = P_hover + d.P_aux
    if m_bat is None:
        m_bat = max(m - d.m_fix - arm.m_frame - pr.m_prop_total, 0.0)
    endurance = m_bat * d.e_b * (1.0-float(p["soc_reserve"])) / max(P_total, 1e-9)

    m_sum = d.m_fix + arm.m_frame + pr.m_prop_total + m_bat
    w: List[str] = []
    if pr.rotor_hover.stalled:
        w.append("blade loading Ct/sigma = %.3f exceeds the stall limit at hover"
                 % pr.rotor_hover.Ct_sigma)
    if pr.rotor_max.stalled:
        w.append("blade loading Ct/sigma = %.3f exceeds the stall limit at T_max"
                 % pr.rotor_max.Ct_sigma)
    if pr.rotor_max.v_tip > 0.75 * 340.0:
        w.append("tip Mach %.2f at maximum thrust" % (pr.rotor_max.v_tip / 340.0))
    if arm.f_bending < 25.0:
        w.append("first arm bending mode at %.1f Hz sits inside the attitude loop bandwidth"
                 % arm.f_bending)
    if 2.0 * pr.rotor_hover.v_induced > 10.0:
        w.append("downwash of %.1f m/s in the wake is a gale at head height"
                 % (2.0 * pr.rotor_hover.v_induced))
    if m_bat / max(m, 1e-9) > 0.45:
        w.append("battery is %.0f percent of gross mass: the design is deep on the "
                 "steep part of the closure and very sensitive to every assumption"
                 % (100.0 * m_bat / m))
    if pr.rotor_hover.spl_1m > 70.0:
        w.append("about %.0f dB at 1 m, which is conversation level noise beside your head"
                 % pr.rotor_hover.spl_1m)
    if pr.rotor_hover.sigma > float(p["sigma_max"]):
        w.append("solidity %.2f is past the %.2f where blades stop acting independently: "
                 "the blade element model is flattering this rotor"
                 % (pr.rotor_hover.sigma, float(p["sigma_max"])))
    if pr.rotor_hover.reynolds < 40000:
        w.append("blade Reynolds number %.0f is below where a section keeps its lift curve"
                 % pr.rotor_hover.reynolds)

    return Breakdown(
        m=m, m_fix=d.m_fix, m_frame=arm.m_frame, m_prop=pr.m_prop_total,
        m_bat=m_bat, m_sum=m_sum, residual=m - m_sum,
        f_s_implied=arm.m_frame / max(m, 1e-9),
        S_m_implied=pr.S_m_implied,
        FM_implied=pr.rotor_hover.FM_effective,
        P_hover_elec=P_hover, P_aux=d.P_aux, P_total=P_total,
        endurance=endurance, disk_loading=m * float(p["g"]) / d.A,
        downwash=2.0 * pr.rotor_hover.v_induced,
        spl_1m=pr.rotor_hover.spl_1m, span=arm.span, J=J,
        arm=asdict(arm), propulsion={k: v for k, v in asdict(pr).items()
                                     if k not in ("rotor_hover", "rotor_max")},
        rotor=dict(hover=asdict(pr.rotor_hover), max=asdict(pr.rotor_max)),
        warnings=w,
    )


def required_battery(p: Dict[str, float], m: float) -> Dict[str, float]:
    """
    Battery mass at gross mass m, taking the larger of the two requirements:
    the energy the mission needs (inflated by the landing reserve) and the
    peak power the pack must actually deliver.
    """
    d = derived(p)
    N = float(p["N_rotors"])
    pr = propulsion(p, m)
    P_total = N * pr.rotor_hover.P_elec + d.P_aux
    reserve = min(max(float(p["soc_reserve"]), 0.0), 0.9)
    m_energy = P_total * float(p["t_f"]) / (d.e_b * (1.0 - reserve))
    m_power = (pr.P_elec_max + d.P_aux) / float(p["p_b_w_kg"])
    return dict(m_bat=max(m_energy, m_power), m_energy=m_energy,
                m_power=m_power, P_total=P_total,
                limit=1.0 if m_power > m_energy else 0.0)


def _dry_mass(p: Dict[str, float], m: float) -> Dict[str, float]:
    """Everything except the battery, at gross mass m. One pass, no waste."""
    d = derived(p)
    N = float(p["N_rotors"])
    pr = propulsion(p, m)
    arm = arm_structure(p, m, m_tip=pr.m_motor_each + (pr.m_esc + pr.m_props) / N)
    return dict(m_fix=d.m_fix, m_frame=arm.m_frame, m_prop=pr.m_prop_total,
                dry=d.m_fix + arm.m_frame + pr.m_prop_total)


def close_first_principles(p: Dict[str, float], m_guess: Optional[float] = None,
                           iters: int = 60) -> Dict[str, Any]:
    """
    Mass closure using the first-principles submodels instead of f_s and S_m.

    Solves   m = m_fix + m_frame(m) + m_prop(m) + m_bat(m)
    by bracketing the residual and bisecting.  The residual is negative for
    small m, positive on the closing branch, and negative again beyond the
    fold, so the first sign change is the light, physical root, and never
    finding one means the design does not exist.
    """
    def residual(m: float) -> float:
        return m - (_dry_mass(p, m)["dry"] + required_battery(p, m)["m_bat"])

    from .numerics import first_mass_root
    m, status = first_mass_root(residual)
    if m is None:
        return dict(feasible=False, status=status,
                    reason="No component mass root located in [0.001, 100000] kg; "
                           "bounded search failure is not a proof of infeasibility.")
    rb = required_battery(p, m)
    full = breakdown(p, m, m_bat=rb["m_bat"])
    return dict(feasible=True, m=m, breakdown=full.to_dict(),
                m_bat=rb["m_bat"], battery_limit="power" if rb["limit"] else "energy",
                residual=residual(m), status=status)
