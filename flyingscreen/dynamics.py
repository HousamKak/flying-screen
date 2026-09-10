"""
Layers 4 and 5: Newton-Euler rigid-body dynamics, rotor and motor states,
battery energy, and a two-axis screen gimbal.

State vector, length 13 + N + 1 + 4:

    0: 3     r        world position (z up)                 [m]
    3: 6     v        world velocity                        [m/s]
    6:10     q        quaternion, body to world, [w x y z]  [-]
   10:13     w        body angular velocity                 [rad/s]
   13:13+N   Om_i     rotor angular speeds                  [rad/s]
   13+N      E        remaining battery energy              [J]
   +1, +2    th_g     gimbal pitch and roll                 [rad]
   +3, +4    dth_g    gimbal rates                          [rad/s]

The equations implemented are exactly those in the derivation:

    rdot = v
    m vdot = T R e3 - m g e3 + F_drag + F_dist
    J wdot + w x J w = tau
    qdot = 0.5 Omega(w) q
    tau_m Omdot_i = Om_cmd_i - Om_i
    Edot = -(sum_i k_P Om_i^3 / eta + P_aux)
    J_g thddot_g + b_g thdot_g = tau_g - tau_wind
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .params import derived
from . import components as comp

E3 = np.array([0.0, 0.0, 1.0])


# ---------------------------------------------------------------------------
# Quaternion helpers (body -> world, scalar first)
# ---------------------------------------------------------------------------
def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                     (R[1, 2] + R[2, 1]) / s, 0.25 * s])


def quat_deriv(q: np.ndarray, w: np.ndarray) -> np.ndarray:
    """qdot = 0.5 * Omega(w) q with q = [w, x, y, z]."""
    qw, qx, qy, qz = q
    p, r, s = w
    return 0.5 * np.array([
        -qx * p - qy * r - qz * s,
        qw * p + qy * s - qz * r,
        qw * r - qx * s + qz * p,
        qw * s + qx * r - qy * p,
    ])


def hat(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def vee(M: np.ndarray) -> np.ndarray:
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def euler_from_rot(R: np.ndarray) -> Tuple[float, float, float]:
    """Roll, pitch, yaw (ZYX) for reporting only."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Vehicle
# ---------------------------------------------------------------------------
@dataclass
class Vehicle:
    """Everything the ODE needs, built once from a sized design."""
    m: float
    J: np.ndarray
    Jinv: np.ndarray
    N: int
    rotor_pos: np.ndarray        # (N,3) body positions of the rotor hubs
    spin: np.ndarray             # (N,) +-1
    k_T: float
    k_Q: float
    k_P: float
    omega_hover: float
    omega_max: float
    tau_motor: float
    E_max: float
    P_aux: float
    eta: float
    rho: float
    g: float
    A_screen: float
    Cd_n: float
    Cd_t: float
    screen_offset: np.ndarray    # body position of the screen centre
    A_frame: float               # equivalent flat plate area of the airframe
    Cd_frame: float
    J_gimbal: float
    b_gimbal: float
    tau_gimbal_max: float
    e_cp: float                  # aero centre offset from the gimbal axis  [m]
    arm_L: float
    reach: float                 # hub circle plus rotor radius: how far the
                                 # blades extend from the centre            [m]
    span: float
    alloc: np.ndarray            # (4,N)  maps omega_i^2 -> [T, tx, ty, tz]
    alloc_pinv: np.ndarray
    disk_loading: float
    breakdown: Dict[str, Any] = field(default_factory=dict)
    hub_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotor_radius: float = 0.0

    @property
    def n_states(self) -> int:
        return 13 + self.N + 1 + 4


def build_vehicle(p: Dict[str, float], m: float, m_bat: Optional[float] = None,
                  use_first_principles: bool = True) -> Vehicle:
    """
    Assemble a concrete vehicle at gross mass m.

    Rotor constants come from the blade-element/momentum rotor model so that
    k_T, k_Q and k_P are consistent with the diameter, tip speed and blade
    geometry rather than being free parameters.
    """
    d = derived(p)
    N = int(round(float(p["N_rotors"])))
    g = float(p["g"])
    pr = comp.propulsion(p, m)
    m_tip_each = pr.m_motor_each + (pr.m_esc + pr.m_props) / N
    arm = comp.arm_structure(p, m, m_tip=m_tip_each)
    inert = comp.inertia(p, m, arm, m_tip_each)

    if m_bat is None:
        m_bat = max(m - d.m_fix - arm.m_frame - pr.m_prop_total, 1e-3)

    hov = pr.rotor_hover
    k_T = hov.k_T
    k_Q = hov.k_Q
    k_P = hov.k_P
    if not use_first_principles:
        # Force the lumped figure of merit instead of the derived one.
        P_hover_shaft = (m * g) ** 1.5 / (float(p["FM"]) * math.sqrt(2.0 * float(p["rho"]) * d.A))
        k_P = P_hover_shaft / N / hov.omega ** 3

    L = arm.L
    offset = math.pi / N            # X layout for a quad
    psi = 2.0 * math.pi * np.arange(N) / N + offset
    rotor_pos = np.stack([L * np.cos(psi), L * np.sin(psi), np.zeros(N)], axis=1)
    center = np.asarray(inert["center_of_mass"])
    rotor_pos -= center
    spin = np.array([1.0 if i % 2 == 0 else -1.0 for i in range(N)])

    alloc = np.zeros((4, N))
    alloc[0, :] = k_T
    alloc[1, :] = rotor_pos[:, 1] * k_T          # roll   =  y_i T_i
    alloc[2, :] = -rotor_pos[:, 0] * k_T         # pitch  = -x_i T_i
    alloc[3, :] = -spin * k_Q                    # yaw from reaction torque
    alloc_pinv = np.linalg.pinv(alloc)

    J = np.asarray(inert["tensor"])
    omega_hover = hov.omega
    omega_max = pr.omega_max

    # Airframe parasitic area: tubes plus motors seen edge on.
    A_frame = N * (2.0 * float(p["arm_radius"]) * L) * 0.6

    m_screen = float(p["m_screen"]) + float(p["m_gimbal"])
    h = float(p["screen_h"])
    J_g = m_screen * (h * h + float(p["screen_w"]) ** 2) / 12.0

    return Vehicle(
        m=m, J=J, Jinv=np.linalg.inv(J), N=N, rotor_pos=rotor_pos, spin=spin,
        k_T=k_T, k_Q=k_Q, k_P=k_P, omega_hover=omega_hover, omega_max=omega_max,
        tau_motor=float(p["tau_motor"]), E_max=m_bat * d.e_b, P_aux=d.P_aux,
        eta=float(p["eta"]), rho=float(p["rho"]), g=g, A_screen=d.A_screen,
        Cd_n=float(p["Cd_screen_n"]), Cd_t=float(p["Cd_screen_t"]),
        # Positive z is up in the body frame, so a screen carried above the
        # rotor plane has a positive offset. The sign matters: it decides
        # which way screen drag pitches the machine.
        screen_offset=np.asarray(inert["screen_offset"]),
        hub_offset=-center, rotor_radius=float(p["D_rotor"])/2,
        A_frame=A_frame, Cd_frame=1.1,
        J_gimbal=max(J_g, 1e-5), b_gimbal=0.05 * max(J_g, 1e-5) ** 0.5,
        tau_gimbal_max=float(p["tau_gimbal_max"]),
        e_cp=0.05 * h, arm_L=L,
        reach=float(np.max(np.linalg.norm(rotor_pos, axis=1))) + float(p["D_rotor"]) / 2.0,
        span=arm.span,
        alloc=alloc, alloc_pinv=alloc_pinv,
        disk_loading=m * g / d.A,
        breakdown=dict(m_fix=d.m_fix, m_frame=arm.m_frame,
                       m_prop=pr.m_prop_total, m_bat=m_bat,
                       FM_effective=hov.FM_effective, arm_L=L,
                       f_bending=arm.f_bending, spl_1m=hov.spl_1m,
                       downwash=2.0 * hov.v_induced),
    )


# ---------------------------------------------------------------------------
# State packing
# ---------------------------------------------------------------------------
def initial_state(veh: Vehicle, r0: np.ndarray, yaw0: float = 0.0,
                  soc0: float = 1.0, v0: Optional[np.ndarray] = None,
                  R0: Optional[np.ndarray] = None,
                  w0: Optional[np.ndarray] = None,
                  T0: Optional[float] = None) -> np.ndarray:
    """
    Start the machine where the mission wants it, moving with it, and
    trimmed for it.

    Starting at rest beside a person already walking is not a gentler
    initial condition, it is a worse one: on a curved path the screen sits
    in their path, so they walk into it while it accelerates. Starting level
    when the reference already needs a tilt is the same mistake one level
    up: the attitude loop then spends the first second catching up, and the
    transient lands in every whole-window statistic. So the caller may pass
    the trim attitude R0, body rate w0 and thrust T0 for the reference at
    t = 0; the rotor speeds are set to produce that thrust with no torque.
    """
    x = np.zeros(veh.n_states)
    x[0:3] = r0
    if v0 is not None:
        x[3:6] = v0
    if R0 is not None:
        x[6:10] = rot_to_quat(np.asarray(R0, dtype=float))
    else:
        x[6] = math.cos(0.5 * yaw0)
        x[9] = math.sin(0.5 * yaw0)
    if w0 is not None:
        x[10:13] = np.asarray(w0, dtype=float)
    T = veh.m * veh.g if T0 is None else float(T0)
    u = veh.alloc_pinv @ np.array([T, 0.0, 0.0, 0.0])
    x[13:13 + veh.N] = np.sqrt(np.clip(u, 0.0, veh.omega_max ** 2))
    x[13 + veh.N] = soc0 * veh.E_max
    return x


def hover_speeds(veh: Vehicle) -> np.ndarray:
    """Rotor speeds that give thrust mg and zero torque about the centre of mass."""
    u = veh.alloc_pinv @ np.array([veh.m * veh.g, 0.0, 0.0, 0.0])
    return np.sqrt(np.clip(u, 0.0, veh.omega_max ** 2))


def unpack(veh: Vehicle, x: np.ndarray):
    N = veh.N
    return (x[0:3], x[3:6], x[6:10], x[10:13], x[13:13 + N],
            x[13 + N], x[14 + N:16 + N], x[16 + N:18 + N])


# ---------------------------------------------------------------------------
# Forces
# ---------------------------------------------------------------------------
def screen_axes(R: np.ndarray, th_g: np.ndarray):
    """
    Screen face normal and screen up direction, in world coordinates.

    The gimbal rotates the screen relative to the body by pitch th_g[0]
    about body y and roll th_g[1] about body x.  At rest the face normal is
    body +x and the top of the image is body +z, so the screen is a vertical
    plane spanning body y and z.

    Written out rather than built from two 3x3 matrices because this sits in
    the innermost loop of the integrator.  Note that gimbal roll leaves the
    normal alone and only turns the image in its own plane, which is why the
    readability metric has to look at the up vector and not just the normal.
    """
    cp, sp = math.cos(th_g[0]), math.sin(th_g[0])
    cr, sr = math.cos(th_g[1]), math.sin(th_g[1])
    n_b = np.array([cp, 0.0, -sp])
    u_b = np.array([sp * cr, -sr, cp * cr])
    return R @ n_b, R @ u_b


def screen_normal_world(R: np.ndarray, th_g: np.ndarray) -> np.ndarray:
    return screen_axes(R, th_g)[0]


def aero_forces(veh: Vehicle, R: np.ndarray, v_rel: np.ndarray,
                th_g: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Screen plus airframe drag.

    The screen is a flat plate: the component of the relative wind along its
    normal sees Cd_n over the full area, the in-plane component sees only
    Cd_t.  This is what couples attitude into translation, and via the
    offset of the screen from the centre of mass, translation back into
    attitude.
    """
    n_w = screen_normal_world(R, th_g)
    vn = float(np.dot(v_rel, n_w))
    v_t = v_rel - vn * n_w
    q = 0.5 * veh.rho * veh.A_screen
    F_n = -q * veh.Cd_n * abs(vn) * vn * n_w
    F_t = -q * veh.Cd_t * np.linalg.norm(v_t) * v_t
    F_screen = F_n + F_t

    sp = np.linalg.norm(v_rel)
    F_frame = -0.5 * veh.rho * veh.Cd_frame * veh.A_frame * sp * v_rel

    # Screen force acts at the gimbal pivot, offset from the centre of mass.
    r_w = R @ veh.screen_offset
    M_screen = np.cross(r_w, F_screen)              # world frame moment
    tau_body = R.T @ M_screen
    drag_normal = abs(q * veh.Cd_n * vn * vn)
    return F_screen + F_frame, tau_body, drag_normal


def rotor_forces(veh: Vehicle, Om: np.ndarray) -> Tuple[float, np.ndarray]:
    om2 = np.clip(Om, 0.0, None) ** 2
    wr = veh.alloc @ om2
    return float(wr[0]), wr[1:4]


def electrical_power(veh: Vehicle, Om: np.ndarray) -> float:
    """Hover-calibrated near-hover power approximation; no inflow correction."""
    return float(np.sum(veh.k_P * np.clip(Om, 0.0, None) ** 3) / veh.eta) + veh.P_aux


# ---------------------------------------------------------------------------
# The right hand side
# ---------------------------------------------------------------------------
def derivatives(veh: Vehicle, t: float, x: np.ndarray, Om_cmd: np.ndarray,
                tau_g_cmd: np.ndarray, v_air: np.ndarray,
                F_dist: np.ndarray) -> np.ndarray:
    r, v, q, w, Om, E, th_g, dth_g = unpack(veh, x)
    R = quat_to_rot(q)

    T, tau_rot = rotor_forces(veh, Om)
    v_rel = v - v_air
    F_aero, tau_aero, drag_n = aero_forces(veh, R, v_rel, th_g)

    F = T * (R @ E3) - veh.m * veh.g * E3 + F_aero + F_dist
    a = F / veh.m

    tau_g = np.clip(tau_g_cmd, -veh.tau_gimbal_max, veh.tau_gimbal_max)
    # Gimbal motor torque reacts on the airframe about body y and x.
    tau_react = np.array([-tau_g[1], -tau_g[0], 0.0])
    tau = tau_rot + tau_aero + tau_react
    wdot = veh.Jinv @ (tau - np.cross(w, veh.J @ w))

    dx = np.zeros_like(x)
    dx[0:3] = v
    dx[3:6] = a
    dx[6:10] = quat_deriv(q, w)
    dx[10:13] = wdot
    dx[13:13 + veh.N] = (np.clip(Om_cmd, 0.0, veh.omega_max) - Om) / veh.tau_motor

    P = electrical_power(veh, Om)
    dx[13 + veh.N] = -P

    # Gimbal: the aerodynamic moment about its own axes comes from the small
    # offset between the screen aero centre and the pivot.
    tau_wind = np.array([drag_n * veh.e_cp, 0.0])
    dth = (tau_g - tau_wind - veh.b_gimbal * dth_g) / veh.J_gimbal
    dx[14 + veh.N:16 + veh.N] = dth_g
    dx[16 + veh.N:18 + veh.N] = dth
    return dx


def normalize_quat(x: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(x[6:10])
    if n > 1e-9:
        x[6:10] /= n
    return x


# ---------------------------------------------------------------------------
# Hover linearisation
# ---------------------------------------------------------------------------
def hover_linearisation(veh: Vehicle) -> Dict[str, Any]:
    """
    Numerical A and B at hover, with the eigenvalues and the controllability
    rank of the 12 rigid-body states.  Confirms the classic structure: two
    double integrators in x and y driven only through pitch and roll.
    """
    N = veh.N
    n = 13 + N
    x0 = np.zeros(veh.n_states)
    x0[6] = 1.0
    # Trim, not equal speeds: when the centre of mass is off the rotor
    # centroid (a display on a boom), hover needs unequal rotor speeds.
    Om_trim = hover_speeds(veh)
    Om_h = float(np.mean(Om_trim))
    x0[13:13 + N] = Om_trim
    x0[13 + N] = veh.E_max
    zero3 = np.zeros(3)
    v_air = np.zeros(3)

    def f(xv: np.ndarray) -> np.ndarray:
        return derivatives(veh, 0.0, xv, Om_trim, np.zeros(2), v_air, zero3)[:n]

    A = np.zeros((n, n))
    for i in range(n):
        h = 1e-6 * max(1.0, abs(x0[i]))
        xp = x0.copy(); xp[i] += h
        xm = x0.copy(); xm[i] -= h
        A[:, i] = (f(xp) - f(xm)) / (2.0 * h)

    B = np.zeros((n, N))
    for i in range(N):
        h = 1e-4 * Om_h
        up = Om_trim.copy(); up[i] += h
        um = Om_trim.copy(); um[i] -= h
        fp = derivatives(veh, 0.0, x0, up, np.zeros(2), v_air, zero3)[:n]
        fm = derivatives(veh, 0.0, x0, um, np.zeros(2), v_air, zero3)[:n]
        B[:, i] = (fp - fm) / (2.0 * h)

    # Drop the quaternion scalar row/column, which is not a free state.
    keep = [i for i in range(n) if i != 6]
    Ar = A[np.ix_(keep, keep)]
    Br = B[keep, :]
    eig = np.linalg.eigvals(Ar)

    from .numerics import controllability_rank
    rank = controllability_rank(Ar, Br)

    return dict(
        A=Ar.tolist(), B=Br.tolist(),
        eigenvalues=[[float(z.real), float(z.imag)] for z in eig],
        controllable=rank == len(keep), rank=rank, n_states=len(keep),
        omega_hover=Om_h,
        note="States: r(3) v(3) qvec(3) w(3) Omega(N). Inputs: commanded rotor speeds. Energy and gimbal excluded.",
    )


def rotor_clearance(veh: Vehicle, r, R, point):
    """Minimum point-to-oriented-rotor-disk distance (no head radius)."""
    hubs = np.asarray(r) + veh.rotor_pos @ R.T
    delta = np.asarray(point) - hubs
    normal = R[:, 2]
    axial = delta @ normal
    radial = np.linalg.norm(delta - axial[:, None]*normal, axis=1)
    return float(np.min(np.hypot(axial, np.maximum(radial-veh.rotor_radius, 0))))
