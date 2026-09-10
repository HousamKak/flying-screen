"""
Layer 6: the cascaded controller.

    position -> velocity -> attitude -> body rate -> rotor speed

The outer loop asks for an acceleration, that becomes a force, the force
direction becomes the desired body z axis, and a geometric attitude
controller on SO(3) drives the body there.  The mixer then inverts the
allocation matrix to get rotor speeds.  A separate PD loop drives the
two-axis gimbal so the screen stays upright while the airframe tilts.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .dynamics import (E3, Vehicle, quat_to_rot, screen_axes,
                       euler_from_rot, vee)


@dataclass
class ControlOutput:
    Om_cmd: np.ndarray
    tau_g: np.ndarray
    T_cmd: float
    T_applied: float
    tau_cmd: np.ndarray
    saturated: bool
    tilt_cmd: float          # commanded body tilt from vertical  [rad]
    e_p: np.ndarray
    e_v: np.ndarray


class CascadedController:
    def __init__(self, p: Dict[str, float], veh: Vehicle):
        self.p = p
        self.veh = veh
        self.Kp = float(p["Kp_pos"])
        self.Kd = float(p["Kd_pos"])
        self.Ki = float(p["Ki_pos"])
        self.K_R = float(p["K_R"])
        self.K_w = float(p["K_w"])
        self.Kp_g = float(p["Kp_gimbal"])
        self.Kd_g = float(p["Kd_gimbal"])
        self.T_max = float(p["lam"]) * veh.m * float(p["g"])
        self.integral = np.zeros(3)
        self.i_limit = 2.0 * float(p["g"])

    def reset(self) -> None:
        self.integral = np.zeros(3)

    # -- outer loop ---------------------------------------------------------
    def desired_force(self, r: np.ndarray, v: np.ndarray, ref: Dict[str, np.ndarray],
                      dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        e_p = ref["r"] - r
        e_v = ref["v"] - v
        self.integral = np.clip(self.integral + e_p * dt,
                                -self.i_limit, self.i_limit)
        a_c = ref["a"] + self.Kp * e_p + self.Kd * e_v + self.Ki * self.integral
        F_c = self.veh.m * (a_c + self.veh.g * E3)
        return F_c, e_p, e_v

    # -- full step ----------------------------------------------------------
    def __call__(self, t: float, x: np.ndarray, ref: Dict[str, np.ndarray],
                 dt: float) -> ControlOutput:
        veh = self.veh
        N = veh.N
        r, v = x[0:3], x[3:6]
        q, w = x[6:10], x[10:13]
        th_g, dth_g = x[14 + N:16 + N], x[16 + N:18 + N]
        R = quat_to_rot(q)

        F_c, e_p, e_v = self.desired_force(r, v, ref, dt)
        T_cmd = float(np.linalg.norm(F_c))
        sat = T_cmd > self.T_max
        T_cmd = min(T_cmd, self.T_max)

        R_d, w_d, wd_dot = desired_attitude(veh, F_c, ref)
        b3 = R_d[:, 2]
        e_R = 0.5 * vee(R_d.T @ R - R.T @ R_d)
        RtRd = R.T @ R_d
        e_w = w - RtRd @ w_d
        tau = (veh.J @ (-self.K_R * e_R - self.K_w * e_w)
               + np.cross(w, veh.J @ w)
               - veh.J @ (np.cross(w, RtRd @ w_d) - RtRd @ wd_dot))

        # -- mixer ----------------------------------------------------------
        wrench = np.concatenate([[T_cmd], tau])
        om2 = veh.alloc_pinv @ wrench
        om2_max = veh.omega_max ** 2
        if np.any(om2 < 0.0) or np.any(om2 > om2_max):
            sat = True
            # Keep the torque, give up thrust: recentre about the achievable mean.
            for _ in range(6):
                om2 = np.clip(om2, 0.0, om2_max)
                achieved = veh.alloc @ om2
                err = wrench - achieved
                err[0] *= 0.5          # thrust yields first
                om2 = om2 + veh.alloc_pinv @ err
            om2 = np.clip(om2, 0.0, om2_max)
        Om_cmd = np.sqrt(np.clip(om2, 0.0, None))
        T_applied = float(veh.alloc[0] @ np.clip(om2, 0.0, None))

        # -- gimbal ---------------------------------------------------------
        roll, pitch, _ = euler_from_rot(R)
        th_cmd = np.array([-pitch, -roll])
        dth_cmd = np.array([-w[1], -w[0]])
        tau_g = veh.J_gimbal * (self.Kp_g * (th_cmd - th_g)
                                + self.Kd_g * (dth_cmd - dth_g))
        tau_g = np.clip(tau_g, -veh.tau_gimbal_max, veh.tau_gimbal_max)

        tilt_cmd = math.acos(max(-1.0, min(1.0, float(b3[2]))))
        return ControlOutput(Om_cmd=Om_cmd, tau_g=tau_g, T_cmd=T_cmd,
                             T_applied=T_applied, tau_cmd=tau, saturated=bool(sat),
                             tilt_cmd=tilt_cmd, e_p=e_p, e_v=e_v)


def desired_attitude(veh: Vehicle, F_c: np.ndarray, ref: Dict[str, Any]):
    """
    Desired rotation, body rate and (heading part of the) body angular
    acceleration for a commanded force and a reference with jerk and heading
    rate. Shared by the controller and by the simulator's initial trim.

    The rate follows from differential flatness (Mellinger and Kumar): the
    reference jerk rotates the thrust direction, and the reference heading
    rate turns the frame about b3. Without it the loop chases a rotating b3
    with zero rate feedforward and lags it by K_w |omega| / K_R, which on a
    curved path is several degrees.
    """
    if np.linalg.norm(F_c) < 1e-9:
        b3 = E3.copy()
    else:
        b3 = F_c / np.linalg.norm(F_c)
    yaw_d = float(ref.get("yaw", 0.0))
    b1_c = np.array([math.cos(yaw_d), math.sin(yaw_d), 0.0])
    b2 = np.cross(b3, b1_c)
    n2 = np.linalg.norm(b2)
    if n2 < 1e-6:
        b1_c = np.array([1.0, 0.0, 0.0])
        b2 = np.cross(b3, b1_c)
        n2 = np.linalg.norm(b2)
    b2 /= n2
    b1 = np.cross(b2, b3)
    R_d = np.column_stack([b1, b2, b3])

    j_ref = np.asarray(ref.get("j", np.zeros(3)), dtype=float)
    h_w = (veh.m / max(float(np.linalg.norm(F_c)), 1e-9)) * (j_ref - float(b3 @ j_ref) * b3)
    yaw_rate = float(ref.get("yaw_rate", 0.0))
    w_d = np.array([-float(h_w @ b2), float(h_w @ b1), yaw_rate * float(b3[2])])
    # Only the heading part of the desired angular acceleration is kept;
    # the tilt part would need the reference snap.
    wd_dot = np.array([0.0, 0.0, float(ref.get("yaw_acc", 0.0)) * float(b3[2])])
    return R_d, w_d, wd_dot


def control_authority(veh: Vehicle) -> Dict[str, float]:
    """
    How much angular acceleration the rotors can actually produce, holding
    total thrust at weight.

    This is the number that should set the attitude gains, and it is not the
    same for every layout: a hexacopter carries two rotors that contribute
    nothing to pitch, so despite having more motors it can have less pitch
    authority than a quad of the same span.
    """
    from scipy.optimize import linprog
    A = veh.alloc * veh.omega_hover**2
    moments = []
    limits = (veh.omega_max/veh.omega_hover)**2
    for axis in (1, 2, 3):
        rows = [i for i in range(4) if i != axis]
        b = [veh.m*veh.g if i == 0 else 0.0 for i in rows]
        both = []
        for sign in (-1, 1):
            fit = linprog(-sign*A[axis], A_eq=A[rows], b_eq=b,
                          bounds=[(0, limits)]*veh.N, method="highs")
            both.append(max(0.0, -float(fit.fun)) if fit.success else 0.0)
        moments.append(min(both))  # bidirectional authority at fixed hover thrust
    roll, pitch, yaw = moments
    # Angular acceleration about axis j for a pure torque about j is
    # (J^-1)_jj tau_j; with a product of inertia (display on a boom) this is
    # not tau_j / J_jj.
    Ji = np.asarray(veh.Jinv)
    a_roll, a_pitch, a_yaw = roll * Ji[0, 0], pitch * Ji[1, 1], yaw * Ji[2, 2]
    return dict(
        tau_roll=roll, tau_pitch=pitch, tau_yaw=yaw,
        alpha_roll=float(a_roll), alpha_pitch=float(a_pitch),
        alpha_yaw=float(a_yaw), alpha=float(min(a_roll, a_pitch)),
    )


def suggest_gains(veh: Vehicle, e_max_deg: float = 25.0,
                  a_max: float = 2.5, e_pos_max: float = 0.6,
                  zeta: float = 1.0) -> Dict[str, float]:
    """
    Gains sized by what the machine can do, rather than by habit.

    Attitude: the loop must not command more angular acceleration than the
    rotors can deliver at the largest attitude error it is expected to see,
    so K_R = alpha_available / e_max.  Rate gain follows from the damping.

    Position: the same argument one level up, K_p = a_max / e_pos_max, with
    the acceleration cap being the one the reference governor enforces.
    """
    auth = control_authority(veh)
    e_max = math.radians(e_max_deg)
    # Reserve half the single-axis torque and cap bandwidth relative to motor
    # lag. This is a tuning heuristic, not a coupled saturation certificate.
    K_R = min(0.5*auth["alpha"] / max(e_max, 1e-6),
              (0.3/max(veh.tau_motor, 1e-6))**2)
    K_w = 2.0 * zeta * math.sqrt(max(K_R, 1e-9))
    # The cascade rests on time-scale separation: the position loop must be
    # at least a factor SEPARATION slower than the attitude loop, or a
    # machine whose attitude authority has been cut (a display on a boom)
    # is destabilised by position gains that were fine without the boom.
    Kp = min(a_max / max(e_pos_max, 1e-6), K_R / SEPARATION ** 2)
    Kd = 2.0 * zeta * math.sqrt(max(Kp, 1e-9))
    return dict(K_R=K_R, K_w=K_w, Kp_pos=Kp, Kd_pos=Kd,
                Ki_pos=0.1 * Kp, **auth)


SEPARATION = 3.0     # attitude bandwidth over position bandwidth, at least


def screen_tilt_error(veh: Vehicle, x: np.ndarray) -> float:
    """
    How far the image is from upright, as the reader sees it.

    Taken as the angle between the top of the screen and world vertical, so
    it catches both the screen leaning away (gimbal pitch) and the image
    rolling in its own plane (gimbal roll).  Zero means perfectly upright,
    which is the whole reason the gimbal exists: the airframe has to tilt to
    accelerate and the display must not.
    """
    N = veh.N
    R = quat_to_rot(x[6:10])
    _, u_w = screen_axes(R, x[14 + N:16 + N])
    return math.acos(max(-1.0, min(1.0, float(u_w[2]))))


def lqr_gains(A: np.ndarray, B: np.ndarray, Q: np.ndarray,
              Rw: np.ndarray) -> Optional[np.ndarray]:
    """Optional LQR design on the hover linearisation."""
    try:
        from scipy.linalg import solve_continuous_are
    except Exception:
        return None
    try:
        P = solve_continuous_are(A, B, Q, Rw)
        return np.linalg.solve(Rw, B.T @ P)
    except Exception:
        return None
