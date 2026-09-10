"""
The mission: what the human does, and what the air does.

The human trajectory r_h(t) is the disturbance the vehicle must track.  The
desired screen position is r_d = r_h + d, an offset in front of the eyes,
and the desired yaw keeps the screen facing the person.

All trajectories are given analytically with their first and second
derivatives so the controller gets clean feedforward, which is what keeps
the tracking error honest rather than lag dominated.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

TWO_PI = 2.0 * math.pi


@dataclass
class HumanState:
    r: np.ndarray
    v: np.ndarray
    a: np.ndarray
    heading: float


class Human:
    """Base class: position, velocity and acceleration of the person."""

    name = "hover"

    def __init__(self, p: Dict[str, float]):
        self.p = p
        self.v_max = float(p["v_follow"])
        self.a_max = float(p["a_follow"])

    def __call__(self, t: float) -> HumanState:
        z = np.zeros(3)
        return HumanState(z.copy(), z.copy(), z.copy(), 0.0)


class Standing(Human):
    """A person standing still, with slow postural sway."""
    name = "standing"

    def __call__(self, t: float) -> HumanState:
        w = 2.0 * math.pi / 7.0
        A = 0.03
        r = np.array([A * math.sin(w * t), A * 0.6 * math.sin(0.77 * w * t), 0.0])
        v = np.array([A * w * math.cos(w * t),
                      A * 0.6 * 0.77 * w * math.cos(0.77 * w * t), 0.0])
        a = np.array([-A * w * w * math.sin(w * t),
                      -A * 0.6 * (0.77 * w) ** 2 * math.sin(0.77 * w * t), 0.0])
        return HumanState(r, v, a, 0.0)


class Pacing(Human):
    """
    Back and forth along x while keeping the screen in view, so the heading
    does not change.  This is reading while walking.
    """
    name = "pacing"
    turns = False

    def __init__(self, p: Dict[str, float]):
        super().__init__(p)
        # Peak speed = v_max, peak acceleration = a_max fixes both A and w.
        self.w = max(self.a_max / max(self.v_max, 1e-6), 1e-3)
        self.A = self.v_max / self.w

    def __call__(self, t: float) -> HumanState:
        A, w = self.A, self.w
        r = np.array([A * math.sin(w * t), 0.0, 0.0])
        v = np.array([A * w * math.cos(w * t), 0.0, 0.0])
        a = np.array([-A * w * w * math.sin(w * t), 0.0, 0.0])
        heading = 0.0
        if self.turns:
            # A real person turns over about a second rather than instantly,
            # so the heading is a smooth switch, not a sign test.  A
            # discontinuous heading would teleport the reference by twice the
            # standoff, which is a modelling artefact, not a control problem.
            k = 3.0 / max(self.v_max, 1e-6)
            heading = 0.5 * math.pi * (1.0 - math.tanh(k * v[0]))
        return HumanState(r, v, a, heading)


class TurnAround(Pacing):
    """
    The same pace, but the person turns around at each end.  The screen then
    has to swing through a half circle of radius equal to the standoff in
    about a second, which is by far the most demanding thing a following
    display is ever asked to do.
    """
    name = "turnaround"
    turns = True


class WalkLoop(Human):
    """A steady loop of the room, so the vehicle must also yaw around."""
    name = "walk_loop"

    def __init__(self, p: Dict[str, float]):
        super().__init__(p)
        self.R = max(self.v_max ** 2 / max(self.a_max, 1e-6), 0.5)
        self.w = self.v_max / self.R

    def __call__(self, t: float) -> HumanState:
        R, w = self.R, self.w
        c, s = math.cos(w * t), math.sin(w * t)
        r = np.array([R * c, R * s, 0.0])
        v = np.array([-R * w * s, R * w * c, 0.0])
        a = np.array([-R * w * w * c, -R * w * w * s, 0.0])
        return HumanState(r, v, a, math.atan2(v[1], v[0]))


class Jogging(Human):
    """Loop plus vertical bob: the case that stresses the tracking loop."""
    name = "jogging"

    def __init__(self, p: Dict[str, float]):
        super().__init__(p)
        self.base = WalkLoop(p)
        self.wb = TWO_PI * 1.5           # stride frequency
        self.Ab = 0.05

    def __call__(self, t: float) -> HumanState:
        h = self.base(t)
        r = h.r + np.array([0.0, 0.0, self.Ab * math.sin(self.wb * t)])
        v = h.v + np.array([0.0, 0.0, self.Ab * self.wb * math.cos(self.wb * t)])
        a = h.a + np.array([0.0, 0.0, -self.Ab * self.wb ** 2 * math.sin(self.wb * t)])
        return HumanState(r, v, a, h.heading)


class SitStand(Human):
    """Vertical steps: the screen has to follow eye height up and down."""
    name = "sit_stand"

    def __init__(self, p: Dict[str, float]):
        super().__init__(p)
        self.period = 40.0
        self.drop = 0.45
        self.tau = 1.2                    # smoothed step

    def __call__(self, t: float) -> HumanState:
        ph = (t % self.period) / self.period
        s = 0.5 * (1.0 + math.tanh((ph - 0.25) * self.period / self.tau)) \
            - 0.5 * (1.0 + math.tanh((ph - 0.75) * self.period / self.tau))
        # derivative of the tanh pair
        def dsech(u):
            return 1.0 / math.cosh(u) ** 2
        k = self.period / self.tau
        u1 = (ph - 0.25) * k
        u2 = (ph - 0.75) * k
        ds = 0.5 * k * (dsech(u1) - dsech(u2)) / self.period
        dds = 0.5 * k * k * (-2.0 * math.tanh(u1) * dsech(u1)
                             + 2.0 * math.tanh(u2) * dsech(u2)) / self.period ** 2
        r = np.array([0.0, 0.0, -self.drop * s])
        v = np.array([0.0, 0.0, -self.drop * ds])
        a = np.array([0.0, 0.0, -self.drop * dds])
        return HumanState(r, v, a, 0.0)


HUMANS: Dict[str, Any] = {
    "standing": Standing,
    "pacing": Pacing,
    "turnaround": TurnAround,
    "walk_loop": WalkLoop,
    "jogging": Jogging,
    "sit_stand": SitStand,
}


def make_human(p: Dict[str, float], kind: str = "pacing") -> Human:
    cls = HUMANS.get(kind, Pacing)
    return cls(p)


def desired_position(p: Dict[str, float], human: Human, t: float,
                     screen_offset=None) -> np.ndarray:
    """
    Where the *vehicle centroid* must go.

    The standoff is specified to the screen, because that is what the user
    cares about. What the controller flies is the rotor centroid, which sits
    a boom length further away when the display is cantilevered forward.
    Confusing the two puts the rotors exactly one boom length too close to
    the person.
    """
    h = human(t)
    d = float(p["standoff"])
    psi = h.heading
    if screen_offset is None:
        from .components import screen_sign
        screen_offset = [float(p.get("screen_forward", 0.0)), 0.0,
                         screen_sign(p)*float(p["r_cp"])]
    yaw = psi + math.pi
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c,-s,0], [s,c,0], [0,0,1]])
    return h.r + np.array([d * math.cos(psi), d * math.sin(psi),
                           float(p["standoff_z"])]) - R @ np.asarray(screen_offset)


class ReferenceGovernor:
    """
    Rate and acceleration limited reference.

    Tracking a person's face rigidly is the wrong specification: when they
    turn around, holding a fixed standoff in front of the eyes demands a
    half circle of radius 0.7 m in about a second, which is over 10 m/s^2
    and whips a 1.5 kg object past their head.  A real product lets the
    screen fall behind and catch up.

    This is a second order reference filter, the standard trajectory
    smoother: the governed reference chases the raw one under explicit
    speed and acceleration caps, so what reaches the controller is always
    something the aircraft can actually fly.

        a = clip(a_raw + kp (r_raw - r_g) + kd (v_raw - v_g), a_max)
        v_g = clip(v_g + a dt, v_max)

    The caps are the product decision; everything downstream is unchanged.
    """

    def __init__(self, p: Dict[str, float], r0: np.ndarray,
                 v_max: float = 2.5, a_max: float = 2.5,
                 kp: float = 16.0, kd: float = 8.0,
                 tracking_margin: float = 0.10,
                 yaw_rate_max: float = 1.5, yaw_acc_max: float = float("inf"),
                 yaw0: float = 0.0, j_max: float = 40.0):
        self.j_max = float(j_max)
        # Yaw is governed the same way as position. It is the weak axis: the
        # reaction torque of a slow quiet rotor is small, so the heading the
        # controller is asked to follow must respect what the rotors can do.
        self.yaw = float(yaw0)
        self.yaw_rate = 0.0
        self.yaw_acc = 0.0
        self.yaw_rate_max = float(yaw_rate_max)
        self.yaw_acc_max = float(yaw_acc_max)
        # The filter itself must be fast (kp, kd set about 4 rad/s, critically
        # damped) so that it is transparent during ordinary following and
        # only intervenes when the acceleration cap actually bites. A slow
        # filter would add a steady lag on every curved path, which is a
        # different and much worse behaviour than yielding during a turn.
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.kp = kp
        self.kd = kd
        self.r = np.asarray(r0, dtype=float).copy()
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.j = np.zeros(3)
        self.blocked = False
        self.clipped = False
        self.feasible = True
        self.tracking_margin = max(0.0, float(tracking_margin))

    def step(self, r_raw: np.ndarray, v_raw: np.ndarray, dt: float,
             keep_out: Optional[Tuple[np.ndarray, float]] = None,
             center_velocity=None, center_acceleration=None,
             a_raw=None, j_raw=None) -> None:
        """
        One update. The filter is

            a = a_raw + kp (r_raw - r_g) + kd (v_raw - v_g)

        with the raw acceleration fed forward, so that during ordinary
        following the governed reference is the raw one and the caps and the
        keep-out only act when they must. Without the feedforward the filter
        lags every acceleration by a_raw / kp, which is enough to be pushed
        into the keep-out sphere on a mission with no turn in it.
        """
        if dt <= 0 or self.a_max <= 0 or self.v_max <= 0:
            raise ValueError("governor dt and motion limits must be positive")
        a_ff = np.zeros(3) if a_raw is None else np.asarray(a_raw, dtype=float)
        j_ff = np.zeros(3) if j_raw is None else np.asarray(j_raw, dtype=float)
        e_r = np.asarray(r_raw) - self.r
        e_v = np.asarray(v_raw) - self.v
        a = a_ff + self.kp * e_r + self.kd * e_v
        raw_a = a.copy()
        cv = np.zeros(3) if center_velocity is None else np.asarray(center_velocity)
        ca = np.zeros(3) if center_acceleration is None else np.asarray(center_acceleration)
        n_hat, lower = None, -float("inf")
        if keep_out is not None:
            centre, radius = keep_out
            delta = self.r - np.asarray(centre)
            dist = float(np.linalg.norm(delta))
            n_hat = delta/dist if dist > 1e-9 else np.array([1.,0.,0.])
            relative_v = self.v-cv
            radial_v = float(n_hat @ relative_v)
            tangent2 = max(0., float(relative_v @ relative_v)-radial_v**2)
            # Relative-motion second-order barrier and one-step tangent-plane
            # clearance constraint under constant center acceleration.
            lower = max(float(n_hat @ ca)-tangent2/max(dist,1e-9)
                        -8*radial_v-16*(dist-radius),
                        float(n_hat @ ca)+2*(radius-dist-radial_v*dt)/dt**2)
        # Alternating projections onto convex acceleration/next-speed balls
        # and the keep-out halfspace. No position jumps or velocity resets.
        pushed = 0.0
        for _ in range(24):
            a *= min(1., self.a_max/max(float(np.linalg.norm(a)),1e-30))
            vn = self.v+a*dt
            vn *= min(1., self.v_max/max(float(np.linalg.norm(vn)),1e-30))
            a = (vn-self.v)/dt
            if n_hat is not None:
                pushed = max(0., lower-float(n_hat @ a))
                a += pushed*n_hat
        self.feasible = (np.linalg.norm(a) <= self.a_max+1e-6 and
                         np.linalg.norm(self.v+a*dt) <= self.v_max+1e-6)
        if not self.feasible:
            # Preserve the acceleration limit and report the incompatible
            # constraints; never disguise an impossible step by teleporting.
            a *= min(1., self.a_max/max(float(np.linalg.norm(a)),1e-30))
        # "blocked" means the keep-out acted, not that a speed or
        # acceleration cap did.
        self.blocked = bool(n_hat is not None and pushed > 1e-9)
        self.clipped = bool(np.linalg.norm(a-raw_a) > 1e-6)
        # Jerk, for the attitude-rate feedforward. Where the filter law was
        # not clipped its derivative is exact; where it was, a backward
        # difference of a clipped signal can spike, so it is bounded.
        if self.clipped:
            j = (a - self.a) / dt
            j *= min(1.0, self.j_max / max(float(np.linalg.norm(j)), 1e-30))
        else:
            j = j_ff + self.kp * e_v + self.kd * (a_ff - a)
        self.r = self.r + self.v*dt + .5*a*dt**2
        self.v = self.v + a*dt
        self.j = j
        self.a = a

    def step_yaw(self, yaw_raw: float, rate_raw: float, dt: float) -> None:
        """Second-order yaw filter with rate and acceleration caps."""
        e = _wrap(float(yaw_raw) - self.yaw)
        acc = self.kp * e + self.kd * (float(rate_raw) - self.yaw_rate)
        acc = max(-self.yaw_acc_max, min(self.yaw_acc_max, acc))
        r_new = max(-self.yaw_rate_max, min(self.yaw_rate_max, self.yaw_rate + acc * dt))
        acc = (r_new - self.yaw_rate) / dt
        self.yaw += self.yaw_rate * dt + 0.5 * acc * dt * dt
        self.yaw_rate = r_new
        self.yaw_acc = acc


def reference(p: Dict[str, float], human: Human, t: float,
              h_fd: float = 1e-3, screen_offset=None) -> Dict[str, Any]:
    """
    Desired screen state with true feedforward.

    The offset rotates with the heading, so a turning person generates a
    genuine centripetal demand on the vehicle.  Differentiating the full
    r_d(t) rather than only the human position is what removes the standing
    lag error on a curved path.
    """
    h = human(t)
    r0 = desired_position(p, human, t, screen_offset)
    rp = desired_position(p, human, t + h_fd, screen_offset)
    rm = desired_position(p, human, t - h_fd, screen_offset)
    v = (rp - rm) / (2.0 * h_fd)
    a = (rp - 2.0 * r0 + rm) / (h_fd * h_fd)
    # Jerk, for the angular-rate feedforward of the attitude loop.
    rpp = desired_position(p, human, t + 2.0 * h_fd, screen_offset)
    rmm = desired_position(p, human, t - 2.0 * h_fd, screen_offset)
    j = (rpp - 2.0 * rp + 2.0 * rm - rmm) / (2.0 * h_fd ** 3)
    psi = h.heading
    hp, hm = human(t+h_fd), human(t-h_fd)
    up = _wrap(hp.heading - psi)
    dn = _wrap(psi - hm.heading)
    return dict(r=r0, v=v, a=a, j=j, yaw=psi + math.pi, human=h.r, heading=psi,
                yaw_rate=(up + dn) / (2.0 * h_fd),
                yaw_acc=(up - dn) / (h_fd * h_fd),
                human_v=(hp.r-hm.r)/(2*h_fd),
                human_a=(hp.r-2*h.r+hm.r)/h_fd**2)


def _wrap(a: float) -> float:
    """Angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Wind
# ---------------------------------------------------------------------------
class Wind:
    """Mean wind plus a band-limited Ornstein-Uhlenbeck gust on each axis."""

    def __init__(self, p: Dict[str, float], t_end: float, dt: float = 0.02,
                 seed: int = 12345):
        self.mean = np.array([float(p["wind_mean"]), 0.0, 0.0])
        sigma = float(p["wind_gust"])
        self.dt = dt
        n = max(int(t_end / dt) + 2, 2)
        rng = np.random.default_rng(seed)
        tau = max(float(p["gust_tau"]), 1e-3)
        g = np.zeros((n, 3))
        if sigma > 0.0:
            a = math.exp(-dt / tau)
            s = sigma * math.sqrt(1.0 - a * a)
            for i in range(1, n):
                g[i] = a * g[i - 1] + s * rng.standard_normal(3)
            g[:, 2] *= 0.5                # vertical gusts are weaker
        self.g = g

    def __call__(self, t: float) -> np.ndarray:
        i = t / self.dt
        i0 = int(i)
        if i0 >= len(self.g) - 1:
            return self.mean + self.g[-1]
        f = i - i0
        return self.mean + (1.0 - f) * self.g[i0] + f * self.g[i0 + 1]


MISSIONS = list(HUMANS.keys())
