"""
The acoustic model, written out.

Quoting "51 dBA at 1 m" hides three things that matter for a machine that
hovers in a room next to a person: where that number comes from, how it
changes with distance, and the fact that in a room it very largely does not
change with distance at all.

---------------------------------------------------------------------------
1. Source strength: broadband rotor noise
---------------------------------------------------------------------------
Small rotor noise at these scales is dominated by broadband loading noise.
Blade sections shed unsteady lift, which radiates as a dipole, and dipole
acoustic power scales with the sixth power of the characteristic velocity:

    W_ac  ~  rho A_b V_tip^6 / c^3

Taking 10 log10 of that gives the tip speed term, thrust enters through the
blade loading, and N statistically independent rotors add incoherently:

    L_p(1 m) = C + 60 log10(V_tip / V_ref)
                 + 10 log10(T_rotor / T_ref)
                 + 10 log10(N)                                        (1)

with V_ref = 100 m/s, T_ref = 5 N, and the anchor C calibrated against
measured hardware rather than assumed (see components.py).

The sixth power is the whole design lever: halving tip speed is
60 log10(0.5) = -18 dB.

---------------------------------------------------------------------------
2. Sound power, and why the published numbers look worse than they are
---------------------------------------------------------------------------
EU 2019/945 makes manufacturers declare a sound *power* level L_WA, which is
a property of the source alone. Converting to a pressure at distance r over
a reflecting plane:

    L_p(r) = L_WA + 10 log10( Q / (4 pi r^2) )                        (2)

so at r = 1 m with directivity Q = 2 (a source radiating into a hemisphere,
which is what a rotor over a floor is):

    L_p(1 m) = L_WA - 10 log10(2 pi) = L_WA - 8.0 dB                  (3)

Reading a declared 83 dB as a pressure at 1 m rather than a power is an
8 dB error, and it is the most common mistake made with these figures.

---------------------------------------------------------------------------
3. The room, which is the part that actually decides whether you can work
---------------------------------------------------------------------------
Free field is the wrong model indoors. In a room the level is the sum of a
direct field that falls with distance and a reverberant field that does not:

    L_p(r) = L_W + 10 log10( Q / (4 pi r^2)  +  4 / R )               (4)

    R = S * alpha_bar / (1 - alpha_bar)                               (5)

R is the room constant, S the total surface area and alpha_bar the average
absorption coefficient. The two terms are equal at the critical distance

    r_c = sqrt( Q R / (16 pi) )                                       (6)

Beyond r_c, moving away from the machine buys nothing: the level is set by
how absorbent the room is, not by how far away you stand. For an ordinary
furnished room r_c lands under a metre, which means **a flying screen makes
the whole room as loud as the space right next to it.**

---------------------------------------------------------------------------
4. Tones
---------------------------------------------------------------------------
Broadband level is what equation (1) predicts. What makes a rotor
*annoying* rather than merely loud is tonal content at the blade passage
frequency and its harmonics:

    f_BPF = n_blades * rpm / 60                                       (7)

A-weighting strongly attenuates low frequencies, so a slow rotor's
fundamental is largely weighted away; that is a real benefit and it is also
a warning, because the energy is still there as unweighted low frequency
sound, which rooms absorb poorly and people perceive as a throb rather than
a hiss. Several rotors at slightly different speeds beat against each
other, which is worse again. None of that is in the broadband number.

A-weighting is the standard IEC 61672 curve:

    R_A(f) = 12194^2 f^4 /
             ( (f^2 + 20.6^2)(f^2 + 12194^2)
               sqrt((f^2 + 107.7^2)(f^2 + 737.9^2)) )                 (8)
    A(f)   = 20 log10(R_A(f)) + 2.00
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

V_REF = 100.0            # m/s, tip speed reference for equation (1)
T_REF = 5.0              # N, thrust reference for equation (1)
Q_HEMI = 2.0             # directivity of a source over one reflecting plane

# Typical average absorption coefficients, broadband, mid frequency.
ROOM_ABSORPTION = {
    "bare hard room": 0.10,
    "ordinary furnished room": 0.20,
    "soft furnished, carpet and curtains": 0.35,
    "acoustically treated": 0.55,
}


def a_weighting(f: float) -> float:
    """IEC 61672 A-weighting, dB, equation (8)."""
    if f <= 0:
        return -math.inf
    f2 = f * f
    num = (12194.0 ** 2) * (f2 ** 2)
    den = ((f2 + 20.6 ** 2)
           * math.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2))
           * (f2 + 12194.0 ** 2))
    return 20.0 * math.log10(num / den) + 2.00


def blade_passage_frequency(rpm: float, n_blades: float) -> float:
    """Equation (7)."""
    return n_blades * rpm / 60.0


def room_constant(length: float, width: float, height: float,
                  alpha_bar: float) -> Dict[str, float]:
    """Equation (5), with the surface area it came from."""
    S = 2.0 * (length * width + length * height + width * height)
    alpha_bar = min(max(alpha_bar, 1e-3), 0.95)
    R = S * alpha_bar / (1.0 - alpha_bar)
    return dict(surface_area=S, alpha_bar=alpha_bar, R=R,
                volume=length * width * height)


def critical_distance(R: float, Q: float = Q_HEMI) -> float:
    """Equation (6): where direct and reverberant fields are equal."""
    return math.sqrt(max(Q * R, 0.0) / (16.0 * math.pi))


def spl_at(L_W: float, r: float, R: Optional[float] = None,
           Q: float = Q_HEMI) -> Dict[str, float]:
    """
    Equation (4). With R given this is the in-room level; without it, free
    field over a reflecting plane, equation (2).
    """
    r = max(r, 0.05)
    direct_term = Q / (4.0 * math.pi * r * r)
    reverb_term = (4.0 / R) if (R and R > 0) else 0.0
    total = L_W + 10.0 * math.log10(direct_term + reverb_term)
    return dict(
        total=total,
        direct=L_W + 10.0 * math.log10(direct_term),
        reverberant=(L_W + 10.0 * math.log10(reverb_term)
                     if reverb_term > 0 else -math.inf),
        r=r,
    )


def rotor_noise(v_tip: float, T_rotor: float, N: float,
                anchor: float) -> Dict[str, float]:
    """
    Equation (1), with each term kept separate so the design lever is
    visible rather than buried in a single number.
    """
    tip_term = 60.0 * math.log10(max(v_tip, 1.0) / V_REF)
    thrust_term = 10.0 * math.log10(max(T_rotor, 1e-3) / T_REF)
    count_term = 10.0 * math.log10(max(N, 1.0))
    spl_1m = anchor + tip_term + thrust_term + count_term
    return dict(
        spl_1m=spl_1m, anchor=anchor, tip_term=tip_term,
        thrust_term=thrust_term, count_term=count_term,
        L_WA=spl_1m + 10.0 * math.log10(2.0 * math.pi),   # invert eq (3)
        v_tip=v_tip, T_rotor=T_rotor, N=N,
    )


def report(p: Dict[str, float], rotor: Any, N: float,
           anchor: float) -> Dict[str, Any]:
    """
    The full acoustic picture for one design: source terms, the room, the
    level where the person actually is, and the tonal content the broadband
    number does not contain.
    """
    src = rotor_noise(rotor.v_tip, rotor.T, N, anchor)
    L_WA = src["L_WA"]

    room = room_constant(float(p["room_length"]), float(p["room_width"]),
                         float(p["ceiling_height"]), float(p["room_alpha"]))
    r_c = critical_distance(room["R"])

    at_listener = spl_at(L_WA, float(p["standoff"]), room["R"])
    at_one_m = spl_at(L_WA, 1.0, room["R"])
    far = spl_at(L_WA, max(float(p["room_length"]), float(p["room_width"])) * 0.9,
                 room["R"])
    free_1m = spl_at(L_WA, 1.0, None)

    bpf = blade_passage_frequency(rotor.rpm, float(p["n_blades"]))
    harmonics = []
    for k in (1, 2, 3, 4):
        f = bpf * k
        harmonics.append(dict(harmonic=k, f=f, a_weight=a_weighting(f)))

    notes = []
    if r_c < float(p["standoff"]):
        notes.append(
            "The critical distance is %.2f m and you sit at %.2f m, so you are "
            "already in the reverberant field: walking away from the machine "
            "does not make it quieter, it just moves you inside the same "
            "%.0f dBA room." % (r_c, float(p["standoff"]), far["total"]))
    else:
        notes.append(
            "The critical distance is %.2f m against a %.2f m standoff, so you "
            "sit in the direct field, but anyone else in the room hears a "
            "steady %.0f dBA regardless of where they stand."
            % (r_c, float(p["standoff"]), far["total"]))
    if bpf < 100.0:
        notes.append(
            "Blade passage is %.0f Hz, which A-weighting discounts by %.0f dB. "
            "That flatters the dBA figure: the energy is still there as low "
            "frequency sound, which rooms absorb poorly and people feel as a "
            "throb rather than hear as a hiss."
            % (bpf, -a_weighting(bpf)))
    notes.append(
        "Rotors running at slightly different speeds beat against one another "
        "at the difference frequency. That is perceptually worse than a steady "
        "tone and no broadband model contains it.")

    return dict(
        source=src, L_WA=L_WA, room=room, critical_distance=r_c,
        at_listener=at_listener, at_one_m=at_one_m, far_field=far,
        free_field_1m=free_1m,
        bpf=bpf, harmonics=harmonics,
        reverberant_level=far["total"],
        absorption_options={k: spl_at(
            L_WA, float(p["standoff"]),
            room_constant(float(p["room_length"]), float(p["room_width"]),
                          float(p["ceiling_height"]), a)["R"])["total"]
            for k, a in ROOM_ABSORPTION.items()},
        notes=notes,
    )
