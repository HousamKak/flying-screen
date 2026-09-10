"""
Figure data for sections 3 to 5 (worker A).

    a_tipspeed.dat   one rotor of the design at its hover thrust, swept over
                     tip speed with automatic tip speed off: induced, profile,
                     shaft and ideal power, blade loading, figure of merit,
                     Reynolds-corrected profile drag coefficient
    a_macros.tex     thrust per rotor, the tip speeds at the design and the
                     stall blade loading, the optimum, air density, disk
                     loading and wake speed of the design
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import base_params, default_out, write_macros, write_table  # noqa: E402


def generate(out_dir: str) -> None:
    from flyingscreen import components as C

    p = base_params()
    fp = C.close_first_principles(p)
    m = fp["m"]
    N = float(p["N_rotors"])
    g = float(p["g"])
    rho = float(p["rho"])
    T = m * g / N
    R = float(p["D_rotor"]) / 2.0
    A_rot = math.pi * R * R
    q = dict(p, tip_speed_auto=0)

    rows = []
    v = 18.0
    while v <= 80.0 + 1e-9:
        rp = C.rotor_point(dict(q, tip_speed=v), T)
        rows.append((v, rp.P_induced, rp.P_profile, rp.P_shaft, rp.P_ideal,
                     rp.Ct_sigma, rp.FM_effective, rp.cd0_effective, rp.reynolds,
                     1.0 if rp.stalled else 0.0))
        v += 0.5
    write_table(out_dir, "a_tipspeed",
                ["vtip", "Pind", "Pprof", "Pshaft", "Pideal", "ctsig", "FM",
                 "cdz", "Re", "stalled"], rows)

    sigma = float(p["n_blades"]) / (math.pi * float(p["blade_AR"]))

    def v_at(ctsig: float) -> float:
        return math.sqrt(T / (rho * A_rot * sigma * ctsig))

    # At fixed geometry the induced power does not depend on tip speed and
    # the profile power grows with it, so the least shaft power the blade
    # loading limit admits is on the stall boundary itself.
    v_stall = v_at(float(p["ct_sigma_max"]))
    at_stall = C.rotor_point(dict(q, tip_speed=v_stall), T)
    hov = C.rotor_point(dict(q, tip_speed=v_at(float(p["ct_sigma_design"]))), T)
    DL = m * g / (N * A_rot)
    write_macros(out_dir, "a_macros", {
        "aThover": "%.2f" % T,
        "aVstall": "%.2f" % v_stall,
        "aVdesign": "%.2f" % v_at(float(p["ct_sigma_design"])),
        "aPstall": "%.2f" % at_stall.P_shaft,
        "aPdesign": "%.2f" % hov.P_shaft,
        "aPind": "%.2f" % hov.P_induced,
        "aCtMax": "%.2f" % float(p["ct_sigma_max"]),
        "aCtDesign": "%.2f" % float(p["ct_sigma_design"]),
        "aSigma": "%.3f" % sigma,
        "aRho": "%.3f" % rho,
        "aDL": "%.1f" % DL,
        "aWake": "%.2f" % (2.0 * math.sqrt(DL / (2.0 * rho))),
        "aKappa": "%.3f" % hov.kappa_total,
    })


if __name__ == "__main__":
    generate(default_out())
