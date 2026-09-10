"""
Figure data for the design study and the limitations (sec17, sec18).

Writes results/fig/e_*.dat and results/fig/e_macros.tex.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np

from _common import PAPER, base_params, default_out, write_macros, write_table, GOV, SIM


def generate(out_dir: str) -> None:
    from flyingscreen import components as C
    from flyingscreen import mission as M
    from flyingscreen import payloads, tether
    from flyingscreen.dynamics import build_vehicle
    from flyingscreen.params import derived
    from flyingscreen.simulate import simulate_flight

    p = base_params()
    d = derived(p)
    base = C.close_first_principles(p)
    m, mb = base["m"], base["m_bat"]
    bd = base["breakdown"]
    mac = {}

    # -- readability against blade clearance, over standoff -----------------
    panel = dict(w=p["screen_w"], h=p["screen_h"], px_w=1920, px_h=1080,
                 m_screen=p["m_screen"])
    rows = []
    for dist in np.linspace(0.35, 2.0, 100):
        rd = payloads.readability(panel, float(dist))
        geo = C.geometry_relative_to_user(dict(p, standoff=float(dist)))
        rows.append((dist, rd["cap_arcmin_at_100"], rd["logical_w"],
                     rd["pixels_per_degree"], rd["angular_width"],
                     geo["tip_to_eye"]))
    write_table(out_dir, "e_readability",
                ["d", "cap", "logical", "ppd", "angle", "gap"], rows)
    geo0 = C.geometry_relative_to_user(p)
    d_safe_cross = geo0["standoff_for_safety"]
    # Distance at which 16 px text meets 16 arcmin with no scaling, and the
    # largest distance that still leaves a 1280-wide logical desktop.
    pitch = p["screen_w"] / 1920.0
    cap_m = payloads.CAP_RATIO * payloads.TEXT_EM_PX * pitch
    d_text = cap_m / math.tan(math.radians(payloads.GLYPH_MIN_ARCMIN / 60.0))
    d_full = d_text * 1920.0 / 1280.0
    rd_cross = payloads.readability(panel, d_safe_cross)
    mac.update(eDsafeCross="%.2f" % d_safe_cross, eDtext="%.2f" % d_text,
               eDfull="%.2f" % d_full,
               eLogicalAtCross="%.0f" % rd_cross["logical_w"])

    # -- geometry for the drawings ------------------------------------------
    arm = bd["arm"]
    R = p["D_rotor"] / 2.0
    mac.update(eArmL="%.4f" % arm["L"], eRotorR="%.4f" % R,
               eSpan="%.3f" % arm["span"], eReach="%.3f" % (arm["L"] + R),
               eScreenW="%.3f" % p["screen_w"], eScreenH="%.3f" % p["screen_h"],
               eRcp="%.3f" % p["r_cp"], eEye="%.3f" % p["standoff_z"],
               eZrotor="%.3f" % C.rotor_plane_height(p),
               eZrotorBelow="%.3f" % C.rotor_plane_height(dict(p, screen_above=0)),
               eStandoff="%.2f" % p["standoff"], eDsafe="%.2f" % p["d_safe"],
               eDeltaZ="%.3f" % abs(C.rotor_plane_height(p) - p["standoff_z"]))

    # -- displays: installed mass against the aircraft it needs --------------
    R_all = json.load(open(os.path.join(PAPER, "results", "results.json"),
                           encoding="utf-8"))
    # Symbolic names double as the bar-chart categories; the figure lists
    # them in this order, lightest payload first within each kind.
    tags = dict(laptop13="13.3 in laptop panel", laptop156="15.6 in laptop panel",
                ipad13_bare="iPad Pro 13 panel", ipad13="iPad Pro 13 whole",
                laptop156_lid="15.6 in lid assembly", desktop215="21.5 in monitor panel",
                desktop24="24 in monitor panel", laptop13_whole="13.3 in ultrabook",
                laptop156_whole="15.6 in laptop whole")
    ok_rows, bad_rows = [], []
    for k in tags:
        s = R_all["displays"]["rows"][k]
        c = payloads.BY_KEY[k]
        row = (c["m_screen"] + c["m_electronics"], s["m"], s["free_1m"], s["at_ear"],
               "{%s}" % tags[k])
        (ok_rows if s["ok"] else bad_rows).append(row)
    mac["eDisplayOrder"] = ",".join(tags.values())
    cols = ["payload", "gross", "spl", "ear", "tag"]
    write_table(out_dir, "e_displays_ok", cols, ok_rows)
    write_table(out_dir, "e_displays_bad", cols, bad_rows)
    mac.update(eOptSpl="%.0f" % R_all["displays"]["config"]["limits"]["spl_max"],
               eOptMass="%.1f" % R_all["displays"]["config"]["limits"]["m_max"])

    # -- the turnaround, flown ----------------------------------------------
    veh = build_vehicle(p, m, mb)
    s = simulate_flight(p, m, m_bat=mb, mission_kind="turnaround", governor=GOV, **SIM)
    human = M.make_human(p, "turnaround")
    tel, tr = s.telemetry, s.trajectory
    rows = []
    for i, t in enumerate(s.t):
        raw = M.reference(p, human, t, screen_offset=veh.screen_offset)["r"]
        r = np.asarray(tr["r"][i])
        rows.append((t, 1000 * tel["e_track"][i],
                     1000 * float(np.linalg.norm(np.asarray(raw) - r)),
                     tel["rotor_clearance"][i], tel["P"][i],
                     tel["screen_tilt_deg"][i], tel["tilt_deg"][i]))
    write_table(out_dir, "e_turn", ["t", "track", "lag", "clear", "P",
                                    "screen", "tilt"], rows)
    mac.update(eTurnClear="%.2f" % s.rotor_clearance_min,
               eTurnTrack="%.0f" % (1000 * s.e_track_max_all),
               eTrackMargin="%.0f" % (1000 * GOV["tracking_margin"]),
               eTurnPmean="%.1f" % s.P_mean)

    # -- the six human motions, and where the display ideally is --------------
    # (x, y, z) is the person, (rx, ry, rz) the ideal vehicle position that
    # puts the display at the standoff in front of their eyes, (dx, dy) the
    # vector from person to display, (hx, hy) the unit heading.
    for kind in M.HUMANS:
        h = M.make_human(p, kind)
        rows = []
        rz0 = M.reference(p, h, 0.0, screen_offset=veh.screen_offset)["r"][2]
        for t in np.linspace(0.0, SIM["t_window"], 321):
            st = h(float(t))
            full = M.reference(p, h, float(t), screen_offset=veh.screen_offset)
            ref = full["r"]
            rows.append((t, st.r[0], st.r[1], st.r[2], math.cos(st.heading),
                         math.sin(st.heading), ref[0], ref[1], ref[2],
                         ref[0] - st.r[0], ref[1] - st.r[1], ref[2] - rz0,
                         max(float(np.linalg.norm(full["a"])), 1e-3)))
        write_table(out_dir, "e_human_%s" % kind,
                    ["t", "x", "y", "z", "hx", "hy", "rx", "ry", "rz", "dx", "dy",
                     "rzrel", "acc"], rows)

    # -- coupled battery by mission ------------------------------------------
    names = dict(standing="standing", pacing="pacing", walk_loop="loop",
                 jogging="jogging", turnaround="turning")
    rows = []
    for i, k in enumerate(["standing", "pacing", "walk_loop", "jogging", "turnaround"]):
        cr = R_all["coupled"]["rows"][k]
        fr = R_all["flights"]["rows"][k]
        rows.append((i, names[k], 1000 * cr["m_bat"], 100 * fr["soc_end"],
                     1000 * cr["m"]))
    write_table(out_dir, "e_coupled", ["i", "mission", "mbat", "soc", "gross"], rows)
    mac.update(eHoverBat="%.0f" % (1000 * mb),
               eReservePct="%.0f" % (100 * p["soc_reserve"]))

    # -- tether conductor sizing ---------------------------------------------
    q = dict(p, tether_V=48.0, tether_len=5.0)
    tc = tether.tether_constants(q)
    rows = []
    for P_src in np.linspace(1.0, 400.0, 160):
        a_drop = 2.0 * tc["rho_e"] * tc["L"] * P_src / (tc["drop"] * tc["V"] ** 2) * 1e6
        a_amp = (P_src / tc["V"]) / tether.J_MAX_A_MM2
        a_min = tether.A_MIN_CONDUCTOR * 1e6
        rows.append((P_src, a_drop, a_amp, a_min, max(a_drop, a_amp, a_min)))
    write_table(out_dir, "e_tether", ["P", "drop", "amp", "hand", "req"], rows)
    tt = tether.close_tethered_first_principles(q)
    mac.update(eTethPsrc="%.1f" % tt["P_source"],
               eTethArea="%.3f" % tt["conductor_area_mm2"],
               eTethV="%.0f" % tc["V"], eTethL="%.0f" % tc["L"],
               eTethDrop="%.0f" % (100 * tc["drop"]))

    write_macros(out_dir, "e_macros", mac)


if __name__ == "__main__":
    generate(default_out())
