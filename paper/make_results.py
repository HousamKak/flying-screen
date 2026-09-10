"""
Every number in the design-study part of the paper, computed from the engine.

    python paper/make_results.py                 # everything
    python paper/make_results.py flights coupled # only some sections

Writes paper/results/results.json (numbers plus the configuration that
produced them) and paper/results/macros.tex and table-row files that the
LaTeX sources \\input. Sections not recomputed keep their previous values in
results.json, so a partial run still emits a complete macro file.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from typing import Any, Dict, List

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from flyingscreen import acoustics, closure, payloads, sizing, sweep, tether  # noqa: E402
from flyingscreen import components as C  # noqa: E402
from flyingscreen import optimize as O  # noqa: E402
from flyingscreen import params as P  # noqa: E402
from flyingscreen.control import control_authority, suggest_gains  # noqa: E402
from flyingscreen.dynamics import build_vehicle  # noqa: E402
from flyingscreen.electrical import electrical_chain  # noqa: E402
from flyingscreen.params import clamp_params, derived  # noqa: E402
from flyingscreen.simulate import simulate_flight  # noqa: E402

OUT = os.path.join(HERE, "results")
PRESET = "DESK-1/W, full 1920x1080 desktop"
PAYLOAD = "pi_thinclient"
SIM = dict(t_window=16.0, dt=0.004, settle=3.0, seed=12345)
GOV = dict(v_max=2.5, a_max=2.5, tracking_margin=0.10, yaw_rate_max=1.5)
CLOSE = dict(tol=2e-3, max_iter=10)
OPT = dict(variables=["D_rotor", "ct_sigma_design", "blade_AR", "n_blades"],
           weights=dict(w_m=0.6, w_P=0.004, w_N=0.35, w_span=1.2),
           limits=dict(span_max=1.45, spl_max=58.0, downwash_max=7.5, m_max=6.0,
                       lam_margin=1.35),
           maxiter=30, popsize=12, seed=3)
MISSIONS = ["standing", "pacing", "walk_loop", "jogging", "turnaround", "sit_stand"]
COUPLED = ["standing", "pacing", "walk_loop", "jogging", "turnaround"]
DISPLAYS = ["laptop13", "laptop156", "ipad13", "ipad13_bare", "desktop215",
            "desktop24", "laptop156_lid", "laptop13_whole", "laptop156_whole"]


def base_params() -> Dict[str, float]:
    return payloads.apply(P.preset_params(PRESET), PAYLOAD)


def panel_of(p: Dict[str, float]) -> Dict[str, Any]:
    return dict(w=p["screen_w"], h=p["screen_h"], px_w=1920, px_h=1080,
                m_screen=p["m_screen"])


def acoustic(p: Dict[str, float], m: float) -> Dict[str, float]:
    rep = acoustics.report(p, C.propulsion(p, m).rotor_hover, p["N_rotors"], C._SPL_REF)
    return dict(free_1m=rep["free_field_1m"]["total"], at_ear=rep["at_listener"]["total"],
                far=rep["far_field"]["total"], r_c=rep["critical_distance"],
                bpf=rep["bpf"])


def flight_row(s) -> Dict[str, Any]:
    keys = ("ok", "reason", "P_mean", "P_peak", "P_hover_ref", "overhead", "e_track_rms",
            "e_track_max_all", "margin_holds", "tracking_margin", "e_lag_rms", "e_lag_max",
            "tilt_max_deg", "screen_tilt_max_deg", "saturation_fraction",
            "min_human_distance", "rotor_clearance_min", "ref_keepout_margin_min",
            "governor_infeasible_fraction", "soc_end", "constraints_ok", "violations",
            "config")
    return {k: getattr(s, k) for k in keys}


# ---------------------------------------------------------------------------
def sec_design(R: Dict[str, Any]) -> None:
    p = base_params()
    d = derived(p)
    r = C.close_first_principles(p)
    m, mb = r["m"], r["m_bat"]
    b = r["breakdown"]
    hov, mx = b["rotor"]["hover"], b["rotor"]["max"]
    veh = build_vehicle(p, m, mb)
    auth = control_authority(veh)
    sg = suggest_gains(veh)
    geo = C.geometry_relative_to_user(p)
    chain = electrical_chain(p, m, mb)["chosen"] or {}
    rd = payloads.readability(panel_of(p), p["standoff"])
    rd70 = payloads.readability(panel_of(p), 0.70)
    e_use = d.e_b * (1.0 - p["soc_reserve"])
    wall20 = sizing.payload_wall(dict(p, t_f=1200.0))
    below = C.close_first_principles(dict(p, screen_above=0))
    R["design"] = dict(
        z_rotor_above=C.rotor_plane_height(p),
        z_rotor_below=C.rotor_plane_height(dict(p, screen_above=0)),
        m_below=below["m"] if below["feasible"] else float("nan"),
        params=p, status=r["status"], m=m, m_bat=mb, P_total=b["P_total"], P_aux=d.P_aux,
        m_fix=d.m_fix, m_screen=p["m_screen"], m_electronics=p["m_electronics"],
        m_gimbal=p["m_gimbal"], m_frame=b["m_frame"], m_arms=b["arm"]["m_arms"],
        m_guards=b["arm"]["m_guards"], m_prop=b["m_prop"],
        m_motors=b["propulsion"]["m_motors"], m_esc=b["propulsion"]["m_esc"],
        m_props=b["propulsion"]["m_props"], FM=b["FM_implied"],
        rpm=hov["rpm"], rpm_max=mx["rpm"], v_tip=hov["v_tip"], v_tip_max=mx["v_tip"],
        ct_sigma=hov["Ct_sigma"], reynolds=hov["reynolds"], cd0=hov["cd0_effective"],
        chord=hov["chord"], sigma=hov["sigma"], span=b["span"], reach=geo["reach"],
        D=p["D_rotor"], n_blades=p["n_blades"], AR=p["blade_AR"],
        wake=b["downwash"], tip_gap=geo["tip_to_eye"], d_safe=p["d_safe"],
        endurance_reserve=mb * e_use / b["P_total"],
        endurance_empty=mb * d.e_b / b["P_total"],
        pack=dict(cell=chain.get("cell"), S=chain.get("S"), wh=chain.get("energy_wh")),
        acoustic=acoustic(p, m),
        J=np.asarray(veh.J).tolist(), authority=auth,
        gains=dict(K_R=p["K_R"], K_w=p["K_w"], Kp=p["Kp_pos"], Kd=p["Kd_pos"]),
        gains_suggested=dict(K_R=sg["K_R"], K_w=sg["K_w"]),
        tau_motor=p["tau_motor"],
        readability=rd, readability_070=rd70,
        wall20=dict(M0_max=wall20["M0_max"], M0=sizing.coefficients(dict(p, t_f=1200.0)).M0),
    )


def sec_flights(R: Dict[str, Any]) -> None:
    p = base_params()
    r = C.close_first_principles(p)
    rows = {}
    for k in MISSIONS:
        t0 = time.time()
        s = simulate_flight(p, r["m"], m_bat=r["m_bat"], mission_kind=k,
                            governor=GOV, **SIM)
        rows[k] = flight_row(s)
        print("  flight %-10s %.1fs" % (k, time.time() - t0), flush=True)
    wind = {}
    for label, wm, wg in (("draught", 0.8, 0.4), ("fan", 1.5, 0.8), ("breeze", 3.0, 1.5)):
        q = dict(p, wind_mean=wm, wind_gust=wg)
        s = simulate_flight(q, r["m"], m_bat=r["m_bat"], mission_kind="pacing",
                            governor=GOV, **SIM)
        wind[label] = dict(flight_row(s), wind_mean=wm, wind_gust=wg)
    R["flights"] = dict(rows=rows, wind=wind, config=dict(SIM, governor=GOV))


def sec_coupled(R: Dict[str, Any]) -> None:
    p = base_params()
    rows = {}
    for k in COUPLED:
        t0 = time.time()
        run = closure.close_with_simulation(p, mission_kind=k, governor=GOV,
                                            keep_telemetry=False, **SIM, **CLOSE)
        veh = build_vehicle(p, run.m, run.m_bat)
        s = run.sim
        rows[k] = dict(
            status=run.status, converged=run.converged, feasible=run.feasible,
            battery_ok=run.battery_ok, constraints_ok=run.constraints_ok,
            m=run.m, m_bat=run.m_bat, P_mean=run.P_mean, overhead=run.overhead,
            iterations=run.iterations, sim_calls=run.sim_calls,
            energy_margin=run.energy_margin, power_margin=run.power_margin,
            reserve_end=run.reserve_end, Jyy=float(veh.J[1, 1]),
            e_lag_rms=s.get("e_lag_rms"), rotor_clearance_min=s.get("rotor_clearance_min"),
            violations=s.get("violations"), trace=run.trace)
        print("  coupled %-10s %s %d sims %.0fs" % (k, run.status, run.sim_calls,
                                                   time.time() - t0), flush=True)
    R["coupled"] = dict(rows=rows, config=dict(SIM, governor=GOV, **CLOSE))


def sec_clearance(R: Dict[str, Any]) -> None:
    b0 = base_params()
    options = [("d070", dict(standoff=0.70)), ("d095", dict(standoff=0.95)),
               ("d120", dict(standoff=1.20)),
               ("b020", dict(standoff=0.70, screen_forward=0.20)),
               ("b038", dict(standoff=0.70, screen_forward=0.38)),
               ("b050", dict(standoff=0.70, screen_forward=0.50))]
    rows = {}
    for key, change in options:
        q = dict(b0, **change)
        r = C.close_first_principles(q)
        if not r["feasible"]:
            rows[key] = dict(feasible=False, change=change)
            continue
        veh = build_vehicle(q, r["m"], r["m_bat"])
        auth = control_authority(veh)
        # Attitude gains capped by the heuristic bound for this option: a
        # boom lowers the pitch authority, and flying it at the no-boom gains
        # would measure the gains, not the geometry.
        sg = suggest_gains(veh, a_max=GOV["a_max"])
        if sg["K_R"] < q["K_R"]:
            q = dict(q, K_R=sg["K_R"], K_w=sg["K_w"], Kp_pos=sg["Kp_pos"],
                     Kd_pos=sg["Kd_pos"], Ki_pos=sg["Ki_pos"])
        geo = C.geometry_relative_to_user(q)
        rd = payloads.readability(panel_of(q), q["standoff"])
        s = simulate_flight(q, r["m"], m_bat=r["m_bat"], mission_kind="turnaround",
                            governor=GOV, **SIM)
        rows[key] = dict(feasible=True, change=change, m=r["m"],
                         P=r["breakdown"]["P_total"], Jyy=float(veh.J[1, 1]),
                         Jxz=float(veh.J[0, 2]), alpha_pitch=auth["alpha_pitch"],
                         K_R=q["K_R"], K_w=q["K_w"], K_R_bound=sg["K_R"],
                         Kp=q["Kp_pos"], started_inside=s.started_inside,
                         tip_gap=geo["tip_to_eye"], readability=rd,
                         at_ear=acoustic(q, r["m"])["at_ear"],
                         turn=flight_row(s))
        print("  clearance %s done" % key, flush=True)
    R["clearance"] = dict(rows=rows, config=dict(SIM, governor=GOV))


def optimised(q: Dict[str, float]) -> Dict[str, Any]:
    res = O.optimise(q, variables=OPT["variables"], weights=OPT["weights"],
                     limits=OPT["limits"], maxiter=OPT["maxiter"],
                     popsize=OPT["popsize"], seed=OPT["seed"])
    p2 = clamp_params(dict(q, **res["best"]["x"]))
    r = C.close_first_principles(p2)
    return dict(params=p2, closure=r, violations=res["best"]["violations"],
                ok=res["best"]["ok"])


def sec_displays(R: Dict[str, Any]) -> None:
    b0 = P.preset_params(PRESET)
    rows = {}
    for k in DISPLAYS:
        t0 = time.time()
        c = payloads.BY_KEY[k]
        q = payloads.apply(b0, k)
        o = optimised(q)
        r = o["closure"]
        rd = payloads.readability(c, b0["standoff"])
        row = dict(name=c["name"], readability=rd, areal=c["m_screen"] / (c["w"] * c["h"]),
                   whole=bool(c.get("self_powered")) and k != "ipad13",
                   feasible=r["feasible"], ok=o["ok"], violations=o["violations"],
                   x={v: o["params"][v] for v in OPT["variables"]})
        if r["feasible"]:
            row.update(m=r["m"], P=r["breakdown"]["P_total"],
                       **acoustic(o["params"], r["m"]))
        rows[k] = row
        print("  display %-16s %.0fs  ok=%s" % (k, time.time() - t0, o["ok"]), flush=True)
    R["displays"] = dict(rows=rows, config=dict(OPT, preset=PRESET,
                                                 standoff=b0["standoff"]))


def sec_hex(R: Dict[str, Any]) -> None:
    out = {}
    for label, N in (("quad", 4.0), ("hex", 6.0)):
        q = dict(base_params(), N_rotors=N)
        o = optimised(q)
        r = o["closure"]
        row = dict(ok=o["ok"], violations=o["violations"], feasible=r["feasible"])
        if r["feasible"]:
            veh = build_vehicle(o["params"], r["m"], r["m_bat"])
            auth = control_authority(veh)
            row.update(m=r["m"], P=r["breakdown"]["P_total"], span=r["breakdown"]["span"],
                       D=o["params"]["D_rotor"], authority=auth,
                       J=np.asarray(veh.J).tolist(), **acoustic(o["params"], r["m"]))
            if N == 6.0:
                red = veh.alloc[:, [1, 2, 4, 5]]
                red = red / np.linalg.norm(red, axis=1, keepdims=True)
                row["rank_opposite_pair_removed"] = int(np.linalg.matrix_rank(red))
        out[label] = row
        print("  %s done" % label, flush=True)
    R["hex"] = dict(rows=out, config=OPT)


def sec_tether(R: Dict[str, Any]) -> None:
    # A desk-side supply: 5 m of lead at 48 V, below the 60 V DC limit for
    # safety extra-low voltage.
    p = dict(base_params(), tether_V=48.0, tether_len=5.0)
    t = tether.close_tethered_first_principles(p)
    lumped = tether.solve_tethered(p)
    R["tether"] = dict(component={k: v for k, v in t.items() if k != "breakdown"},
                       at_ear=acoustic(p, t["m"])["at_ear"],
                       lumped={k: v for k, v in lumped.items() if k != "closure"},
                       lumped_closure=lumped.get("closure"),
                       length=p["tether_len"], V=p["tether_V"])


def sec_pwall(R: Dict[str, Any]) -> None:
    from scipy.optimize import minimize_scalar
    p = dict(P.DEFAULTS)
    anchor = sizing.coefficients(p, area_p=0.0).anchor_mass
    rows = []
    for pp in (-0.25, 0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5):
        t = sweep.endurance_wall(p, area_p=pp, anchor=anchor)
        c = sizing.coefficients(p, area_p=pp, anchor_mass=anchor)
        row = dict(p=pp, q=(3 - pp) / 2, t_max=t)
        if pp < 1.0:
            # Mass at which the endurance ceiling is attained, the envelope
            # argument's m-hat, found by maximising the admissible duration.
            def neg(lm):
                m = math.exp(lm)
                return -(c.alpha * m - c.m_fix) * c.e_b / (c.c_P * m ** c.q + c.P_aux)
            opt = minimize_scalar(neg, bounds=(math.log(1e-3), math.log(1e6)),
                                  method="bounded", options=dict(xatol=1e-12))
            row.update(t_sup=-float(opt.fun), m_hat=math.exp(opt.x), m0=anchor)
        elif pp == 1.0:
            row.update(t_sup=c.alpha * c.e_b / c.c_P, m0=anchor)
        rows.append(row)
    R["pwall"] = dict(rows=rows, anchor=anchor, params="DEFAULTS")


def sec_nearfield(R: Dict[str, Any]) -> None:
    p = base_params()
    rows = {}
    for key, ige, inter in (("both", 1, 1), ("nosurface", 0, 1), ("nointerference", 1, 0),
                            ("neither", 0, 0)):
        q = dict(p, ige_enable=ige, interference_enable=inter)
        r = C.close_first_principles(q)
        rows[key] = dict(m=r["m"], P=r["breakdown"]["P_total"], spl=r["breakdown"]["spl_1m"],
                         kappa_total=r["breakdown"]["rotor"]["hover"]["kappa_total"])
    R["nearfield"] = dict(rows=rows)


SECTIONS = dict(design=sec_design, flights=sec_flights, coupled=sec_coupled,
                nearfield=sec_nearfield,
                clearance=sec_clearance, displays=sec_displays, hex=sec_hex,
                tether=sec_tether, pwall=sec_pwall)


# ---------------------------------------------------------------------------
# LaTeX emission
# ---------------------------------------------------------------------------
def f(x: float, n: int = 1) -> str:
    return ("%%.%df" % n) % x


def emit(R: Dict[str, Any]) -> None:
    M: List[str] = ["% Generated by paper/make_results.py. Do not edit by hand."]

    def mac(name: str, val: str) -> None:
        M.append("\\newcommand{\\r%s}{%s}" % (name, val))

    D = R.get("design")
    if D:
        a = D["authority"]
        mac("Gross", f(D["m"], 3)); mac("GrossG", f(1000 * D["m"], 0))
        mac("Battery", f(1000 * D["m_bat"], 0)); mac("Power", f(D["P_total"], 1))
        mac("Paux", f(D["P_aux"], 1)); mac("Fixed", f(1000 * D["m_fix"], 0))
        mac("Panel", f(1000 * D["m_screen"], 0)); mac("Electronics", f(1000 * D["m_electronics"], 0))
        mac("Gimbal", f(1000 * D["m_gimbal"], 0)); mac("Frame", f(1000 * D["m_frame"], 0))
        mac("Arms", f(1000 * D["m_arms"], 0)); mac("Guards", f(1000 * D["m_guards"], 0))
        mac("Prop", f(1000 * D["m_prop"], 0)); mac("Motors", f(1000 * D["m_motors"], 0))
        mac("Props", f(1000 * D["m_props"], 0)); mac("FM", f(D["FM"], 3))
        mac("Rpm", f(D["rpm"], 0)); mac("RpmMax", f(D["rpm_max"], 0))
        mac("Vtip", f(D["v_tip"], 1)); mac("VtipMax", f(D["v_tip_max"], 1))
        mac("CtSigma", f(D["ct_sigma"], 3)); mac("Reynolds", "\\num{%d}" % round(D["reynolds"], -3))
        mac("Cdzero", f(D["cd0"], 3)); mac("Chord", f(1000 * D["chord"], 0))
        mac("Solidity", f(D["sigma"], 3)); mac("Span", f(D["span"], 2)); mac("Reach", f(D["reach"], 2))
        mac("Diam", f(1000 * D["D"], 0)); mac("Wake", f(D["wake"], 1))
        mac("TipGap", f(D["tip_gap"], 3))
        mac("EndReserve", f(D["endurance_reserve"] / 60, 1))
        mac("EndEmpty", f(D["endurance_empty"] / 60, 1))
        pk = D["pack"]
        mac("PackCell", str(pk["cell"]).split(" ")[0] if pk["cell"] else "n/a")
        mac("PackS", str(pk["S"] or "n/a")); mac("PackWh", f(pk["wh"], 1) if pk["wh"] else "n/a")
        ac = D["acoustic"]
        mac("SplOne", f(ac["free_1m"], 1)); mac("SplEar", f(ac["at_ear"], 1))
        mac("SplFar", f(ac["far"], 1)); mac("CritDist", f(ac["r_c"], 2)); mac("Bpf", f(ac["bpf"], 0))
        mac("AlphaRoll", f(a["alpha_roll"], 0)); mac("AlphaPitch", f(a["alpha_pitch"], 0))
        mac("AlphaYaw", f(a["alpha_yaw"], 1)); mac("TauRoll", f(a["tau_roll"], 2))
        mac("TauYaw", f(a["tau_yaw"], 3))
        mac("Jxx", f(D["J"][0][0], 4)); mac("Jyy", f(D["J"][1][1], 4)); mac("Jzz", f(D["J"][2][2], 4))
        mac("KR", f(D["gains"]["K_R"], 1)); mac("Kw", f(D["gains"]["K_w"], 1))
        mac("KRsug", f(D["gains_suggested"]["K_R"], 1))
        mac("KRlag", f((0.3 / D["tau_motor"]) ** 2, 1))
        mac("KRauth", f(0.5 * min(a["alpha_roll"], a["alpha_pitch"]) / math.radians(25.0), 0))
        rd, r7 = D["readability"], D["readability_070"]
        mac("Ppd", f(rd["pixels_per_degree"], 0)); mac("ScreenDeg", f(rd["angular_width"], 1))
        mac("CapArcmin", f(rd["cap_arcmin_at_100"], 1)); mac("UiScale", f(100 * rd["ui_scale_min"], 0))
        mac("UiScalePref", f(100 * rd["ui_scale_preferred"], 0))
        mac("LogicalW", f(rd["logical_w"], 0)); mac("LogicalH", f(rd["logical_h"], 0))
        mac("CapArcminNear", f(r7["cap_arcmin_at_100"], 1))
        mac("LogicalWNear", f(r7["logical_w"], 0))
        mac("WallTwenty", f(D["wall20"]["M0_max"], 2)); mac("MzeroTwenty", f(D["wall20"]["M0"], 2))
        mac("WallUse", f(100 * D["wall20"]["M0"] / D["wall20"]["M0_max"], 0))
        mac("ZrotorAbove", f(D["z_rotor_above"], 2)); mac("ZrotorBelow", f(D["z_rotor_below"], 2))
        mac("GrossBelow", f(D["m_below"], 3))
        mac("Standoff", f(D["params"]["standoff"], 2)); mac("EyeHeight", f(D["params"]["standoff_z"], 2))

    Fl = R.get("flights")
    if Fl:
        rows = Fl["rows"]
        names = dict(standing="Standing", pacing="Pacing", walk_loop="Walking a loop",
                     jogging="Jogging", turnaround="Turning around", sit_stand="Sit to stand")
        T = []
        e_use = derived(D["params"]).e_b * (1 - D["params"]["soc_reserve"]) if D else 0.0
        for k in MISSIONS:
            s = rows[k]
            T.append("%s & \\SI{%s}{\\watt} & %s & \\SI{%s}{\\minute} & \\SI{%s}{\\milli\\metre} & \\SI{%s}{\\milli\\metre} & \\ang{%s} & \\SI{%s}{\\percent} & \\SI{%s}{\\metre} & \\SI{%s}{\\metre}\\\\"
                     % (names[k], f(s["P_mean"], 1), f(s["overhead"], 3),
                        f(D["m_bat"] * e_use / s["P_mean"] / 60, 1) if D else "n/a",
                        f(1000 * s["e_track_max_all"], 0), f(1000 * s["e_lag_rms"], 0),
                        f(s["screen_tilt_max_deg"], 2), f(100 * s["saturation_fraction"], 1),
                        f(s["min_human_distance"], 2), f(s["rotor_clearance_min"], 2)))
        write("tab_flight_rows.tex", T)
        ov = [rows[k]["overhead"] for k in MISSIONS]
        mac("KappaMin", f(min(ov), 3)); mac("KappaMax", f(max(ov), 3))
        tr = rows["turnaround"]
        mac("TurnClear", f(tr["rotor_clearance_min"], 2)); mac("TurnEnv", f(tr["min_human_distance"], 2))
        mac("TurnTrack", f(1000 * tr["e_track_max_all"], 0)); mac("TurnLag", f(1000 * tr["e_lag_rms"], 0))
        mac("TurnSat", f(100 * tr["saturation_fraction"], 1))
        mac("TrackMargin", f(1000 * GOV["tracking_margin"], 0))
        mac("MarginAll", "all" if all(rows[k]["margin_holds"] for k in MISSIONS) else "not all")
        mac("TrackWorst", f(1000 * max(rows[k]["e_track_max_all"] for k in MISSIONS), 0))
        mac("ClearWorst", f(min(rows[k]["rotor_clearance_min"] for k in MISSIONS), 2))
        br = Fl["wind"]["breeze"]
        mac("BreezeLag", f(1000 * br["e_lag_rms"], 0)); mac("BreezeTilt", f(br["screen_tilt_max_deg"], 2))
        mac("BreezeLean", f(br["tilt_max_deg"], 0))
        mac("Twindow", f(SIM["t_window"], 0)); mac("Settle", f(SIM["settle"], 0))
        mac("Dt", f(1000 * SIM["dt"], 0)); mac("Seed", str(SIM["seed"]))

    Cp = R.get("coupled")
    if Cp and D:
        rows = Cp["rows"]
        names = dict(standing="standing", pacing="pacing", walk_loop="walking a loop",
                     jogging="jogging", turnaround="turning around")
        T = []
        for k in COUPLED:
            s = rows[k]
            T.append("%s & \\SI{%s}{\\kilo\\gram} & \\SI{%s}{\\gram} & $%+.0f$\\,g & \\SI{%s}{\\watt} & %s & %d/%d & \\SI{%s}{\\percent} & \\SI{%s}{\\metre} & %s\\\\"
                     % (names[k], f(s["m"], 3), f(1000 * s["m_bat"], 0),
                        1000 * (s["m_bat"] - D["m_bat"]), f(s["P_mean"], 1),
                        f(s["overhead"], 3), s["iterations"], s["sim_calls"],
                        f(100 * s["reserve_end"], 2), f(s["rotor_clearance_min"], 2),
                        "yes" if s["feasible"] else "no"))
        write("tab_coupled_rows.tex", T)
        tr, st = rows["turnaround"], rows["standing"]
        mac("CoupTurnBat", f(1000 * (tr["m_bat"] - D["m_bat"]), 0))
        mac("CoupTurnGross", f(1000 * (tr["m"] - D["m"]), 0))
        mac("CoupTurnKappa", f(tr["overhead"], 3))
        mac("CoupTurnUp", f(100 * (tr["m"] / D["m"] - 1), 1))
        mac("CoupStandBat", f(1000 * (st["m_bat"] - D["m_bat"]), 1))
        ordinary = [rows[k]["m_bat"] - D["m_bat"] for k in ("pacing", "walk_loop", "jogging")]
        mac("CoupOrdLo", f(1000 * min(ordinary), 0)); mac("CoupOrdHi", f(1000 * max(ordinary), 0))
        js = [rows[k]["Jyy"] for k in COUPLED]
        mac("CoupJlo", f(min(js), 4)); mac("CoupJhi", f(max(js), 4))
        mac("CoupItLo", str(min(rows[k]["iterations"] for k in COUPLED)))
        mac("CoupItHi", str(max(rows[k]["iterations"] for k in COUPLED)))
        mac("CoupSimLo", str(min(rows[k]["sim_calls"] for k in COUPLED)))
        mac("CoupSimHi", str(max(rows[k]["sim_calls"] for k in COUPLED)))
        mac("CoupAllFeasible", "all" if all(rows[k]["feasible"] for k in COUPLED) else "not all")
        # Session length on the hover-sized battery, turnaround throughout.
        Ft = R.get("flights", {}).get("rows", {}).get("turnaround")
        if Ft:
            e_use = derived(D["params"]).e_b * (1 - D["params"]["soc_reserve"])
            mac("EndTurn", f(D["m_bat"] * e_use / Ft["P_mean"] / 60, 1))

    Cl = R.get("clearance")
    if Cl:
        labels = dict(d070="$d=\\SI{0.70}{\\metre}$, no boom", d095="$d=\\SI{0.95}{\\metre}$",
                      d120="$d=\\SI{1.20}{\\metre}$", b020="boom \\SI{0.20}{\\metre}",
                      b038="boom \\SI{0.38}{\\metre}", b050="boom \\SI{0.50}{\\metre}")
        T = []
        for k in ("d070", "d095", "d120", "b020", "b038", "b050"):
            s = Cl["rows"][k]
            if not s.get("feasible"):
                T.append("%s & \\multicolumn{8}{c}{does not close}\\\\" % labels[k]); continue
            t = s["turn"]
            T.append("%s & \\SI{%s}{\\metre} & \\SI{%s}{\\kilo\\gram} & \\SI{%s}{\\watt} & %s & %s & %s & \\ang{%s} & %s & %s\\\\"
                     % (labels[k], f(s["tip_gap"], 3), f(s["m"], 3), f(s["P"], 1),
                        f(s["Jyy"], 4), f(s["alpha_pitch"], 0), f(s.get("K_R", 0.0), 0),
                        f(s["readability"]["angular_width"], 1),
                        f(s["readability"]["logical_w"], 0),
                        ("\\SI{%s}{\\metre}" % f(t["rotor_clearance_min"], 2)) if t["ok"] else "diverged"))
        write("tab_clearance_rows.tex", T)
        b38, d120 = Cl["rows"]["b038"], Cl["rows"]["d120"]
        # Macro names may contain letters only.
        names = dict(d070="dNear", d095="dMid", d120="dFar", b020="bShort",
                     b038="bMid", b050="bLong")
        for k, s in Cl["rows"].items():
            if s.get("feasible"):
                n = names[k]
                mac("Cl%sKR" % n, f(s["K_R"], 1)); mac("Cl%sSat" % n, f(100 * s["turn"]["saturation_fraction"], 1))
                mac("Cl%sClear" % n, f(s["turn"]["rotor_clearance_min"], 2))
                mac("Cl%sKp" % n, f(s["Kp"], 2)); mac("Cl%sInside" % n, f(100 * s["started_inside"], 0))
        if b38.get("feasible") and d120.get("feasible"):
            mac("BoomJratio", f(b38["Jyy"] / d120["Jyy"], 2))
            mac("BoomAlphaRatio", f(b38["alpha_pitch"] / d120["alpha_pitch"], 2))
            mac("BoomGross", f(1000 * (b38["m"] - d120["m"]), 0))
            mac("BoomEar", f(b38["at_ear"], 1)); mac("StandEar", f(d120["at_ear"], 1))
            mac("BoomTurnClear", f(b38["turn"]["rotor_clearance_min"], 2))
            mac("NearTurnClear", f(Cl["rows"]["d070"]["turn"]["rotor_clearance_min"], 2))

    Ds = R.get("displays")
    if Ds:
        T = []
        for k in DISPLAYS:
            s = Ds["rows"][k]
            rd = s["readability"]
            verdict = "closes" if (s["feasible"] and s["ok"]) else \
                ("no closure" if not s["feasible"] else "over limits")
            if s["feasible"]:
                T.append("%s & %s & %s & %s & \\SI{%s}{\\kilo\\gram} & \\SI{%s}{\\watt} & %s & %s & %s\\\\"
                         % (tex_name(s["name"]), f(rd["pixels_per_degree"], 0),
                            f(rd["logical_w"], 0), "n/a" if s["whole"] else f(s["areal"], 1),
                            f(s["m"], 3), f(s["P"], 1), f(s["free_1m"], 1),
                            f(s["at_ear"], 1), verdict))
            else:
                T.append("%s & %s & %s & %s & \\multicolumn{4}{c}{does not close} & fails\\\\"
                         % (tex_name(s["name"]), f(rd["pixels_per_degree"], 0),
                            f(rd["logical_w"], 0), "n/a" if s["whole"] else f(s["areal"], 1)))
        write("tab_displays_rows.tex", T)
        lim = Ds["config"]["limits"]
        mac("OptSpl", f(lim["spl_max"], 0)); mac("OptSpan", f(lim["span_max"], 2))
        mac("OptWash", f(lim["downwash_max"], 1)); mac("OptMass", f(lim["m_max"], 1))

    Hx = R.get("hex")
    if Hx:
        q, h = Hx["rows"]["quad"], Hx["rows"]["hex"]
        if q.get("feasible") and h.get("feasible"):
            T = []
            for label, s in (("quadrotor", q), ("hexacopter", h)):
                a = s["authority"]
                T.append("%s & \\SI{%s}{\\milli\\metre} & \\SI{%s}{\\kilo\\gram} & \\SI{%s}{\\watt} & %s & \\SI{%s}{\\metre} & %s & %s & %s & %s\\\\"
                         % (label, f(1000 * s["D"], 0), f(s["m"], 3), f(s["P"], 1), f(s["at_ear"], 1),
                            f(s["span"], 2), f(a["alpha_roll"], 0), f(a["alpha_pitch"], 0),
                            f(a["alpha_yaw"], 1), f(a["tau_roll"] / a["tau_pitch"], 2)))
            write("tab_hex_rows.tex", T)
            mac("HexGross", f(h["m"], 3)); mac("QuadGross", f(q["m"], 3))
            mac("HexPower", f(h["P"], 1)); mac("QuadPower", f(q["P"], 1))
            mac("HexEar", f(h["at_ear"], 1)); mac("QuadEar", f(q["at_ear"], 1))
            mac("HexSpan", f(h["span"], 2)); mac("QuadSpan", f(q["span"], 2))
            mac("HexPitch", f(h["authority"]["alpha_pitch"], 0))
            mac("QuadPitch", f(q["authority"]["alpha_pitch"], 0))
            mac("HexRoll", f(h["authority"]["alpha_roll"], 0))
            mac("QuadRoll", f(q["authority"]["alpha_roll"], 0))
            mac("HexYaw", f(h["authority"]["alpha_yaw"], 1))
            mac("QuadYaw", f(q["authority"]["alpha_yaw"], 1))
            mac("HexRank", str(h.get("rank_opposite_pair_removed", "n/a")))
            mac("HexD", f(1000 * h["D"], 0)); mac("QuadD", f(1000 * q["D"], 0))

    Tt = R.get("tether")
    if Tt:
        c = Tt["component"]
        mac("TethGross", f(c["m"], 3)); mac("TethPower", f(c["P_bus"], 1))
        mac("TethEar", f(Tt["at_ear"], 1)); mac("TethArea", f(c["conductor_area_mm2"], 2))
        mac("TethCurrent", f(c["current"], 2)); mac("TethMass", f(1000 * c["m_tether_total"], 0))
        mac("TethReserve", f(1000 * c["m_reserve"], 0)); mac("TethResLimit", c["reserve_limit"])
        mac("TethLen", f(Tt["length"], 0)); mac("TethV", f(Tt["V"], 0))
        mac("TethFeasible", "meets" if c["feasible"] else "fails")
        mac("TethVbus", f(c["v_bus"], 1)); mac("TethVdel", f(c["v_delivered"], 1))
        mac("TethSizedBy", c["conductor_sized_by"]); mac("TethDensity", f(c["current_density"], 1))
        mac("TethJmax", f(tether.J_MAX_A_MM2, 0))
        mac("TethAreaMin", f(1e6 * tether.A_MIN_CONDUCTOR, 2))
        lc = Tt.get("lumped") or {}
        mac("TethGaugeLimited", "yes" if lc.get("gauge_limited") else "no")

    Nf = R.get("nearfield")
    if Nf:
        labels = dict(both="Both corrections", nosurface="No surface effects",
                      nointerference="No interference", neither="Neither")
        T = []
        for k in ("both", "nosurface", "nointerference", "neither"):
            s = Nf["rows"][k]
            T.append("%s & %s & \\SI{%s}{\\kilo\\gram} & \\SI{%s}{\\watt} & \\SI{%s}{\\decibel}\\\\"
                     % (labels[k], f(s["kappa_total"], 3), f(s["m"], 3), f(s["P"], 1), f(s["spl"], 2)))
        write("tab_nearfield_rows.tex", T)
        ms = [s["m"] for s in Nf["rows"].values()]
        ss = [s["spl"] for s in Nf["rows"].values()]
        mac("NfSpanG", f(1000 * (max(ms) - min(ms)), 0)); mac("NfSpanDb", f(max(ss) - min(ss), 2))

    Pw = R.get("pwall")
    if Pw:
        T = []
        for row in Pw["rows"]:
            if row["t_max"] is None:
                tm = "unbounded"; extra = "n/a"
            else:
                tm = "\\SI{%d}{\\second}" % round(row["t_max"])
                extra = f(row["m_hat"] / row["m0"], 1) if "m_hat" in row else "unbounded"
            fold = "yes" if row["p"] < 1 else "no"
            T.append("$%.2f$ & $%.3f$ & %s & %s & %s\\\\" % (row["p"], row["q"], fold, tm, extra))
        write("tab_pwall_rows.tex", T)
        one = [r for r in Pw["rows"] if r["p"] == 1.0][0]
        mac("TlimOne", "%d" % round(one["t_sup"]))
        mac("TwallOne", "%d" % round(one["t_max"]) if one["t_max"] else "n/a")
        mac("PwallAnchor", f(Pw["anchor"], 2))
        mac("PwallTf", "%d" % round(float(P.DEFAULTS["t_f"])))

    write("macros.tex", M)


def short(v: List[str]) -> str:
    return "; ".join(x.split(" ")[0] + (" " + x.split(" ")[1] if len(x.split(" ")) > 1 else "")
                     for x in v)[:40] if v else ""


def tex_name(s: str) -> str:
    return s.replace(" inch", " in").replace(", flown whole", ", whole")


def write(name: str, lines: List[str]) -> None:
    # A row file must not end with "\\": TeX cannot follow the end of an
    # input file with a rule, so the closing rule is emitted here.
    if name.startswith("tab_"):
        lines = list(lines) + ["\\bottomrule"]
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def main(argv: List[str]) -> None:
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "results.json")
    R: Dict[str, Any] = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as fh:
            R = json.load(fh)
    todo = list(SECTIONS) if not argv else [a for a in argv if a != "emit"]
    for name in todo:
        t0 = time.time()
        print("section %s" % name, flush=True)
        SECTIONS[name](R)
        print("section %s done in %.0fs" % (name, time.time() - t0), flush=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(R, fh, indent=1, default=_json)
        with open(path, encoding="utf-8") as fh:
            R = json.load(fh)
    R["protocol"] = dict(preset=PRESET, payload=PAYLOAD, sim=SIM, governor=GOV,
                         closure=CLOSE, optimiser=OPT)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(R, fh, indent=1, default=_json)
    emit(R)
    print("written %s" % OUT)


def _json(o):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


if __name__ == "__main__":
    main(sys.argv[1:])
