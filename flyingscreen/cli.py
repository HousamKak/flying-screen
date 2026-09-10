"""
Command line front end.

    python -m flyingscreen.cli design --m_screen 0.25 --t_f 1800
    python -m flyingscreen.cli wall --t_f 3600
    python -m flyingscreen.cli sim --mission turnaround --t_window 15
    python -m flyingscreen.cli closure --m_screen 0.3
    python -m flyingscreen.cli optimize
    python -m flyingscreen.cli tether
    python -m flyingscreen.cli serve
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Any, Dict

from . import closure as closure_mod
from . import components as comp
from . import optimize as opt_mod
from . import sizing
from . import sweep as sweep_mod
from . import tether as tether_mod
from .params import DEFAULTS, SCHEMA, clamp_params
from .simulate import simulate_flight


def add_param_args(ap: argparse.ArgumentParser) -> None:
    g = ap.add_argument_group("model parameters")
    for e in SCHEMA:
        g.add_argument("--" + e["key"], type=float, default=None,
                       help="%s [%s], default %g" % (e["label"], e["unit"],
                                                     float(e["default"])))


def collect(args: argparse.Namespace) -> Dict[str, float]:
    p = dict(DEFAULTS)
    for e in SCHEMA:
        v = getattr(args, e["key"], None)
        if v is not None:
            p[e["key"]] = v
    return clamp_params(p)


def _f(v: Any, fmt: str = "%.4g") -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "--"
    if isinstance(v, (int, float)):
        return fmt % v
    return str(v)


def cmd_design(args) -> int:
    p = collect(args)
    dp = sizing.solve(p, args.model)
    c = dp.coeffs
    print("model: %s" % args.model)
    print("  alpha = %s   beta = %s   M0 = %s kg" %
          (_f(c.get("alpha")), _f(c.get("beta")), _f(c.get("M0"))))
    if not dp.feasible:
        print("  INFEASIBLE: %s" % dp.reason)
        w = sizing.payload_wall(p)
        print("  wall: M0_max = %s kg at m* = %s kg (you are asking for %s kg)"
              % (_f(w.get("M0_max")), _f(w.get("m_star")), _f(w.get("M0"))))
        print("  the screen budget at this endurance is %s kg"
              % _f(w.get("m_screen_max")))
        return 1
    print("  gross mass       %s kg" % _f(dp.m))
    print("    fixed          %s kg" % _f(dp.m_fix))
    print("    structure      %s kg" % _f(dp.m_str))
    print("    propulsion     %s kg" % _f(dp.m_prop))
    print("    battery        %s kg  (%s limited)" % (_f(dp.m_bat), dp.battery_limit))
    print("  hover power      %s W  (aux %s W)" % (_f(dp.P_total), _f(dp.P_aux)))
    print("  disk loading     %s N/m2   induced %s m/s" %
          (_f(dp.disk_loading), _f(dp.v_induced)))
    print("  rotor diameter   %s m over %d rotors" % (_f(dp.D_rotor), int(p["N_rotors"])))
    print("  sizing stability d(m_str+m_prop+m_bat)/dm = %s (needs < 1)" % _f(dp.dmdm))
    if dp.closure.get("m_heavy"):
        print("  second root      %s kg (the unstable upper branch)" % _f(dp.closure["m_heavy"]))
    for w in dp.warnings:
        print("  ! %s" % w)

    fp = comp.close_first_principles(p)
    print("\nfirst principles:")
    if not fp["feasible"]:
        print("  INFEASIBLE: %s" % fp["reason"])
    else:
        b = fp["breakdown"]
        print("  gross mass %s kg   frame %s   propulsion %s   battery %s"
              % (_f(b["m"]), _f(b["m_frame"]), _f(b["m_prop"]), _f(b["m_bat"])))
        print("  implied f_s = %s (assumed %s), S_m = %s N/kg (assumed %s), FM = %s (assumed %s)"
              % (_f(b["f_s_implied"]), _f(p["f_s"]), _f(b["S_m_implied"]),
                 _f(p["S_m"]), _f(b["FM_implied"]), _f(p["FM"])))
        print("  span %s m   downwash %s m/s   SPL at 1 m %s dB   arm mode %s Hz"
              % (_f(b["span"]), _f(b["downwash"]), _f(b["spl_1m"]), _f(b["arm"]["f_bending"])))
        for w in b["warnings"]:
            print("  ! %s" % w)
    return 0


def cmd_wall(args) -> int:
    p = collect(args)
    w = sizing.payload_wall(p)
    print("M0            = %s kg" % _f(w.get("M0")))
    print("M0_max        = %s kg   at m* = %s kg" % (_f(w.get("M0_max")), _f(w.get("m_star"))))
    print("screen budget = %s kg" % _f(w.get("m_screen_max")))
    print("utilisation   = %s" % _f(w.get("utilisation")))
    print("\nlogarithmic sensitivities of M0_max:")
    for k, v in sorted(sizing.sensitivities(p).items(), key=lambda kv: -abs(kv[1])):
        print("   d ln M0_max / d ln %-12s = %+.2f" % (k, v))
    print("\nwall against endurance:")
    b = sweep_mod.payload_boundary(p, n=9)
    for t, mm in zip(b["t"], b["m_screen_max"]):
        print("   t = %8.0f s   screen budget %s kg" % (t, _f(mm)))
    return 0


def cmd_sim(args) -> int:
    p = collect(args)
    fp = comp.close_first_principles(p)
    if not fp["feasible"]:
        print("cannot size a vehicle: %s" % fp["reason"])
        return 1
    r = simulate_flight(p, fp["m"], m_bat=fp["m_bat"], mission_kind=args.mission,
                        t_window=args.t_window, dt=args.dt)
    if not r.ok:
        print("simulation failed: %s" % r.reason)
        return 1
    print("mission %s, %.0f s window, vehicle %.3f kg" % (args.mission, args.t_window, fp["m"]))
    print("  mean power        %s W  (steady hover %s W, so the mission costs %sx)"
          % (_f(r.P_mean), _f(r.P_hover_ref), _f(r.overhead)))
    print("  closed-form layer assumes %s W for the same hover" % _f(r.P_ideal_lumped))
    print("  tracking error    rms %s m, max %s m" % (_f(r.e_track_rms), _f(r.e_track_max)))
    print("  airframe tilt     rms %s deg, max %s deg" % (_f(r.tilt_rms_deg), _f(r.tilt_max_deg)))
    print("  screen tilt       rms %s deg, max %s deg"
          % (_f(r.screen_tilt_rms_deg), _f(r.screen_tilt_max_deg)))
    print("  motor saturation  %s percent of the time" % _f(100 * r.saturation_fraction))
    print("  endurance         %s s at this power (asked for %s s)"
          % (_f(r.endurance_est), _f(p["t_f"])))
    for w in r.warnings:
        print("  ! %s" % w)
    return 0


def cmd_closure(args) -> int:
    p = collect(args)
    run = closure_mod.close_with_simulation(
        p, mission_kind=args.mission, t_window=args.t_window,
        m_bat0=args.m_bat0, max_iter=args.max_iter, keep_telemetry=False)
    print("simulation-in-the-loop closure: %s" % run.status)
    if run.reason:
        print("  %s" % run.reason)
    print("  %-4s %-10s %-14s %-10s" % ("it", "m_bat", "m_bat required", "P_mean"))
    for t in run.trace:
        print("  %-4d %-10s %-14s %-10s" % (t["iteration"], _f(t["m_bat"]),
                                            _f(t["m_bat_required"]), _f(t["P_mean"])))
    print("  final gross mass %s kg, battery %s kg, mean power %s W (overhead %sx)"
          % (_f(run.m), _f(run.m_bat), _f(run.P_mean), _f(run.overhead)))
    for w in run.warnings:
        print("  ! %s" % w)
    return 0 if run.converged else 1


def cmd_optimize(args) -> int:
    p = collect(args)
    r = opt_mod.optimise(p, maxiter=args.maxiter, popsize=args.popsize,
                         refine_with_sim=args.sim, mission_kind=args.mission)
    b, base = r["best"], r["baseline"]
    print("variables: %s" % ", ".join(r["variables"]))
    print("  baseline  J=%s  m=%s kg  P=%s W  span=%s m  SPL=%s dB"
          % (_f(base["J"]), _f(base["m"]), _f(base["P_total"]),
             _f(base["span"]), _f(base["spl"])))
    print("  optimum   J=%s  m=%s kg  P=%s W  span=%s m  SPL=%s dB  downwash=%s m/s"
          % (_f(b["J"]), _f(b["m"]), _f(b["P_total"]), _f(b["span"]),
             _f(b["spl"]), _f(b["downwash"])))
    for k, v in b["x"].items():
        print("    %-12s %s" % (k, _f(v)))
    if b["violations"]:
        print("  constraints still violated: %s" % "; ".join(b["violations"]))
    if "sim" in r:
        s = r["sim"]
        print("  simulated: P=%s W  tracking rms %s m  saturation %s percent"
              % (_f(s["P_mean"]), _f(s["e_track_rms"]), _f(100 * s["saturation_fraction"])))
    return 0


def cmd_tether(args) -> int:
    p = collect(args)
    cmpres = tether_mod.compare(p)
    bat, te = cmpres["battery"], cmpres["tether"]
    print("battery version: %s" % ("feasible" if bat["feasible"] else "INFEASIBLE: " + bat["reason"]))
    if bat["feasible"]:
        print("   m = %s kg, battery %s kg, power %s W, endurance %s s"
              % (_f(bat["m"]), _f(bat["m_bat"]), _f(bat["P_total"]), _f(p["t_f"])))
    print("tethered version: %s" % ("feasible" if te["feasible"] else "INFEASIBLE: " + te.get("reason", "")))
    if te.get("feasible"):
        print("   m = %s kg, bus power %s W, source %s W, current %s A"
              % (_f(te["m"]), _f(te["P_bus"]), _f(te["P_source"]), _f(te["current"])))
        print("   conductor %s mm2 per leg (%s mm), tether %s kg total, %s kg carried"
              % (_f(te["conductor_area_mm2"]), _f(te["conductor_dia_mm"]),
                 _f(te["m_tether_total"]), _f(te["m_tether_carried"])))
        print("   endurance: %s" % te["endurance"])
    print("\n%s" % cmpres["note"])
    return 0


def cmd_serve(args) -> int:
    import uvicorn
    uvicorn.run("flyingscreen.api:app", host=args.host, port=args.port,
                reload=args.reload)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="flyingscreen",
                                 description="Flying screen co-design engine")
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("design", help="solve the mass closure")
    d.add_argument("--model", default="fixed_area",
                   choices=["fixed_area", "constant_dl", "power_law"])
    add_param_args(d); d.set_defaults(func=cmd_design)

    w = sub.add_parser("wall", help="feasibility wall and sensitivities")
    add_param_args(w); w.set_defaults(func=cmd_wall)

    s = sub.add_parser("sim", help="run the 6-DOF simulator")
    s.add_argument("--mission", default="pacing")
    s.add_argument("--t_window", type=float, default=12.0)
    s.add_argument("--dt", type=float, default=0.004)
    add_param_args(s); s.set_defaults(func=cmd_sim)

    c = sub.add_parser("closure", help="battery closure with the simulator in the loop")
    c.add_argument("--mission", default="pacing")
    c.add_argument("--t_window", type=float, default=8.0)
    c.add_argument("--m_bat0", type=float, default=None)
    c.add_argument("--max_iter", type=int, default=8)
    add_param_args(c); c.set_defaults(func=cmd_closure)

    o = sub.add_parser("optimize", help="search the design space")
    o.add_argument("--maxiter", type=int, default=30)
    o.add_argument("--popsize", type=int, default=12)
    o.add_argument("--sim", action="store_true")
    o.add_argument("--mission", default="pacing")
    add_param_args(o); o.set_defaults(func=cmd_optimize)

    t = sub.add_parser("tether", help="compare battery and tethered closures")
    add_param_args(t); t.set_defaults(func=cmd_tether)

    sv = sub.add_parser("serve", help="run the web tool")
    sv.add_argument("--host", default="127.0.0.1")
    sv.add_argument("--port", type=int, default=8000)
    sv.add_argument("--reload", action="store_true")
    sv.set_defaults(func=cmd_serve)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
