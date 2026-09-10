"""
HTTP surface over the engine.

Every endpoint takes the same flat parameter dictionary, so the frontend
never has to know which layer of the model a given assumption belongs to.
The schema endpoint is what the UI builds its controls from.
"""

from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional

from fastapi import FastAPI
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import acoustics
from . import closure as closure_mod
from . import components as comp
from . import electrical
from . import payloads
from . import mission as mission_mod
from . import optimize as opt_mod
from . import sizing
from . import sweep as sweep_mod
from . import tether as tether_mod
from .control import control_authority, suggest_gains
from .dynamics import build_vehicle, hover_linearisation
from .params import (DEFAULTS, GROUPS, PRESETS, SCHEMA, clamp_params, derived)
from .simulate import simulate_flight

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")


# ---------------------------------------------------------------------------
# JSON hygiene: NaN and infinity are not valid JSON, so they become null.
# ---------------------------------------------------------------------------
def clean(obj: Any) -> Any:
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    if hasattr(obj, "tolist"):
        return clean(obj.tolist())
    return obj


def ok(payload: Any) -> JSONResponse:
    return JSONResponse(content=clean(payload))


app = FastAPI(title="Flying screen co-design engine", version="0.1.0")


# ---------------------------------------------------------------------------
# Request bodies
# ---------------------------------------------------------------------------
class Req(BaseModel):
    params: Dict[str, float] = Field(default_factory=dict)
    model: str = "fixed_area"


class AxisSpec(BaseModel):
    key: str
    lo: float
    hi: float
    n: int = 40
    log: bool = False


class SweepReq(Req):
    x: AxisSpec
    y: AxisSpec
    metric: str = "m"
    first_principles: bool = False


class SimReq(Req):
    m: Optional[float] = None
    m_bat: Optional[float] = None
    mission: str = "pacing"
    t_window: float = 12.0
    dt: float = 0.004
    seed: int = 12345
    governor: Optional[Dict[str, float]] = None


class ClosureReq(Req):
    mission: str = "pacing"
    t_window: float = 8.0
    dt: float = 0.004
    m_bat0: Optional[float] = None
    max_iter: int = 8
    settle: float = 3.0
    seed: int = 12345
    governor: Optional[Dict[str, float]] = dict(v_max=2.5, a_max=2.5)


class OptReq(Req):
    variables: Optional[List[str]] = None
    weights: Optional[Dict[str, float]] = None
    limits: Optional[Dict[str, float]] = None
    maxiter: int = 30
    popsize: int = 12
    refine_with_sim: bool = False
    mission: str = "pacing"


class ParetoReq(Req):
    variable: str = "D_rotor"
    n: int = 24
    lo: Optional[float] = None
    hi: Optional[float] = None


def P(req: Req) -> Dict[str, float]:
    q = dict(DEFAULTS)
    q.update(req.params or {})
    return clamp_params(q)


# ---------------------------------------------------------------------------
# Schema and defaults
# ---------------------------------------------------------------------------
@app.get("/api/schema")
def get_schema() -> JSONResponse:
    return ok(dict(
        schema=SCHEMA, groups=GROUPS, defaults=DEFAULTS,
        missions=mission_mod.MISSIONS,
        metrics=sweep_mod.METRICS,
        models=["fixed_area", "constant_dl", "power_law"],
        design_vars=opt_mod.DESIGN_VARS,
        default_vars=opt_mod.DEFAULT_VARS,
        weights=opt_mod.DEFAULT_WEIGHTS,
        limits=opt_mod.DEFAULT_LIMITS,
        levers=sizing.LEVERS,
        displays=[dict(c, readability=payloads.readability(c, 0.70))
                  for c in payloads.CATALOGUE],
        absorptions=acoustics.ROOM_ABSORPTION,
        presets=[dict(name=p["name"], note=p["note"],
                      model=p.get("model", "fixed_area"),
                      params=clamp_params({**DEFAULTS, **p["values"]}))
                 for p in PRESETS],
    ))


# ---------------------------------------------------------------------------
# Layers 1 and 2
# ---------------------------------------------------------------------------
@app.post("/api/design")
def post_design(req: Req) -> JSONResponse:
    p = P(req)
    d = derived(p)
    dp = sizing.solve(p, req.model)
    fp = comp.close_first_principles(p)
    out = dict(
        params=p,
        derived=dict(A=d.A, A_rotor=d.A_rotor, e_b=d.e_b,
                     e_b_mission=d.e_b * (1.0 - float(p["soc_reserve"])),
                     m_fix=d.m_fix, P_aux=d.P_aux, A_screen=d.A_screen),
        design=dp.to_dict(),
        curve=sizing.closure_curve(p, req.model),
        wall=sizing.payload_wall(p),
        sensitivities=sizing.sensitivities(p),
        first_principles=fp,
        alternatives=dict(
            fixed_area=sizing.solve_fixed_area(p).to_dict(),
            constant_dl=sizing.solve_constant_dl(p).to_dict(),
            power_law=sizing.solve_power_law(p).to_dict(),
        ),
    )
    return ok(out)


@app.post("/api/curve")
def post_curve(req: Req) -> JSONResponse:
    return ok(sizing.closure_curve(P(req), req.model))


@app.post("/api/wall")
def post_wall(req: Req) -> JSONResponse:
    p = P(req)
    return ok(dict(wall=sizing.payload_wall(p),
                   sensitivities=sizing.sensitivities(p),
                   boundary=sweep_mod.payload_boundary(p)))


class FPReq(Req):
    m0: Optional[float] = None
    iters: int = 150


@app.post("/api/fixed_point")
def post_fixed_point(req: FPReq) -> JSONResponse:
    return ok(sizing.fixed_point_trace(P(req), m0=req.m0, iters=req.iters,
                                       model=req.model))


# ---------------------------------------------------------------------------
# Layer 3
# ---------------------------------------------------------------------------
class CompReq(Req):
    m: Optional[float] = None


@app.post("/api/components")
def post_components(req: CompReq) -> JSONResponse:
    p = P(req)
    fp = comp.close_first_principles(p)
    m = req.m if req.m is not None else (fp["m"] if fp["feasible"] else 2.0)
    b = comp.breakdown(p, m)
    N = float(p["N_rotors"])
    T_hover = m * float(p["g"]) / N
    return ok(dict(closure=fp, breakdown=b.to_dict(),
                   optimal_tip_speed=comp.optimal_tip_speed(p, T_hover),
                   m_used=m))


class SpecReq(Req):
    m: Optional[float] = None
    m_bat: Optional[float] = None
    mission: str = "pacing"
    t_window: float = 14.0
    fly: bool = True
    governor: Optional[Dict[str, float]] = dict(v_max=2.5, a_max=2.5)


@app.post("/api/spec")
def post_spec(req: SpecReq) -> JSONResponse:
    """
    The whole sheet for one machine, computed rather than transcribed:
    geometry, mass build-up, rotor operating point, the electrical chain,
    control authority, and the same design flown across every mission.
    """
    p = P(req)
    d = derived(p)
    fp = comp.close_first_principles(p)
    if not fp["feasible"]:
        return ok(dict(feasible=False, reason=fp["reason"]))
    m = req.m if req.m is not None else fp["m"]
    m_bat = req.m_bat if req.m_bat is not None else fp["m_bat"]
    b = comp.breakdown(p, m, m_bat=m_bat).to_dict()

    veh = build_vehicle(p, m, m_bat)
    auth = control_authority(veh)
    gains = suggest_gains(veh, a_max=(req.governor or {}).get("a_max", 2.5))
    chain = electrical.electrical_chain(p, m, m_bat)

    hov = comp.propulsion(p, m).rotor_hover
    acou = acoustics.report(p, hov, float(p["N_rotors"]), comp._SPL_REF)
    surf = comp.surface_effects(p)
    inter = comp.interference_factor(p)
    recirc = comp.recirculation(p, d.A, hov.v_induced)
    panel = dict(w=float(p["screen_w"]), h=float(p["screen_h"]),
                 m_screen=float(p["m_screen"]),
                 px_w=1920, px_h=1080)
    read = payloads.readability(panel, float(p["standoff"]))

    flights = []
    if req.fly:
        for k in mission_mod.MISSIONS:
            s = simulate_flight(p, m, m_bat=m_bat, mission_kind=k,
                                t_window=req.t_window, governor=req.governor)
            flights.append(dict(
                mission=k, ok=s.ok, reason=s.reason, P_mean=s.P_mean,
                overhead=s.overhead, e_lag_rms=s.e_lag_rms, e_lag_max=s.e_lag_max,
                e_track_rms=s.e_track_rms, screen_tilt_max_deg=s.screen_tilt_max_deg,
                tilt_max_deg=s.tilt_max_deg,
                saturation_fraction=s.saturation_fraction,
                ref_accel_max=s.ref_accel_max, lam_needed=s.lam_needed,
                rotor_clearance_min=s.rotor_clearance_min,
                min_human_distance=s.min_human_distance,
                e_track_max_all=s.e_track_max_all, margin_holds=s.margin_holds,
                soc_end=s.soc_end, constraints_ok=s.constraints_ok,
                violations=s.violations, warnings=s.warnings,
            ))
        worst = max((f["P_mean"] for f in flights if f["ok"]), default=b["P_total"])
        session_worst = m_bat * d.e_b * (1.0 - float(p["soc_reserve"])) / max(worst, 1e-9)
    else:
        worst, session_worst = b["P_total"], b["endurance"]

    # Same submodels as the battery closure, so the two are comparable.
    teth = tether_mod.close_tethered_first_principles(p)

    return ok(dict(
        feasible=True, m=m, m_bat=m_bat, breakdown=b,
        derived=dict(A=d.A, m_fix=d.m_fix, P_aux=d.P_aux, e_b=d.e_b,
                     e_b_mission=d.e_b * (1.0 - float(p["soc_reserve"]))),
        authority=auth, suggested_gains=gains, electrical=chain,
        acoustics=acou, surface=surf, interference=inter,
        recirculation=recirc, readability=read,
        flights=flights, worst_power=worst, session_worst=session_worst,
        session_nominal=b["endurance"], tether=teth,
        governor=req.governor,
        inertia=dict(Jxx=veh.J[0][0], Jyy=veh.J[1][1], Jzz=veh.J[2][2]),
        vehicle=dict(span=veh.span, arm_L=veh.arm_L, N=veh.N,
                     omega_hover=veh.omega_hover, omega_max=veh.omega_max,
                     E_max=veh.E_max),
    ))


# ---------------------------------------------------------------------------
# Sweeps
# ---------------------------------------------------------------------------
@app.post("/api/sweep")
def post_sweep(req: SweepReq) -> JSONResponse:
    p = P(req)
    grid = sweep_mod.sweep2d(p, req.x.model_dump(), req.y.model_dump(),
                             metric=req.metric, model=req.model,
                             first_principles=req.first_principles)
    if req.x.key == "t_f" and req.y.key == "m_screen":
        grid["analytic_boundary"] = sweep_mod.payload_boundary(
            p, t_lo=req.x.lo, t_hi=req.x.hi)
    return ok(grid)


@app.post("/api/boundary")
def post_boundary(req: Req) -> JSONResponse:
    return ok(sweep_mod.payload_boundary(P(req)))


@app.post("/api/scaling")
def post_scaling(req: Req) -> JSONResponse:
    return ok(sweep_mod.scaling_sweep(P(req)))


# ---------------------------------------------------------------------------
# Layers 4, 5 and 6
# ---------------------------------------------------------------------------
@app.post("/api/simulate")
def post_simulate(req: SimReq) -> JSONResponse:
    p = P(req)
    m, m_bat = req.m, req.m_bat
    if m is None:
        fp = comp.close_first_principles(p)
        if not fp["feasible"]:
            return ok(dict(ok=False, reason=fp["reason"]))
        m, m_bat = fp["m"], fp["m_bat"]
    res = simulate_flight(p, m, m_bat=m_bat, mission_kind=req.mission,
                          t_window=req.t_window, dt=req.dt, seed=req.seed,
                          governor=req.governor)
    veh = build_vehicle(p, m, m_bat)
    out = res.to_dict()
    out["authority"] = control_authority(veh)
    out["suggested_gains"] = suggest_gains(
        veh, a_max=(req.governor or {}).get("a_max", 2.5))
    out["vehicle"] = dict(m=veh.m, span=veh.span, arm_L=veh.arm_L,
                          N=veh.N, k_T=veh.k_T, k_Q=veh.k_Q, k_P=veh.k_P,
                          omega_hover=veh.omega_hover, omega_max=veh.omega_max,
                          E_max=veh.E_max, J=veh.J.tolist(),
                          rotor_pos=veh.rotor_pos.tolist(),
                          spin=veh.spin.tolist(),
                          disk_loading=veh.disk_loading,
                          breakdown=veh.breakdown)
    out["m"] = m
    out["m_bat"] = m_bat
    return ok(out)


class LinReq(Req):
    m: Optional[float] = None


@app.post("/api/linearise")
def post_linearise(req: LinReq) -> JSONResponse:
    p = P(req)
    m = req.m
    if m is None:
        fp = comp.close_first_principles(p)
        if not fp["feasible"]:
            return ok(dict(ok=False, reason=fp["reason"]))
        m = fp["m"]
    veh = build_vehicle(p, m)
    lin = hover_linearisation(veh)
    lin["m"] = m
    return ok(lin)


@app.post("/api/closure")
def post_closure(req: ClosureReq) -> JSONResponse:
    p = P(req)
    run = closure_mod.close_with_simulation(
        p, mission_kind=req.mission, t_window=req.t_window, dt=req.dt,
        m_bat0=req.m_bat0, max_iter=req.max_iter, governor=req.governor,
        settle=req.settle, seed=req.seed)
    return ok(run.to_dict())


# ---------------------------------------------------------------------------
# Tether and optimisation
# ---------------------------------------------------------------------------
@app.post("/api/tether")
def post_tether(req: Req) -> JSONResponse:
    return ok(tether_mod.compare(P(req)))


@app.post("/api/optimize")
def post_optimize(req: OptReq) -> JSONResponse:
    return ok(opt_mod.optimise(P(req), variables=req.variables,
                               weights=req.weights, limits=req.limits,
                               maxiter=req.maxiter, popsize=req.popsize,
                               refine_with_sim=req.refine_with_sim,
                               mission_kind=req.mission))


@app.post("/api/pareto")
def post_pareto(req: ParetoReq) -> JSONResponse:
    return ok(opt_mod.pareto(P(req), var=req.variable, n=req.n,
                             lo=req.lo, hi=req.hi))


# ---------------------------------------------------------------------------
# Static frontend
# ---------------------------------------------------------------------------
NO_CACHE = {"Cache-Control": "no-store, must-revalidate", "Pragma": "no-cache"}


class NoCacheStatic(StaticFiles):
    """
    Serve the frontend without caching.

    This is a tool that is edited while it is open. A browser holding a stale
    index.html against a fresh app.js fails in a way that looks like the app
    is broken rather than out of date, which costs far more than the requests
    the cache would have saved.
    """

    def is_not_modified(self, response_headers, request_headers) -> bool:  # noqa: D102
        return False

    async def get_response(self, path: str, scope):
        response = await super().get_response(path, scope)
        response.headers.update(NO_CACHE)
        return response


if os.path.isdir(WEB_DIR):
    app.mount("/app", NoCacheStatic(directory=WEB_DIR, html=True), name="web")

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(os.path.join(WEB_DIR, "index.html"), headers=NO_CACHE)
