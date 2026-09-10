"""Independent review calculations. Run from the repository root."""
from pathlib import Path
import sys
import json
import math
import numpy as np
from scipy.optimize import brentq, linprog, minimize_scalar

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from flyingscreen import params, payloads, components as comp, closure
from flyingscreen.dynamics import build_vehicle
from flyingscreen.control import control_authority

preset = next(x['name'] for x in params.PRESETS if x['name'].startswith('DESK-1/W'))
p = payloads.apply(params.preset_params(preset), 'pi_thinclient')
d = params.derived(p)
base = comp.close_first_principles(p)
veh = build_vehicle(p, base['m'], base['m_bat'])
reported = control_authority(veh)
authority = {}
# Normalize rotor squared speed to hover to avoid badly scaled LP constraints.
A = veh.alloc * veh.omega_hover**2
for j, name in enumerate(['roll', 'pitch', 'yaw'], start=1):
    rows = [i for i in range(4) if i != j]
    b = np.array([veh.m * veh.g if i == 0 else 0 for i in rows])
    lp = linprog(-A[j], A_eq=A[rows], b_eq=b,
                 bounds=[(0, (veh.omega_max/veh.omega_hover)**2)]*veh.N,
                 method='highs')
    assert lp.success, lp.message
    authority[name] = dict(engine_Nm=reported['tau_'+name], lp_Nm=-lp.fun,
                           ratio=-lp.fun/reported['tau_'+name],
                           wrench=(A@lp.x).tolist())

S = lambda m: 4 / (1 + math.exp(-4*(m-3)))
inner_roots = [brentq(lambda m: m-1-S(m), a, b)
               for a, b in [(1, 2), (2.5, 3.5), (4, 5)]]

def duration_limit(m):
    dry = comp._dry_mass(p, m)['dry']
    rb = comp.required_battery(p, m)
    available = m-dry
    if available < rb['m_power']:
        return -1.0
    return available*d.e_b*(1-p['soc_reserve'])/rb['P_total']

grid = np.geomspace(base['m'], 10000, 400)
values = np.array([duration_limit(m) for m in grid])
i = int(np.argmax(values))
peak = minimize_scalar(lambda m: -duration_limit(m),
                       bounds=(grid[max(i-1, 0)], grid[min(i+1,len(grid)-1)]),
                       method='bounded')
near = dict(p, t_f=-peak.fun*0.99999)
F = lambda m: m-comp._dry_mass(near,m)['dry']-comp.required_battery(near,m)['m_bat']
light = brentq(F, base['m'], peak.x)
heavy = brentq(F, peak.x, 10000)
near_result = comp.close_first_principles(near)

q = dict(p, N_rotors=6.0)
vh = build_vehicle(q, base['m'], base['m_bat'])
reduced = vh.alloc[:, [1,2,4,5]]
normalized = reduced/np.linalg.norm(reduced, axis=1, keepdims=True)

output = dict(parameters=p,
    baseline=dict(m=base['m'], m_bat=base['m_bat'], P=comp.required_battery(p,base['m'])['P_total'],
                  endurance_with_reserve_s=d.e_b*base['m_bat']*(1-p['soc_reserve'])/comp.required_battery(p,base['m'])['P_total']),
    torque_authority=authority, inner_map_counterexample_roots=inner_roots,
    near_fold=dict(t_f_s=near['t_f'], maximizing_mass=peak.x,
                   light_root=light, heavy_root=heavy,
                   light_residual=F(light), heavy_residual=F(heavy),
                   engine_result=near_result),
    opposite_pair_removed_hex_rank=int(np.linalg.matrix_rank(normalized)))
outpath = Path(__file__).with_name('checks.json')
outpath.write_text(json.dumps(output, indent=2), encoding='utf-8')
print(json.dumps({k:v for k,v in output.items() if k != 'parameters'}, indent=2), flush=True)

original = closure.simulate_flight
counter = [0]
def counted(*args, **kwargs):
    counter[0] += 1
    return original(*args, **kwargs)
closure.simulate_flight = counted
rows = []
try:
    for mission in ['standing', 'pacing', 'walk_loop', 'jogging', 'turnaround']:
        counter[0] = 0
        run = closure.close_with_simulation(p, mission_kind=mission, t_window=14,
                    governor={'v_max':2.5, 'a_max':2.5}, keep_telemetry=False)
        row = dict(mission=mission, converged=run.converged, m=run.m, m_bat=run.m_bat,
                   P_mean=run.P_mean, overhead=run.overhead, iterations=len(run.trace),
                   simulation_calls=counter[0], last_trace=run.trace[-1] if run.trace else {},
                   sim={k:v for k,v in run.sim.items() if k not in ['telemetry','trajectory','t']})
        rows.append(row)
        output['coupled_runs'] = rows
        outpath.write_text(json.dumps(output, indent=2), encoding='utf-8')
        print(json.dumps(row), flush=True)
finally:
    closure.simulate_flight = original
