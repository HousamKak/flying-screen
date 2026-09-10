# Flying screen

A co-design engine and tool for the flying-screen problem: a display that hovers
beside you while you work, so you can stand or move instead of sitting.

The question the whole thing exists to answer is not "can a multirotor fly", it
is:

> Given a screen, a mission and the technology available, does any feasible
> aircraft exist at all, and if so, what rotor size, battery, motors, geometry
> and controller make it the best one physics allows?

Everything here follows the derivation in `Cleaned derivation.txt` and
`Initial Conversation.txt`. Nothing is a black box: every number in the tool
traces back to an equation, and the test suite checks the code against the
closed-form theory rather than against itself.

---

## Running it

```
pip install -r requirements.txt
python -m flyingscreen.cli serve --port 8077
```

then open <http://127.0.0.1:8077>. The port defaults to 8000; pick another if
something else already has it.

The command line covers the same ground without the browser:

```
python -m flyingscreen.cli design   --m_screen 0.25 --t_f 1800
python -m flyingscreen.cli wall     --t_f 3600
python -m flyingscreen.cli sim      --mission turnaround --t_window 15
python -m flyingscreen.cli closure  --m_screen 0.3
python -m flyingscreen.cli optimize
python -m flyingscreen.cli tether
python -m pytest tests -q
```

Every model parameter is available as a `--flag` on every subcommand.

---

## The seven layers

| Layer | Module | What it decides |
| --- | --- | --- |
| 1. Momentum theory | `sizing.py` | hover energy |
| 2. Mass closure | `sizing.py` | whether the design can exist |
| 3. Structure and components | `components.py` | frame mass, motor mass, real figure of merit |
| 4. Newton-Euler dynamics | `dynamics.py` | how the body moves |
| 5. Motor and battery states | `dynamics.py` | actuator response and energy use |
| 6. Control | `control.py`, `mission.py`, `simulate.py` | stability and following a person |
| 7. Optimisation | `optimize.py`, `sweep.py` | the best machine the constraints allow |

`closure.py` is the loop that ties layer 6 back into layer 2, and `tether.py`
is the version of the problem with the energy taken off the aircraft.

---

## The central equations, and where they live

**Momentum theory.** A rotor disk of total area `A` accelerating air downward
produces `T = 2 rho A v_i^2`, so the induced velocity and ideal hover power are

```
v_i = sqrt(T / (2 rho A))            P_i = T v_i = T^1.5 / sqrt(2 rho A)
```

With a figure of merit and electrical efficiency,

```
P_prop = (m g)^1.5 / (FM eta sqrt(2 rho A))
```

`sizing._power_coefficient`, `components.rotor_point`.

**Mass closure.** Splitting mass into fixed, structural, propulsive and battery,
with `m_str = f_s m`, `m_prop = (lambda g / S_m) m` and
`m_bat = P_total t_f / e_b`, everything collapses to

```
alpha m - beta m^1.5 = M0

alpha = 1 - f_s - lambda g / S_m
beta  = t_f g^1.5 / (e_b FM eta sqrt(2 rho A))
M0    = m_fix + P_aux t_f / e_b
```

`sizing.coefficients`, `sizing.solve_power_closure`.

**The fold.** The left side is not monotonic. Differentiating,

```
m*     = 4 alpha^2 / (9 beta^2)
M0_max = 4 alpha^3 / (27 beta^2)
```

Below `M0_max` there are two roots, the light one being the design and the heavy
one an unstable branch. At `M0_max` they merge. Above it there is no
equilibrium mass at all: a saddle-node bifurcation, not a heavy aeroplane.
`sizing.payload_wall`, and the curve drawn on the Closure tab.

**Why endurance is brutal.** Since `beta` is proportional to `t_f` and the wall
goes as `1 / beta^2`,

```
M0_max  =  8 alpha^3 rho A e_b^2 FM^2 eta^2 / (27 g^3 t_f^2)
```

so four times the flight time costs sixteen times the payload. The tool
computes these exponents numerically in `sizing.sensitivities` and the test
suite asserts they come out at exactly -2, +2, +2, +2, +2 (diameter), +1
(rotor count) and +1 (density).

**The assumption hidden in `m^1.5`.** It assumes `A` is fixed. If disk area
grows with mass as `A ~ m^p` then

```
P ~ m^((3 - p) / 2)
```

and the fold exists only while `p < 1`. At constant disk loading, `p = 1`,
power is linear in mass, the closure becomes

```
m = M0 / (1 - f_s - gamma_m - gamma_b)
```

and feasibility is only `f_s + gamma_m + gamma_b < 1`. The wall does not
disappear so much as move: what becomes unreasonable is the rotor diameter.
`sizing.solve_power_law`, `sizing.solve_constant_dl`, `sweep.scaling_sweep`,
and the Rotor scaling tab.

**Flight.** Once the machine exists, `m` is constant and the real differential
equations begin:

```
rdot = v
m vdot = T R e3 - m g e3 + F_D + F_dist
J wdot + w x J w = tau
qdot = 0.5 Omega(w) q
tau_m Omegadot_i = Omega_cmd_i - Omega_i
Edot = -(sum_i k_P Omega_i^3 / eta + P_aux)
J_g thddot_g + b_g thdot_g = tau_g - tau_wind
```

`dynamics.derivatives`, integrated with fixed-step RK4 in `simulate.py`.

**Screen drag.** The screen is a flat plate, so its drag depends on
orientation, and because it hangs below the centre of mass that drag becomes a
pitching moment. This is the coupling that makes a flying display different
from a drone: attitude feeds translation and translation feeds back into
attitude. `dynamics.aero_forces`.

**Control.** Position to velocity to attitude to body rate to rotor speed. The
outer loop asks for an acceleration, that becomes a force, the force direction
becomes the desired body z axis, and a geometric controller on SO(3) drives the
body there. A separate PD loop drives the gimbal so the screen stays upright
while the airframe tilts. `control.CascadedController`.

**Mission energy.** The honest version of `E = P t`:

```
E_mission = integral of P(x(t), u(t), p) dt
```

The simulator runs a representative window at full fidelity, measures the mean
electrical power once the transient has decayed, and extrapolates. That mean is
always larger than the ideal hover power, by an amount the model computes
rather than assumes: 1.00x standing still, about 1.05x when the person turns
around with the reference governor letting the screen cut the corner. The
power law behind it is calibrated at hover and has no inflow correction, so
these are near-hover figures, not a model of climbs or aggressive manoeuvres.

**The battery loop.** Guess a battery, close the vehicle around it, fly the
mission, integrate the power, ask what battery that needed, repeat. The
residual pair being driven to zero is

```
R1 = m - (m_fix + m_str + m_prop + m_bat)
R2 = m_bat - max(E_mission / (e_b (1 - reserve)), P_peak / p_b)
```

`R1` is solved at every step by the inner iteration, so the outer loop only
drives `R2` and each update costs one simulation. Once `R2` is within
tolerance the battery is rounded up and the design is flown again; the run
counts as converged only when that verification flight needs no more battery
than it carries, and the final flight's reserve, peak power, rotor clearance
and screen attitude are reported separately. Converging closes the mass and
energy balance; it does not by itself make the flight acceptable.
`closure.close_with_simulation`, and the Energy loop tab.

**The tether.** Taking the battery off the aircraft removes the mission time
from the closure. What replaces it is a conductor,

```
m_conductor = 4 rho_e rho_m L^2 P / (drop V^2)
```

sized by the largest of voltage drop, ampacity and handling. Endurance stops
being a design variable. The fold does not vanish in general: a carried,
power-sized conductor keeps the `m^1.5` term with a much smaller coefficient,
so the fold moves far away rather than disappearing; it disappears only when
the carried cable mass stops depending on power. `tether.py`.

---

## What the model says at the defaults

The tool opens on a 13-inch panel, four 40 cm rotors, 250 Wh/kg cells, a 10 %
landing reserve and a one-hour session, and reports **no solution**. That is
not a bug, it is the result: at those assumptions the wall is 0.60 kg of
effective fixed load and the design is asking for 0.77 kg. (The reserve is
now part of the usable specific energy everywhere in the engine, which is why
these numbers are lower than an earlier version of this file quoted.)

At eight hours the wall falls to about 9 grams.

The Closure tab offers the four escapes the algebra allows (shorter session,
bigger rotors, lighter screen, or a wire) and the rest of the tool is for
finding out what each one actually costs. The `Starting point` menu in the
sidebar drops you on either side of the wall directly.

Three results worth knowing before you start turning dials.

**The first-principles layer and the lumped layer disagree, on purpose.** At
the defaults the closed form says no solution while the component build-up
closes at 3.6 kg. The difference is that `f_s = 0.20` and `S_m = 120 N/kg`
are pessimistic for this machine while `FM = 0.65` is optimistic. The
component version is not good news: it arrives with a 47% battery fraction, a
10.7 m/s downwash at head height and about 72 dB at one metre. Both numbers
are shown side by side on the Closure tab rather than one being picked.

**Rotor diameter has a real interior optimum.** Induced power wants a big
disk, while motor torque, propeller mass, guard rings and arm length all want
a small one. At a one-hour mission with a 0.35 kg screen the minimum sits near
0.45 to 0.55 m per rotor, and the machine gets worse in both directions. This
only appears once tip speed is set by blade loading rather than held fixed,
which is the `Set RPM from blade loading` switch.

**The hardest thing you can ask is not walking, it is turning around.** Pacing
at 1.4 m/s costs 0.3% more power than hovering. Jogging in a loop costs 4%.
But a person turning 180 degrees in about a second, with the screen expected
to stay in front of their face, demands a swing of over 10 m/s^2 if the
display tracks the face rigidly: the airframe hits 64 degrees of tilt, the
motors saturate a third of the time and the display goes 26 degrees off
upright. Run the `turnaround` mission on the Flight tab with the governor
off to see it. With the reference governor on (the default) the screen cuts
the corner under a 2.5 m/s^2 cap, the keep-out sphere moves with the head,
and the same turn costs 5.5% over hover with the rotors never closer than
0.61 m to the eye.

---

## The design this engine produced

Running the tool against honest, component-level assumptions and then flying
the result gives **DESK-1**, written up in [docs/DESK-1.md](docs/DESK-1.md)
and loadable in the tool as the starting point *"DESK-1, the designed
machine"*. The **Spec sheet** tab computes the whole thing live: geometry,
mass build-up, rotor operating point, electrical chain, control authority,
and every mission flown.

**DESK-1/W** carries a full 1920 x 1080 desktop on a 15.6 inch laptop panel
with a Raspberry Pi thin client and a separate flight controller: a
**1.724 kg** quadrotor on four 492 mm two-blade paddle propellers turning
1036 rpm, **53.8 dBA free field at 1 m** and **56.6 dBA at your ear once the
room is in the model**, 106 W, 30 minutes hovering to the 12 % reserve and
28 minutes if you turn around all session, 1.26 m folding span, standing
1.20 m back so that the blades stay 0.59 m from your face. On a 37 g tether
at 48 V it gets lighter (1.62 kg), quieter and never stops.

The current numbers all live in `paper/results/results.json`, generated by
`paper/make_results.py`, and the paper in `paper/` is built from them.

Three findings drove everything:

**The mass closure was never the binding constraint. Noise was.** Getting
there needed three model constants fixed against real hardware, the largest
being a 14 dB error in the acoustic anchor caused by treating a declared
sound *power* level as a pressure at 1 m.

**A resolution is not a payload.** Every 1920-wide panel from 13 to 24 inches
is sharp at 1.20 m, so pixel density does not pick the screen. Areal density
does: a desktop monitor panel is more than twice as heavy per unit area as a
laptop panel, and that is the difference between a 1.7 kg aircraft and a
4.4 kg one that breaks every limit.

**A safe standoff shrinks the desktop.** The standoff that keeps the blades
0.45 m from your face makes 16 px text subtend 5.8 arcminutes, a third of
the 16 arcminute minimum. Meeting it takes about 280 % interface scaling,
which leaves a 692 x 389 logical desktop on a 1920 x 1080 panel. The
resolution is not the constraint; the angular size of a character is.

## Layout

```
flyingscreen/
  params.py      every assumption, with units, ranges and help text
  sizing.py      momentum theory and the algebraic closure
  components.py  rotor, arm, motor, inertia and acoustics from first principles
  dynamics.py    6-DOF state, forces, moments, motor and battery ODEs
  control.py     cascaded controller and gimbal loop
  mission.py     human trajectories and wind
  simulate.py    RK4 integrator, telemetry and energy accounting
  closure.py     the battery loop with the simulator inside it
  sweep.py       feasibility maps and scaling studies
  optimize.py    constrained design search
  tether.py      the closure with the energy off-board
  api.py         HTTP surface
  cli.py         command line
web/             the tool: nine views, no build step
tests/           the theory, asserted
```

The parameter schema in `params.py` is the single source of truth: it drives
validation, the command line flags and the browser controls. Adding a
parameter there makes a slider appear.

---

## Caveats worth stating

* The acoustic estimate is an empirical scaling anchored on one reference
  rotor. It ranks designs; it does not certify them.
* Rotor aerodynamics are momentum theory plus a single profile-drag term, with
  lumped corrections for ground and ceiling effect, rotor-to-rotor
  interference and room recirculation. There is no blade element momentum
  theory, no wake model, and no inflow correction in the flight power law, so
  mission energies are near-hover estimates.
* The tracking simulation assumes perfect knowledge of the state and of the
  person's head position. Estimator latency and dropouts, which are what a
  keep-out sphere is only as good as, are not modelled.
* Structural sizing covers arm bending, tip deflection and the first bending
  mode. It does not cover buckling, joints, fatigue or crash loads.
* Odd rotor counts are excluded: they leave a net reaction torque at hover that
  a real tricopter cancels with a tilting motor, which this model does not
  carry.
* The mission energy is measured over a window and extrapolated, which assumes
  the window is representative. For a periodic mission it is; for one with
  distinct phases it is not, and the window should cover a whole cycle.
