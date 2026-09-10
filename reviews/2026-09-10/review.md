Mathematical and engine review, 10 September 2026

The updated coupled-mission table and explicit discussion of dynamic clearance make the design study more useful. The central analytical hover closure remains a useful model under its stated assumptions. Several propositions, numerical guarantees, and model-to-result connections still require correction before the paper can support its stronger conclusions.

This review checks the updated results and limitations against the associated derivations and current engine. It includes independent torque optimization, a counterexample to inner-map uniqueness, an actual missed pair of component-closure roots, and coupled-flight reruns. The existing suite passes: 52 tests in 9.17 s. Passing these tests does not establish the claims examined below.

Reproduce with `python reviews/2026-09-10/reproduce.py` from the repository root. `checks.json` records the complete parameter set and numerical outputs. These review files do not change the engine or manuscript.

**1. Confirmed engine failure: geometric mass search can skip feasible roots.**

Locations: `flyingscreen/components.py:697`, `paper/sec15-numerics.tex:43`, and the end of `paper/sec08-components.tex`.

The component solver samples masses by multiplying by 1.35 and declares infeasibility if none of those samples has a positive residual. Even a strictly concave residual with two roots can have a positive interval narrower than a sampling gap. Bisection is reliable after a valid bracket is found; the search for that bracket is the defective step.

For the DESK-1/W preset with the `pi_thinclient` payload, changing only the requested duration to 4,280.05020145 s gives two independently bracketed roots:

| Quantity | Independent result |
|---|---:|
| Light mass root | 4.5677791206 kg |
| Heavy mass root | 4.6172610403 kg |
| Residual at either root | less than 5e-16 kg in this run |
| Engine result | `feasible=False` |

These are mass-closure solutions, not a claim that either aircraft meets the acoustic, geometric, or flight constraints. They directly refute the engine's stated reason that no mass closes. Also, the lumped residual's concavity does not automatically transfer to a component residual containing clipping, maximum constraints, and empirical nonlinear functions.

Fix: find or bound residual extrema over a declared design domain, use branch continuation or adaptive bracketing, and distinguish a certified absence of roots from an unsuccessful search. Add this actual near-fold case as a regression test. Do not describe nonconvergence alone as physical impossibility.

**2. Confirmed factor-of-four error in quadrotor torque authority.**

Locations: `flyingscreen/control.py:132`, the authority equation in `paper/sec12-control.tex`, and the corresponding appendix derivation.

The expression divides the summed rotor moments by N. Keeping thrust constant is achieved by balancing positive and negative rotor increments; it does not require dividing the resulting moment by N.

For the current symmetric quadrotor at lambda = 1.8, independent linear programming maximizes each torque while holding thrust at mg, the other two torques at zero, and every squared rotor speed within its limits:

| Axis | Engine, N m | Constrained maximum, N m |
|---|---:|---:|
| Roll | 0.9148604581 | 3.6594418324 |
| Pitch | 0.9148604581 | 3.6594418324 |
| Yaw | 0.1294515061 | 0.5178060244 |

For this symmetric quad, a balanced differential has magnitude Delta = min(Omega_h^2, Omega_max^2 - Omega_h^2), and the single-axis maximum is Delta times the sum of the absolute allocation-row coefficients. There is no 1/N. For general layouts and failures, solve the constrained allocation problem instead of extending this formula blindly.

The gain-selection statement also neglects rate feedback, simultaneous torque demands, yaw limits, and actuator lag. Its proportional-term bound is a conservative design heuristic under additional assumptions, not a necessary-and-sufficient saturation theorem. Recalculate authority first, then retune and simulate; simply multiplying every gain by four is not justified. Layout comparisons based on the current authority values need regeneration.

**3. The endurance-wall proposition is false at p = 1.**

Location: `paper/sec07-scaling.tex`, proposition `prop:twall` and table caption.

The paper already derives the correct constant-disk-loading condition in `paper/sec05-closure.tex`: with propulsive power c_P m, the positive-mass solution requires

    alpha - c_P t_f / e_b > 0.

Consequently the endurance supremum is finite:

    t_lim = alpha e_b / c_P.

There is no finite-mass solution at equality for positive fixed load. With a separate reserve fraction, replace e_b by e_b(1-reserve). The mass diverges as the limit is approached; the endurance does not. The table's finite p = 1 entry agrees with this correction, while its caption and proposition contradict it. As p approaches 1 from below at fixed anchor and fixed M_0, the endurance limit tends to this finite value.

Strict increase of the endurance wall with p for every anchor is also unproved and generally false. For fixed M_0, maximizing endurance over mass gives m_star = q M_0/[alpha(q-1)], and the envelope derivative is

    d(log t_max)/dp = (1/2) log(m_star/m_0).

Its sign depends on the anchor. State the condition under which the plotted family is increasing. Separate absence of a fold from absence of an endurance ceiling; those are different statements.

**4. The inner-map uniqueness theorem has insufficient assumptions.**

Location: `paper/sec13-codesign.tex`, proposition `prop:inner`.

Continuity, nondecreasing structural/propulsion mass, and sublinear asymptotic growth establish existence, but not uniqueness. A smooth counterexample satisfying all those assumptions is

    S(m) = 4 / [1 + exp(-4(m-3))],     m_fix + mu = 1.

The equation m = 1 + S(m) has roots approximately 1.001348654, 3, and 4.998651346. Thus G(mu) is not necessarily a single-valued function under the stated hypotheses. The proof introduces a linear-model restriction that is absent from the proposition and does not cover all component models. Continuity alone also does not permit invocation of the implicit function theorem.

A sufficient replacement is a global Lipschitz bound |S(m2)-S(m1)| <= L|m2-m1| with L < 1, or differentiability with 0 <= S'(m) <= 1-epsilon on the relevant domain, together with existence conditions. Then G' = 1/[1-S'(G)]. Verify that the actual submodels satisfy the proposed condition over the claimed domain, or restrict the theorem to the lumped linear model.

**5. Reserve and power constraints are inconsistent across the mathematical formulations.**

Locations: `paper/sec13-codesign.tex` equations `cd3`, `Hprime2`, and `residual`; `flyingscreen/closure.py:102`.

The co-design statement sets e_b m_bat equal to consumed energy while also requiring a positive terminal reserve. Starting with E(0) = e_b m_bat makes those requirements incompatible. The outer map correctly inserts (1-reserve), but the stated residual omits it. The reduced derivative silently sets reserve to zero in the proof, although the proposition does not state that restriction.

Use the battery constraint consistently:

    m_bat >= max(E_mission/[e_b(1-reserve)], P_peak/p_b).

For a minimally sized continuous battery, equality holds with this maximum; an oversized or discrete pack need not satisfy equality. In the energy-limited branch, the reduced derivative contains 1/(1-reserve) when beta retains its original definition. The maximum introduces a possible nonsmooth branch change, which matters for secant convergence assumptions.

The engine iterates a reserve-aware required mass but records `e_b*m_bat - E_mission` as its residual. That logged quantity is expected to be positive at reserve-aware convergence. Record `m_bat - m_bat_required` and separately report energy and power margins. Recheck the final simulation before declaring the final design converged and constraint-feasible.

The statement that mission overhead is exactly equivalent to degrading alpha is too strong. Multiplying power changes the energy coupling, and multiplying total power also changes the auxiliary-energy contribution. It is a useful analogy for a particular contraction inequality, not an identity of the full mass closure.

**6. The new clearance discussion identifies the right problem but does not repair the guarantee.**

Locations: `paper/sec17-results.tex:314`, `paper/sec18-limitations.tex:64`, the governor proposition in `paper/sec12-control.tex`, and `flyingscreen/mission.py:229`.

For a moving head center c(t), the boundary derivative is

    d/dt ||r_g-c|| = n dot (v_g - c_dot).

The proof and velocity projection use n dot v_g instead. A stationary governed reference can be approached by a moving person even when that tested quantity is zero. Discrete projection establishes a reference-position constraint at update instants; it does not by itself establish continuous-time invariance, acceleration-limited motion, or an actual-vehicle constraint. Position projection can jump the reference, and the reported acceleration is not recomputed to match all velocity corrections.

The added margin has a valid conditional triangle-inequality argument: if ||r-r_g|| <= epsilon uniformly for the modified system, then keeping the reference at least rho+epsilon away keeps the vehicle at least rho away. The maximum error from one previous run is not automatically such a bound. Include head-estimation and latency errors if claiming physical clearance. Increasing the desired standoff is not mathematically equivalent to enlarging the keep-out set during a turn.

There is a second measurement issue: `simulate.py` reports ||r-head|| - reach. This is distance to an enclosing sphere, not the exact nearest blade or rotor disk. A value of 0.41 m establishes violation of the conservative envelope criterion; by itself it does not prove that a physical blade came within 0.41 m of the face. Preserve the conservative check, label it correctly, and compute oriented disk distances when reporting physical rotor clearance.

For the project, use a dynamically feasible reference, monitor actual vehicle clearance, and make clearance violations an explicit feasibility failure. A converged battery loop can still be an unacceptable flight design.

**7. The cited attitude theorem does not establish the implemented controller's guarantees.**

Location: `paper/sec12-control.tex`, theorem `thm:lee`; implementation at `flyingscreen/control.py:104`.

The cited analysis uses positive scalar gains and desired-angular-rate/feedforward terms. The manuscript states positive-definite gains but inserts K_R into a scalar inequality. The implemented controller uses inertia-normalized gains, sets desired body rate to zero, and omits the trajectory feedforward terms. Its plant additionally includes actuator lag and saturation. These changes need their own assumptions or analysis; the cited proposition cannot simply be transferred. See [Lee, Leok and McClamroch, Proposition 1 and controller equation (11)](https://arxiv.org/pdf/1003.2005).

Independently, for the unweighted Psi defined in this manuscript, every 180-degree rotation is a critical attitude: R_d^T R = 2uu^T-I for any unit u is symmetric, so e_R = 0 and Psi = 2. There are infinitely many such attitudes, not three isolated nontrivial critical points. Rewrite this paragraph and avoid characterizing the entire excluded state-space set as three attitudes.

**8. The flight energy model does not implement the stated axial-flight power model.**

Locations: `flyingscreen/dynamics.py:328`, `flyingscreen/components.py:290`, and the mission-energy interpretation in the paper.

The component model accepts axial velocity and computes the ideal contribution T(v_axial+v_i). The flight power function instead uses fixed hover-calibrated k_P values times Omega cubed, with no inflow or climb-velocity input. At fixed rotor speed it returns the same propulsive power for hover and vertical translation. It therefore cannot represent the axial-flight power correction already derived in the manuscript.

The new table closes energy around this approximate power law. That is a useful computational coupling, but it is not validation of real maneuvering energy. Add an explicit near-hover applicability statement now. Next, couple an axial/oblique-inflow model to both thrust and power consistently; merely adding a climb term to power while leaving inconsistent thrust dynamics needs care. Prioritize this before strong takeoff, descent, or aggressive-motion endurance claims.

**9. Geometry and inertia are inconsistent between modules.**

Locations: `flyingscreen/components.py:531`, `flyingscreen/dynamics.py:217`, and `flyingscreen/mission.py:182`.

`screen_axes` defines the resting panel in the body y-z plane. For a thin rectangular panel of width w along y and height h along z, its central diagonal inertia is

    J_screen = (m_screen/12) diag(w^2+h^2, h^2, w^2).

The component inertia uses a different assignment of these dimensions. Correct it before interpreting the reported pitch inertia and gain values. Apply the parallel-axis theorem about the actual vehicle center of mass; an offset in both x and z generally produces an x-z product of inertia. Treat the gimbal separately if it is not reasonably represented by the same slab.

The forward boom affects static clearance and inertia, but the dynamic screen offset omits its horizontal component. The desired centroid also remains at the specified screen/eye height despite the vertical screen offset. This makes the long-boom and raised-screen comparisons inconsistent. Define one shared component geometry that generates center of mass, full inertia, force application points, reference positions, collision geometry, and visualization.

**10. The updated result table needs a reproducible experiment definition.**

The independent hover closure gives 1.723758392 kg gross, 0.288295723 kg battery, and 106.046699 W, matching the printed rounded baseline. However:

- The table says three full simulations. All five reruns perform three outer iterations followed by a fourth simulation. Distinguish iterations from total simulation calls.
- The script in this review explicitly uses a 14 s window, 4 ms step, default settling interval, and the stated 2.5 m/s and 2.5 m/s^2 governor limits. Some mission statistics differ from the paper's rounded values. The manuscript needs the precise window, settling interval, seed, parameter snapshot, initialization, and solver tolerances for exact reproduction.
- The printed 288 g battery and 106 W hover load provide 30 minutes to 12% reserve under the stated energy model. Approximately 34.1 minutes is the estimate to exhaust the model's available energy, not to retain that reserve.
- The statement that the battery must be sized for the most demanding motion is a worst-case operating policy, not the general energy rule. For a defined mixed session, integrate all phases or use their time fractions; independently enforce peak power. Repeated turnarounds for an entire session and a few occasional turns are different requirements.
- Agreement between hover sizing and standing simulation checks implementation consistency, since both share the rotor model. It is not independent physical validation.

Generate manuscript tables from saved engine results and retain the configuration beside them. Report separate mass/energy convergence and constraint-feasibility columns, including actual reserve, clearance margin, saturation, and screen error.

The completed 14-second reruns give:

| Motion | Gross, kg | Battery, g | Mean power, W | Lag RMS, mm | Minimum envelope gap, m |
|---|---:|---:|---:|---:|---:|
| Standing | 1.723647 | 288.207 | 106.038 | 1.756 | 0.5689 |
| Pacing | 1.725457 | 289.653 | 106.571 | 59.369 | 0.4856 |
| Walking a loop | 1.729638 | 292.995 | 107.802 | 111.509 | 0.5761 |
| Jogging | 1.736806 | 298.724 | 109.913 | 119.057 | 0.5761 |
| Turning around | 1.774493 | 328.854 | 121.018 | 1236.111 | 0.4079 |

The turnaround's maximum governed-reference tracking error is about 0.507 m; this is different from the approximately 0.04 m inward clearance shortfall and from the raw-reference lag. Substituting the full error norm into the proposed margin could materially change the geometry and behavior, so it needs a new closed-loop evaluation. All five final reserve estimates fall slightly below 12% (11.9615â€“11.9798%), illustrating the unverified final update and tolerance issue. These are small numerical shortfalls, not a large endurance discrepancy.

**11. Tethering removes duration dependence, but does not generally delete the fold.**

Location: `paper/sec17-results.tex:345`; both closures in `flyingscreen/tether.py`.

If carried cable mass is proportional to power and power scales as m^(3/2), then any positive beta_tether retains the same mathematical fold. A small coefficient moves it. Within this simplified closure the term disappears when the carried cable mass is independent of power, or is fully supported elsewhere. The minimum-gauge regime can be linear over its valid range, but it need not remain so at arbitrarily high power.

The component tether model also carries a battery for a fixed landing duration, which reintroduces a power-dependent mass contribution. Its `feasible=True` is unconditional after the iteration limit, and the reserve battery is sized only by energy; add convergence, discharge-power, and voltage checks. Distinguish a redesigned tethered aircraft from removing the battery on an otherwise fixed airframe, since the current closure resizes propulsion and structure.

**12. The rotor-failure discussion contains incorrect generalizations.**

Location: `paper/sec18-limitations.tex:53`.

With the engine's alternating-spin regular hex layout, removing an opposite pair leaves an allocation matrix of rank 3, independently confirmed in the review script. Four remaining rotors therefore do not give arbitrary independent thrust/roll/pitch/yaw control. Pure total-lift arithmetic alone would require lambda >= 6/4 = 1.5, not 2, but that arithmetic does not fix the lost wrench direction.

The universal claim that a quadrotor necessarily falls after a rotor failure is also too broad: specialized controllers can use relaxed, spinning flight. That capability is not present in this engine and does not establish acceptability beside a person. See [Mueller and D'Andrea's original study](https://www.research-collection.ethz.ch/handle/20.500.11850/91723). Describe the present controller's failure capability explicitly.

**13. Additional corrections and verification gaps.**

- `paper/sec15-numerics.tex:101`: a first-order projection would not justify preserving fourth-order accuracy. Smooth quaternion normalization can preserve the RK method's order near a nonzero exact unit quaternion; give that argument. Distinguish RK4 integration of the held-input plant from approximation of a continuous-feedback controller. Check convergence with controller sampling held fixed and integration substeps varied.
- `flyingscreen/simulate.py:148`: the state is advanced to t+dt and then compared with references/head positions at t, with telemetry labeled t. Align timestamps before interpreting millimeter tracking errors. Peak power is measured only after the settling interval; sizing needs all relevant mission peaks.
- `flyingscreen/dynamics.py:434`: the controllability boolean allows rank n-1. Full controllability requires rank n in the chosen independent coordinates. Scale the matrices sensibly before rank testing.
- `tests/test_engine.py:410`: the optimal-tip-speed comparison leaves automatic tip-speed selection enabled, so the manually written tip speed can be ignored. Test against an explicitly manual-speed case.
- In `paper/sec17-results.tex`, the first six display-table room levels are each only 1 dB above their free-field 1 m levels, whereas the later rows add about 5.1 dB. Under the same room and 0.70 m listener model these cannot all represent the same ear-level calculation. Save and state per-row distances/room inputs, or correct the mislabeled values.
- In the appendix's endurance derivative, the numerator is positive but the denominator changes sign between the light and heavy branches. Mass increases with duration on the light branch, not throughout both branches.
- Pixel density in pixels/degree alone is not a readability criterion: moving a fixed panel farther away increases this density while shrinking a given text glyph. Include glyph angular size and UI scaling before claiming comfortable desktop readability at 1.2 m.
- The abstract describes six parts; the document has three. Replace universal first-principles and optimality language with the actual combination of analytical models, empirical fits, numerical optimization, and simulation. The current LaTeX log also requests another cross-reference pass.

**Suggested next work for this project.**

First correct the equations, mass-root search, reserve accounting, authority calculation, timestamps, and shared geometry, then regenerate the existing study. These are necessary to know which current conclusions survive.

Next define the intended operation precisely: indoor following speeds, permitted turn lag, viewing distance/UI scale, mission phase durations, head-tracking latency, and what happens when the person moves too quickly. Evaluate energy closure and all motion constraints together. A controllable display may deliberately allow heading/standoff lag; the admissible lag should be a product requirement rather than an accidental simulation result.

Then quantify uncertainty in rotor efficiency, motor mass, drag, electrical efficiency, cell energy, and especially acoustic calibration. Show whether plausible uncertainty changes the design ranking and whether the candidate satisfies the constraints across those ranges. The present single best predicted sound level should not carry the entire feasibility conclusion.

The physical validation priority already identified in the updated limitations is appropriate: measure a candidate rotor's thrust, electrical power, RPM, and noise spectrum over the operating range. Use those data to validate the strongest extrapolation in the design. Full room-flow simulation and extension to unrelated aircraft types can follow when the present hover/follow model and measurements agree.
