# Equation index

Every boxed result from the derivation, and the function that implements it.
This is the map to check the code against the mathematics.

## Layer 1: momentum theory

| Equation | Where |
| --- | --- |
| `A = N pi D^2 / 4` | `params.derived` |
| `T = 2 rho A v_i^2` | `components.rotor_point` |
| `v_i = sqrt(T / (2 rho A))` | `components.rotor_point`, with axial inflow |
| `P_i = T v_i = T^1.5 / sqrt(2 rho A)` | `components.rotor_point` |
| `P_prop = (m g)^1.5 / (FM eta sqrt(2 rho A))` | `sizing._power_coefficient` |
| `P_profile = (1/8) rho A (omega R)^3 sigma Cd0` | `components.rotor_point` |
| `FM = T v_i / P_shaft` | `components.rotor_point.FM_effective` |
| `DL = m g / A`, `v_i = sqrt(DL / 2 rho)` | `sizing._fill_point` |
| `P_prop / m = (g / FM eta) sqrt(DL / 2 rho)` | `sizing.solve_constant_dl` |

The figure of merit is an *output* of `components`, not an input: the profile
term is computed from blade geometry, so `FM` falls out of the balance between
induced and profile power. The lumped `FM` parameter remains only for the
closed-form layer, and the Machine tab shows the two side by side.

## Layer 2: mass closure

| Equation | Where |
| --- | --- |
| `m = m_fix + m_str + m_prop + m_bat` | `sizing._fill_point`, `components.close_first_principles` |
| `T_max = lambda m g` | `sizing._fill_point` |
| `m_prop = (lambda g / S_m) m = gamma_m m` | `sizing.coefficients` |
| `m_str = f_s m` | `sizing.coefficients` |
| `m_bat = (t_f / e_b) [P_prop + P_aux]` | `sizing._fill_point` |
| `alpha m - beta m^1.5 = M0` | `sizing.solve_power_closure` |
| `alpha = 1 - f_s - lambda g / S_m` | `sizing.coefficients` |
| `beta = t_f g^1.5 / (e_b FM eta sqrt(2 rho A))` | `sizing.coefficients` |
| `M0 = m_fix + P_aux t_f / e_b` | `sizing.coefficients` |
| `m* = 4 alpha^2 / 9 beta^2` | `sizing.payload_wall` |
| `M0_max = 4 alpha^3 / 27 beta^2` | `sizing.payload_wall` |
| `M0_max ~ 1 / t_f^2` | `sweep.payload_boundary`, `sizing.sensitivities` |
| `M0_max = 8 alpha^3 rho A e_b^2 FM^2 eta^2 / (27 g^3 t_f^2)` | asserted by `sizing.sensitivities` exponents |
| general fold `m* = (alpha / q beta)^(1/(q-1))`, `M0_max = alpha m* (1 - 1/q)` | `sizing.solve_power_closure` |
| `A ~ m^p` gives `P ~ m^((3-p)/2)` | `sizing.coefficients.q` |
| `m = M0 / (1 - f_s - gamma_m - gamma_b)` | `sizing.solve_constant_dl` |
| `f_s + gamma_m + gamma_b < 1` | `sizing.solve_constant_dl` |
| `d/dm [m_str + m_prop + m_bat] < 1` | `sizing._fill_point.dmdm` |
| `dm/dtau = M0 - alpha m + beta m^1.5` | `sizing.fixed_point_trace` |

The general-exponent fold formula reduces to the boxed `4 alpha^2 / 9 beta^2`
and `4 alpha^3 / 27 beta^2` at `q = 3/2`, which is asserted in
`test_fold_location_and_height`.

## Layer 3: structure and components

| Equation | Where |
| --- | --- |
| `sigma = M c / I`, `M ~ T_i L` | `components.arm_structure` |
| `I = pi r^3 t` for a thin-walled tube | `components.arm_structure` |
| `delta = F L^3 / 3 E I` | `components.arm_structure` |
| `f1 = (1.875^2 / 2 pi) sqrt(E I / mu L^4)` | `components.arm_structure` |
| `L = gap R / sin(pi / N)` | `components.arm_structure` |
| `Q = k (m / m_ref)^e m`, solved for motor mass | `components.motor_mass_for_torque` |
| guard mass proportional to `N pi D` | `components.arm_structure` |
| prop mass `k D^2.6 (n_blades / 2)` | `components.propulsion` |
| `Ct = T / (rho A V_tip^2)`, `Ct/sigma` stall limit | `components.rotor_point` |
| `S_m` implied `= T_max / m_prop` | `components.propulsion.S_m_implied` |

## Layers 4 and 5: flight

| Equation | Where |
| --- | --- |
| `rdot = v` | `dynamics.derivatives` |
| `m vdot = T R e3 - m g e3 + F_D + F_dist` | `dynamics.derivatives` |
| `F_D = -0.5 rho Cd A_proj \|v_rel\| v_rel` | `dynamics.aero_forces` |
| `J wdot + w x J w = tau` | `dynamics.derivatives` |
| `qdot = 0.5 Omega(w) q` | `dynamics.quat_deriv` |
| `Rdot = R [w]_x` | `dynamics.hat`, used by the controller |
| `T_i = k_T Omega_i^2`, `Q_i = k_Q Omega_i^2` | `dynamics.rotor_forces`, allocation matrix |
| `tau = sum r_i x T_i e3 + sum s_i Q_i e3` | `dynamics.build_vehicle.alloc` |
| `tau_m Omegadot_i = Omega_cmd - Omega_i` | `dynamics.derivatives` |
| `Edot = -P`, `P = sum k_P Omega_i^3 + P_aux` | `dynamics.electrical_power` |
| `sdot = -P / E_max` | reported as `soc` telemetry |
| `J_g thddot_g + b_g thdot_g = tau_g - tau_wind` | `dynamics.derivatives` |
| `xdot = A x + B u` at hover | `dynamics.hover_linearisation` |

The DC/BLDC electrical model (`L idot = V - R i - k_e Omega`) is not
implemented: the first-order speed lag is used instead, which is the standard
preliminary-design simplification and is what the derivation offers as the
simple model. The torque constant and peak power that the electrical model
would need are still computed, in `components.propulsion`.

## Layer 6: control

| Equation | Where |
| --- | --- |
| `r_d = r_h + d` | `mission.desired_position` |
| `e_p = r_d - r`, `e_v = rdot_d - v` | `control.desired_force` |
| `a_c = rddot_d + Kp e_p + Kd e_v (+ Ki integral)` | `control.desired_force` |
| `F_c = m (a_c + g e3)`, `T_c = \|F_c\|` | `control.__call__` |
| desired attitude from `b3 = F_c / \|F_c\|` and yaw | `control.__call__` |
| `e_R = 0.5 vee(R_d^T R - R^T R_d)` | `control.__call__` |
| `tau = J(-K_R e_R - K_w e_w) + w x J w` | `control.__call__` |
| near-hover `xddot = g theta`, `yddot = -g phi` | recovered numerically by `hover_linearisation` |

## Layer 7: closure and optimisation

| Equation | Where |
| --- | --- |
| `E_mission = integral P(x, u, p) dt` | `simulate.simulate_flight` |
| `m_bat = E_mission / e_b` | `closure.close_with_simulation` |
| `R1 = m - (m_fix + m_str + m_prop + m_bat)` | `closure.gross_mass_for_battery` |
| `R2 = e_b m_bat - E_mission` | `closure.close_with_simulation` |
| `min J = w_m m + w_P P + w_N noise + w_s span` | `optimize.evaluate` |
| `T_max >= lambda m g`, `SOC >= SOC_min`, `Omega <= Omega_max`, `d >= d_safe`, `theta <= theta_readable` | `optimize.evaluate`, `simulate` warnings |

## Tether

| Equation | Where |
| --- | --- |
| `I = P / V` | `tether.solve_tethered` |
| `A_c = 2 rho_e L P / (drop V^2)` | `tether.solve_tethered` |
| `m_conductor = 4 rho_e rho_m L^2 P / (drop V^2)` | `tether.tether_constants` |
| `alpha m - beta_t m^1.5 = M0_t` | `tether.solve_tethered` |
