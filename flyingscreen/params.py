"""
Parameter definitions for the flying-screen co-design problem.

Every assumption in the model lives here as a single flat dictionary of
scalars, together with a schema that carries units, ranges, grouping and
help text.  The schema is what the API serves to the frontend so that the
UI panels are generated directly from the model, never hand-mirrored.

Notation follows the derivation:

    m        gross mass                                        [kg]
    m_fix    payload + electronics that do not scale with m    [kg]
    f_s      structural mass fraction, m_str = f_s m           [-]
    lam      peak thrust ratio, T_max = lam m g                [-]
    S_m      propulsion specific thrust, T_max / m_prop        [N/kg]
    FM       rotor figure of merit                             [-]
    eta      electrical efficiency (motor x ESC)               [-]
    e_b      usable battery specific energy                    [J/kg]
    A        total actuator disk area, N pi D^2 / 4            [m^2]
    t_f      mission / endurance time                          [s]
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List

G0 = 9.80665            # standard gravity                     [m/s^2]
J_PER_WH = 3600.0
WH_PER_J = 1.0 / J_PER_WH


# ---------------------------------------------------------------------------
# Schema
# ---------------------------------------------------------------------------
# Each entry: key, label, unit, default, lo, hi, group, and optional flags.
_SCHEMA: List[Dict[str, Any]] = [
    # -- environment --------------------------------------------------------
    dict(key="rho", label="Air density", unit="kg/m^3", default=1.225,
         lo=0.7, hi=1.3, group="Environment",
         help="Sea level ISA is 1.225. Denver is about 1.0. Enters hover power as 1/sqrt(rho)."),
    dict(key="g", label="Gravity", unit="m/s^2", default=G0,
         lo=1.0, hi=12.0, group="Environment", advanced=True,
         help="Enters the payload wall as g^-3."),
    dict(key="wind_mean", label="Mean wind", unit="m/s", default=0.0,
         lo=0.0, hi=15.0, group="Environment",
         help="Steady headwind used by the 6-DOF simulator. Indoors this is 0."),
    dict(key="wind_gust", label="Gust intensity", unit="m/s", default=0.0,
         lo=0.0, hi=8.0, group="Environment",
         help="RMS of an Ornstein-Uhlenbeck gust process added to the mean wind."),
    dict(key="gust_tau", label="Gust correlation time", unit="s", default=2.0,
         lo=0.1, hi=30.0, group="Environment", advanced=True,
         help="Time constant of the gust process."),

    # -- the room ------------------------------------------------------------
    dict(key="ceiling_height", label="Ceiling height", unit="m", default=2.4,
         lo=2.0, hi=8.0, group="Room",
         help="A machine hovering at head height in an ordinary room is close enough "
              "to the ceiling for it to matter to the rotors."),
    dict(key="room_length", label="Room length", unit="m", default=5.0,
         lo=2.0, hi=30.0, group="Room"),
    dict(key="room_width", label="Room width", unit="m", default=4.0,
         lo=2.0, hi=30.0, group="Room"),
    dict(key="room_alpha", label="Average sound absorption", unit="-", default=0.20,
         lo=0.05, hi=0.80, group="Room",
         help="Bare hard room 0.10, ordinary furnished 0.20, carpet and curtains 0.35, "
              "acoustically treated 0.55. This sets how loud the whole room becomes, "
              "which past the critical distance matters more than how far away you sit."),
    dict(key="ige_enable", label="Model ground and ceiling effect", unit="-", default=1,
         lo=0, hi=1, integer=True, boolean=True, group="Room", advanced=True,
         help="Cheeseman and Bennett for the floor, the same form with a separate "
              "coefficient for the ceiling. Turn off to see what they were worth."),
    dict(key="ceiling_k", label="Ceiling effect coefficient", unit="-", default=2.0,
         lo=0.5, hi=6.0, group="Room", advanced=True,
         help="Ceiling effect is stronger than ground effect at the same spacing "
              "because the ceiling restricts the rotor inflow rather than the wake. "
              "Reported values scatter widely; this is the least certain constant here."),
    dict(key="interference_enable", label="Model rotor interference", unit="-", default=1,
         lo=0, hi=1, integer=True, boolean=True, group="Propulsion", advanced=True,
         help="Adjacent rotors running close together each work in the other's inflow, "
              "which costs induced power. Matters once tip-to-tip gap drops below "
              "about a quarter of a diameter."),

    # -- payload ------------------------------------------------------------
    dict(key="m_screen", label="Screen mass", unit="kg", default=0.45,
         lo=0.02, hi=5.0, group="Payload",
         help="Stripped panel plus driver board. A 13 inch panel with driver is about 0.35-0.5 kg."),
    dict(key="m_electronics", label="Compute + sensors", unit="kg", default=0.12,
         lo=0.0, hi=2.0, group="Payload",
         help="Flight controller, companion computer, tracking camera, radio."),
    dict(key="m_gimbal", label="Gimbal + mount", unit="kg", default=0.10,
         lo=0.0, hi=2.0, group="Payload",
         help="Two-axis screen stabiliser. Counts as fixed mass, not structure."),
    dict(key="P_screen", label="Screen power", unit="W", default=8.0,
         lo=0.0, hi=200.0, group="Payload",
         help="Panel plus backlight. Enters M0 as P_aux t_f / e_b, so it costs battery mass directly."),
    dict(key="P_computer", label="Compute power", unit="W", default=12.0,
         lo=0.0, hi=300.0, group="Payload",
         help="Companion computer running the human tracker."),
    dict(key="P_sensors", label="Sensor + radio power", unit="W", default=3.0,
         lo=0.0, hi=100.0, group="Payload"),
    dict(key="screen_w", label="Screen width", unit="m", default=0.29,
         lo=0.05, hi=1.5, group="Payload",
         help="Sets the sail area that couples attitude into translation."),
    dict(key="screen_h", label="Screen height", unit="m", default=0.18,
         lo=0.03, hi=1.0, group="Payload"),
    dict(key="Cd_screen_n", label="Screen Cd (normal)", unit="-", default=1.17,
         lo=0.5, hi=2.0, group="Payload", advanced=True,
         help="Flat plate normal to the flow."),
    dict(key="Cd_screen_t", label="Screen Cd (edge-on)", unit="-", default=0.05,
         lo=0.0, hi=0.5, group="Payload", advanced=True),
    dict(key="r_cp", label="Screen offset from CoM", unit="m", default=0.15,
         lo=0.0, hi=1.0, group="Payload", advanced=True,
         help="Distance from the vehicle centre of mass to the screen centre. Turns screen "
              "drag into a pitching moment, and sets how far the rotor plane sits from the "
              "display."),
    dict(key="screen_forward", label="Screen boom length", unit="m", default=0.0,
         lo=0.0, hi=0.9, group="Payload",
         help="How far the display is cantilevered ahead of the rotor centroid, toward the "
              "user. Zero puts the screen over the middle of the machine, which means the "
              "near rotor tips reach to within a few centimetres of your face. A boom buys "
              "that clearance back without pushing the screen away, and pays for it in "
              "pitch inertia."),
    dict(key="screen_above", label="Screen above the rotors", unit="-", default=1,
         lo=0, hi=1, integer=True, boolean=True, group="Payload",
         help="On: the rotor plane sits below the display, which drops the blades out of "
              "your eyeline and puts the panel between them and your face. Off: the screen "
              "hangs underneath, which lifts the rotors to head height. The screen is "
              "vertical and the rotor flow is vertical, so this costs almost nothing "
              "aerodynamically; what it changes is where the blades are relative to you."),

    # -- propulsion ---------------------------------------------------------
    dict(key="N_rotors", label="Rotor count", unit="-", default=4,
         lo=4, hi=12, step=2, integer=True, even=True, group="Propulsion",
         help="Even counts only. An odd rotor count leaves a net reaction torque at "
              "hover that a real tricopter cancels with a tilting motor, which this "
              "model does not carry."),
    dict(key="D_rotor", label="Rotor diameter", unit="m", default=0.40,
         lo=0.10, hi=2.00, group="Propulsion",
         help="Disk area A = N pi D^2 / 4 enters the payload wall linearly, so D enters as D^2."),
    dict(key="lam", label="Thrust margin lambda", unit="-", default=2.0,
         lo=1.1, hi=4.0, group="Propulsion",
         help="T_max = lambda m g. Control authority and gust rejection, paid for in motor mass."),
    dict(key="S_m", label="Propulsion specific thrust", unit="N/kg", default=120.0,
         lo=30.0, hi=400.0, group="Propulsion",
         help="Max thrust per kg of motors, ESCs and props. 100-150 N/kg is realistic today."),
    dict(key="FM", label="Figure of merit", unit="-", default=0.65,
         lo=0.3, hi=0.85, group="Propulsion",
         help="Rotor efficiency against the ideal actuator disk. Bounded above by 1, about 0.7 in practice."),
    dict(key="eta", label="Electrical efficiency", unit="-", default=0.75,
         lo=0.3, hi=0.98, group="Propulsion",
         help="Motor times ESC. Multiplies FM in every power expression."),
    dict(key="tau_motor", label="Motor time constant", unit="s", default=0.045,
         lo=0.005, hi=0.5, group="Propulsion", advanced=True,
         help="First order rotor speed lag, tau dOmega/dt = Omega_cmd - Omega."),
    dict(key="tip_speed_auto", label="Set RPM from blade loading", unit="-", default=1,
         lo=0, hi=1, integer=True, boolean=True, group="Propulsion",
         help="On: tip speed follows from the target blade loading, which is how rotors are "
              "actually designed, and larger disks then genuinely cost less power. "
              "Off: the tip speed below is used verbatim, and a large disk spun too fast "
              "wastes everything it gained on profile drag."),
    dict(key="ct_sigma_design", label="Design blade loading Ct/sigma", unit="-", default=0.10,
         lo=0.03, hi=0.12, group="Propulsion",
         help="The working point on the blade. Higher loads the blade harder and runs slower "
              "and quieter, until it stalls."),
    dict(key="tip_speed", label="Design tip speed", unit="m/s", default=110.0,
         lo=40.0, hi=250.0, group="Propulsion",
         help="Used only when the blade-loading rule above is off. Noise scales as the sixth "
              "power of tip speed."),
    dict(key="n_blades", label="Blades per rotor", unit="-", default=2,
         lo=2, hi=6, step=1, integer=True, group="Propulsion", advanced=True),
    dict(key="blade_AR", label="Blade aspect ratio", unit="-", default=8.0,
         lo=3.0, hi=20.0, group="Propulsion", advanced=True,
         help="Radius over chord. Sets solidity and hence profile power."),
    dict(key="Cd0", label="Blade profile drag coefficient", unit="-", default=0.014,
         lo=0.006, hi=0.05, group="Propulsion", advanced=True),
    dict(key="kappa_ind", label="Induced power factor", unit="-", default=1.15,
         lo=1.0, hi=1.4, group="Propulsion", advanced=True,
         help="Non-uniform inflow and tip loss penalty on ideal induced power."),
    dict(key="motor_torque_density", label="Motor torque density at 100 g", unit="N.m/kg",
         default=5.5, lo=0.5, hi=14.0, group="Propulsion", advanced=True,
         help="First-principles route to motor mass, replacing the lumped specific thrust S_m. "
              "Quoted as peak torque at a 100 g reference motor, since the motor is sized by "
              "the burst condition T_max. Calibrated against T-Motor MN2806 (66 g, 0.30 N.m), "
              "MN3110 (96 g, 0.63 N.m) and MN505-S (190 g, 2.0 N.m)."),
    dict(key="motor_scale_exp", label="Motor torque density exponent", unit="-", default=0.30,
         lo=0.0, hi=0.6, group="Propulsion", advanced=True,
         help="Torque density scales as motor mass to this power. Zero makes it size independent, "
              "which unfairly penalises the big slow rotors that a quiet hovering machine wants. "
              "Around 0.3 matches the outrunner range from 30 g to 200 g."),
    dict(key="esc_power_density", label="ESC power density", unit="W/kg", default=9000.0,
         lo=1000.0, hi=40000.0, group="Propulsion", advanced=True),
    dict(key="prop_mass_k", label="Propeller mass coefficient", unit="kg/m^2.6", default=0.35,
         lo=0.05, hi=2.0, group="Propulsion", advanced=True,
         help="m_prop_each = k D^2.6 (n_blades / 2). Carbon props sit near 0.35."),
    dict(key="sigma_max", label="Maximum solidity", unit="-", default=0.25,
         lo=0.08, hi=0.60, group="Propulsion", advanced=True,
         help="Blade area over disk area. Blade element theory treats blades as independent, "
              "which stops being true once they start operating as a cascade. Helicopters run "
              "0.05 to 0.12 and high-solidity propellers reach about 0.25; past that you need a "
              "duct and stators, and this model no longer describes what you have."),
    dict(key="ct_sigma_max", label="Blade loading limit Ct/sigma", unit="-", default=0.14,
         lo=0.05, hi=0.25, group="Propulsion", advanced=True,
         help="Above this the blade stalls and the figure of merit collapses."),

    # -- structure ----------------------------------------------------------
    dict(key="f_s", label="Structural mass fraction", unit="-", default=0.20,
         lo=0.0, hi=0.6, group="Structure",
         help="Lumped model m_str = f_s m. The first-principles beam model replaces this."),
    dict(key="sigma_allow", label="Allowable stress", unit="MPa", default=350.0,
         lo=20.0, hi=1500.0, group="Structure", advanced=True,
         help="CFRP tube, already derated. Used by the beam arm model."),
    dict(key="E_mod", label="Elastic modulus", unit="GPa", default=70.0,
         lo=5.0, hi=400.0, group="Structure", advanced=True),
    dict(key="rho_mat", label="Arm material density", unit="kg/m^3", default=1600.0,
         lo=500.0, hi=8000.0, group="Structure", advanced=True),
    dict(key="arm_radius", label="Arm tube radius", unit="m", default=0.010,
         lo=0.003, hi=0.05, group="Structure", advanced=True),
    dict(key="wall_min", label="Minimum wall thickness", unit="m", default=0.0006,
         lo=0.0002, hi=0.005, group="Structure", advanced=True),
    dict(key="SF_struct", label="Structural safety factor", unit="-", default=2.0,
         lo=1.0, hi=5.0, group="Structure", advanced=True),
    dict(key="hub_frac", label="Hub and joints fraction", unit="-", default=0.55,
         lo=0.0, hi=2.0, group="Structure", advanced=True,
         help="Centre plate, motor mounts, fasteners and guards as a fraction of raw arm mass."),
    dict(key="m_frame_fixed", label="Fixed frame hardware", unit="kg", default=0.15,
         lo=0.0, hi=2.0, group="Structure",
         help="Landing gear, wiring, screen mount and fasteners that do not scale with load."),
    dict(key="guard_mass_k", label="Prop guard mass", unit="kg/m", default=0.025,
         lo=0.0, hi=0.5, group="Structure",
         help="Per metre of guard ring, so the cost scales with rotor circumference rather "
              "than disk area. The default is a 6 mm carbon tube ring plus mounting tabs, "
              "about 14 g/m of tube and 11 g/m of hardware; a full finger-proof mesh cage is "
              "nearer 0.06. Guards are not optional when the machine flies beside a head."),
    dict(key="rotor_gap", label="Rotor tip gap factor", unit="-", default=1.12,
         lo=1.0, hi=1.6, group="Structure", advanced=True,
         help="Arm length multiplier so that adjacent disks do not overlap."),

    # -- battery ------------------------------------------------------------
    dict(key="e_b_wh_kg", label="Cell specific energy", unit="Wh/kg", default=250.0,
         lo=80.0, hi=700.0, group="Battery",
         help="Pack level, before depth of discharge. Enters the payload wall squared."),
    dict(key="dod", label="Usable depth of discharge", unit="-", default=1.0,
         lo=0.4, hi=1.0, group="Battery",
         help="Multiplies e_b for cycle life. The landing reserve is a separate "
              "parameter and is not to be folded in here as well."),
    dict(key="p_b_w_kg", label="Cell specific power", unit="W/kg", default=1000.0,
         lo=150.0, hi=6000.0, group="Battery", advanced=True,
         help="Imposes a second floor on pack mass: the pack must also deliver peak power."),
    dict(key="soc_reserve", label="Landing SOC reserve", unit="-", default=0.10,
         lo=0.0, hi=0.4, group="Battery",
         help="The mission must end above this state of charge."),

    # -- mission ------------------------------------------------------------
    dict(key="t_f", label="Endurance", unit="s", default=3600.0,
         lo=60.0, hi=36000.0, group="Mission", log=True,
         help="The most punishing parameter in the model: maximum payload falls as 1 / t_f^2."),
    dict(key="v_follow", label="Follow speed", unit="m/s", default=1.4,
         lo=0.0, hi=8.0, group="Mission",
         help="Walking is about 1.4, jogging about 3."),
    dict(key="a_follow", label="Follow acceleration", unit="m/s^2", default=1.0,
         lo=0.0, hi=8.0, group="Mission"),
    dict(key="standoff", label="Screen standoff", unit="m", default=0.9,
         lo=0.3, hi=3.0, group="Mission",
         help="Desired distance in front of the eyes of the user."),
    dict(key="standoff_z", label="Screen height", unit="m", default=1.5,
         lo=0.5, hi=3.0, group="Mission"),
    dict(key="d_safe", label="Minimum safe distance", unit="m", default=0.5,
         lo=0.1, hi=3.0, group="Mission",
         help="Hard constraint that the trajectory must never violate."),

    # -- control ------------------------------------------------------------
    dict(key="Kp_pos", label="Position P gain", unit="1/s^2", default=6.0,
         lo=0.1, hi=40.0, group="Control"),
    dict(key="Kd_pos", label="Position D gain", unit="1/s", default=4.0,
         lo=0.1, hi=25.0, group="Control"),
    dict(key="Ki_pos", label="Position I gain", unit="1/s^3", default=0.4,
         lo=0.0, hi=10.0, group="Control"),
    dict(key="K_R", label="Attitude gain", unit="1/s^2", default=90.0,
         lo=5.0, hi=600.0, group="Control"),
    dict(key="K_w", label="Body rate gain", unit="1/s", default=18.0,
         lo=1.0, hi=120.0, group="Control"),
    dict(key="Kp_gimbal", label="Gimbal P gain", unit="1/s^2", default=400.0,
         lo=10.0, hi=3000.0, group="Control"),
    dict(key="Kd_gimbal", label="Gimbal D gain", unit="1/s", default=35.0,
         lo=1.0, hi=300.0, group="Control"),
    dict(key="tau_gimbal_max", label="Gimbal torque limit", unit="N.m", default=0.8,
         lo=0.02, hi=10.0, group="Control", advanced=True),
    dict(key="theta_readable", label="Readable screen tilt", unit="deg", default=8.0,
         lo=1.0, hi=45.0, group="Control",
         help="Screen angle error beyond which the display is considered unreadable."),

    # -- sizing model choice ------------------------------------------------
    dict(key="area_p", label="Disk area exponent p", unit="-", default=0.0,
         lo=-0.5, hi=2.0, group="Sizing model",
         help="A ~ m^p. p=0 fixed rotors gives P ~ m^1.5 and the fold. p=1 is constant disk loading, P ~ m."),
    dict(key="disk_loading", label="Target disk loading", unit="N/m^2", default=70.0,
         lo=5.0, hi=600.0, group="Sizing model",
         help="Used by the constant disk loading closure. Low disk loading is the single best lever."),

    # -- tether -------------------------------------------------------------
    dict(key="tether_len", label="Tether length", unit="m", default=6.0,
         lo=1.0, hi=50.0, group="Tether"),
    dict(key="tether_V", label="Tether bus voltage", unit="V", default=350.0,
         lo=12.0, hi=1000.0, group="Tether",
         help="High voltage is what makes a thin tether possible: conductor mass scales as 1 / V^2."),
    dict(key="tether_drop", label="Allowed voltage drop", unit="-", default=0.05,
         lo=0.01, hi=0.3, group="Tether"),
    dict(key="tether_rho_e", label="Conductor resistivity", unit="ohm.m", default=1.68e-8,
         lo=1.0e-8, hi=1.0e-7, group="Tether", advanced=True),
    dict(key="tether_rho_m", label="Conductor density", unit="kg/m^3", default=8960.0,
         lo=1000.0, hi=20000.0, group="Tether", advanced=True),
    dict(key="tether_insul", label="Insulation mass factor", unit="-", default=1.8,
         lo=1.0, hi=4.0, group="Tether", advanced=True,
         help="Total tether mass divided by bare conductor mass."),
    dict(key="tether_support", label="Tether weight carried", unit="-", default=0.5,
         lo=0.0, hi=1.0, group="Tether",
         help="0 if the tether is fully supported from above, 1 if the aircraft lifts all of it."),
    dict(key="eta_dcdc", label="Onboard converter efficiency", unit="-", default=0.94,
         lo=0.7, hi=0.99, group="Tether", advanced=True),
]

SCHEMA: List[Dict[str, Any]] = _SCHEMA
_BY_KEY = {e["key"]: e for e in _SCHEMA}
DEFAULTS: Dict[str, float] = {e["key"]: e["default"] for e in _SCHEMA}
GROUPS: List[str] = list(dict.fromkeys(e["group"] for e in _SCHEMA))


# ---------------------------------------------------------------------------
# Named starting points
# ---------------------------------------------------------------------------
# Each preset is a diff against the defaults, chosen to land on a different
# side of the feasibility wall so the shape of the problem is visible without
# having to hunt for it.
PRESETS: List[Dict[str, Any]] = [
    dict(name="The obvious idea",
         note="A 13 inch panel beside you for an hour on 40 cm rotors. "
              "This is the case that does not close, and the reason the rest exists.",
         values={}),
    dict(name="Short session",
         note="The same machine asked for twenty minutes instead of an hour. "
              "The wall moves by the square of the time, so it closes comfortably.",
         values=dict(t_f=1200.0)),
    dict(name="Light screen, half hour",
         note="A stripped 10 inch panel and a thirty minute session: about the "
              "largest honest version of the untethered idea.",
         values=dict(m_screen=0.25, screen_w=0.22, screen_h=0.14, t_f=1800.0,
                     P_screen=5.0)),
    dict(name="Big slow rotors",
         note="Buying the hour back with disk area instead of time. It closes, "
              "but the span goes past a metre and the rotors are now the size "
              "of the screen.",
         values=dict(D_rotor=0.50, N_rotors=4, t_f=3600.0, m_screen=0.35,
                     ct_sigma_design=0.10)),
    dict(name="Constant disk loading", model="constant_dl",
         note="Rotors grow with the aircraft, p = 1. The m^1.5 fold disappears "
              "and feasibility becomes f_s + gamma_m + gamma_b < 1. Check the "
              "rotor diameter that buys.",
         values=dict(area_p=1.0, disk_loading=40.0, t_f=3600.0, m_screen=0.35)),
    dict(name="Workday on a wire",
         note="Eight hours, which is two orders of magnitude outside the "
              "battery-powered feasible set and trivial on a tether.",
         values=dict(t_f=28800.0, tether_len=6.0, tether_V=350.0,
                     tether_support=0.5)),
    dict(name="Follow me jogging",
         note="Faster and more aggressive: the thrust margin and the gimbal "
              "start to matter more than the battery.",
         values=dict(t_f=1200.0, m_screen=0.2, screen_w=0.2, screen_h=0.13,
                     v_follow=3.0, a_follow=2.5, lam=2.5, wind_gust=1.5)),
    dict(name="DESK-1, the designed machine",
         note="The answer this engine gives when asked for something buildable and "
              "usable: a 1.46 kg quad on 490 mm two-blade paddle props turning 963 rpm, "
              "51 dBA at 1 m, 34 minutes, 1.25 m span. Noise, not energy, set every "
              "number in it.",
         values=dict(
             # payload: a bare 13.3 inch panel, not a portable monitor
             m_screen=0.20, m_electronics=0.112, m_gimbal=0.085,
             P_screen=6.0, P_computer=5.0, P_sensors=3.0,
             screen_w=0.294, screen_h=0.166, r_cp=0.13,
             # rotors: large, slow, wide-chord, run below the stall margin
             N_rotors=4, D_rotor=0.490, n_blades=2, blade_AR=3.0,
             ct_sigma_design=0.12, tip_speed_auto=1, Cd0=0.020,
             lam=1.8, eta=0.78,
             # structure
             arm_radius=0.008, rotor_gap=1.10, m_frame_fixed=0.12,
             guard_mass_k=0.025,
             # 4S1P 18650 high-energy cells
             e_b_wh_kg=220.0, dod=0.95, soc_reserve=0.12, p_b_w_kg=1150.0,
             # mission
             t_f=1800.0, standoff=0.70, standoff_z=1.50, d_safe=0.45,
             v_follow=1.2, a_follow=1.0,
             # gains sized to the machine's own control authority
             K_R=41.4, K_w=12.9, Kp_pos=4.17, Kd_pos=4.08, Ki_pos=0.42,
         )),
    dict(name="DESK-1/W, full 1920x1080 desktop",
         note="A 15.6 inch laptop panel gives the whole 1905 x 911 working area at "
              "69 px/deg, and costs almost nothing over the 13.3: 1.60 kg against "
              "1.63. Use a laptop panel, never a monitor panel.",
         values=dict(
             m_screen=0.28, P_screen=7.5, m_electronics=0.112, m_gimbal=0.085,
             P_computer=5.0, P_sensors=3.0,
             screen_w=0.345, screen_h=0.194, r_cp=0.13,
             N_rotors=4, D_rotor=0.492, n_blades=2, blade_AR=3.0,
             ct_sigma_design=0.12, tip_speed_auto=1, Cd0=0.020,
             lam=1.8, eta=0.78,
             arm_radius=0.008, rotor_gap=1.10, m_frame_fixed=0.12,
             guard_mass_k=0.025,
             e_b_wh_kg=220.0, dod=0.95, soc_reserve=0.12, p_b_w_kg=1150.0,
             # The standoff is to the screen, but the rotors reach 0.63 m back
             # toward you from the machine centre. At 0.70 m the blade tips sit
             # 0.15 m from your eye; 1.20 m puts them 0.58 m away and holds
             # above the safe distance even with tracking lag. It costs
             # apparent screen size and nothing else, and it is 2 dB quieter.
             t_f=1800.0, standoff=1.20, standoff_z=1.50, d_safe=0.45,
             screen_above=1, screen_forward=0.0,
             v_follow=1.2, a_follow=1.0,
             K_R=41.4, K_w=12.9, Kp_pos=4.17, Kd_pos=4.08, Ki_pos=0.42,
         )),
    dict(name="DESK-1/T, iPad Pro 13 flown whole",
         note="The complete tablet brings its own compute, radio and battery, which "
              "is much simpler to build. The 579 g costs 3.5 dB and 40 W against a "
              "bare laptop panel.",
         values=dict(
             m_screen=0.579, P_screen=0.0, m_electronics=0.077, m_gimbal=0.085,
             P_computer=1.0, P_sensors=3.0,
             screen_w=0.2816, screen_h=0.2152, r_cp=0.13,
             N_rotors=4, D_rotor=0.497, n_blades=3, blade_AR=3.81,
             ct_sigma_design=0.12, tip_speed_auto=1, Cd0=0.020,
             lam=1.8, eta=0.78,
             arm_radius=0.008, rotor_gap=1.10, m_frame_fixed=0.12,
             guard_mass_k=0.025,
             e_b_wh_kg=220.0, dod=0.95, soc_reserve=0.12, p_b_w_kg=1150.0,
             t_f=1800.0, standoff=0.70, standoff_z=1.50, d_safe=0.45,
             v_follow=1.2, a_follow=1.0,
             K_R=41.4, K_w=12.9, Kp_pos=4.17, Kd_pos=4.08, Ki_pos=0.42,
         )),
    dict(name="Windy balcony",
         note="Outdoors with gusts. Watch the saturation fraction and the "
              "screen tilt on the Flight tab.",
         values=dict(t_f=1200.0, m_screen=0.25, wind_mean=3.0, wind_gust=2.0,
                     lam=2.5)),
]


def preset_params(name: str) -> Dict[str, float]:
    for pr in PRESETS:
        if pr["name"] == name:
            return clamp_params({**DEFAULTS, **pr["values"]})
    raise KeyError("unknown preset: %s" % name)


def describe(key: str) -> Dict[str, Any]:
    return _BY_KEY[key]


def make_params(**overrides: float) -> Dict[str, float]:
    """Defaults merged with overrides. Unknown keys are rejected loudly."""
    unknown = set(overrides) - set(DEFAULTS)
    if unknown:
        raise KeyError("unknown parameter(s): %s" % sorted(unknown))
    p = dict(DEFAULTS)
    p.update({k: v for k, v in overrides.items() if v is not None})
    return p


def clamp_params(p: Dict[str, float]) -> Dict[str, float]:
    """Clip to schema ranges, keeping integer fields integral."""
    out = dict(p)
    for e in _SCHEMA:
        k = e["key"]
        if k not in out:
            out[k] = e["default"]
            continue
        try:
            v = float(out[k])
        except (TypeError, ValueError):
            out[k] = e["default"]
            continue
        v = min(max(v, float(e["lo"])), float(e["hi"]))
        if e.get("even"):
            v = 2 * round(v / 2.0)
            v = min(max(v, float(e["lo"])), float(e["hi"]))
        out[k] = int(round(v)) if e.get("integer") else v
    return out


# ---------------------------------------------------------------------------
# Derived quantities used everywhere
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Derived:
    """Quantities that follow from raw parameters without any closure."""
    A: float          # total disk area                          [m^2]
    A_rotor: float    # single rotor disk area                   [m^2]
    e_b: float        # usable specific energy                   [J/kg]
    m_fix: float      # fixed mass (payload + compute + gimbal)  [kg]
    P_aux: float      # non-propulsive electrical power          [W]
    A_screen: float   # screen area                              [m^2]


def derived(p: Dict[str, float]) -> Derived:
    import math
    N = float(p["N_rotors"])
    D = float(p["D_rotor"])
    A_rotor = math.pi * D * D / 4.0
    return Derived(
        A=N * A_rotor,
        A_rotor=A_rotor,
        e_b=float(p["e_b_wh_kg"]) * J_PER_WH * float(p["dod"]),
        m_fix=float(p["m_screen"]) + float(p["m_electronics"]) + float(p["m_gimbal"]),
        P_aux=float(p["P_screen"]) + float(p["P_computer"]) + float(p["P_sensors"]),
        A_screen=float(p["screen_w"]) * float(p["screen_h"]),
    )
