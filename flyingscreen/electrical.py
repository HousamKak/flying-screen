"""
The electrical chain, which the mass model does not see.

Torque sets motor size, but voltage and KV decide whether the motor can be
driven at all.  A rotor turning slowly for quietness needs a very low KV,
and the pack has to keep enough voltage headroom above the hover back-EMF
that the machine still hovers at the bottom of the discharge.

    Kt [N.m/A] = 9.5493 / KV [rpm/V]
    back-EMF at hover = rpm_hover / KV
    KV sized so full rotor speed is reached at the *minimum* pack voltage,
    not the nominal or full one, otherwise the aircraft cannot hover on a
    tired battery.
"""

from __future__ import annotations

from typing import Any, Dict, List

from . import components as comp
from .params import derived

KT_PER_KV = 9.5493

# Representative high-energy cylindrical cells, pack level.
CELLS: List[Dict[str, Any]] = [
    dict(name="18650 high energy", mass=0.048, wh=12.6, v_nom=3.6,
         v_min=3.3, v_full=4.2, w_kg_cont=430.0, w_kg_burst=1000.0),
    dict(name="21700 high energy", mass=0.070, wh=18.0, v_nom=3.6,
         v_min=3.3, v_full=4.2, w_kg_cont=510.0, w_kg_burst=1280.0),
    dict(name="LiPo pouch", mass=0.055, wh=11.5, v_nom=3.7,
         v_min=3.4, v_full=4.2, w_kg_cont=900.0, w_kg_burst=2500.0),
]

PACK_OVERHEAD = 1.18        # BMS, holder, wiring, connector


def electrical_chain(p: Dict[str, float], m: float, m_bat: float,
                     throttle_target: float = 0.80) -> Dict[str, Any]:
    """
    Pick a pack and size the motor for the rotor this design needs.

    `throttle_target` is the fraction of the *minimum* pack voltage the hover
    back-EMF is allowed to reach, which is what leaves the aircraft able to
    hover at the end of a discharge.
    """
    d = derived(p)
    N = int(round(float(p["N_rotors"])))
    pr = comp.propulsion(p, m)
    hov, mx = pr.rotor_hover, pr.rotor_max
    P_hover = N * hov.P_elec + d.P_aux
    energy_needed_wh = m_bat * float(p["e_b_wh_kg"])

    options: List[Dict[str, Any]] = []
    for cell in CELLS:
        for S in range(2, 9):
            v_min, v_full = S * cell["v_min"], S * cell["v_full"]
            v_nom = S * cell["v_nom"]
            # Full rotor speed must be reachable at the worst pack voltage.
            KV = mx.rpm / v_min
            Kt = KT_PER_KV / KV
            I_hover = hov.Q / Kt
            I_max = mx.Q / Kt
            bemf = hov.rpm / KV
            mass = S * cell["mass"] * PACK_OVERHEAD
            energy = S * cell["wh"]
            reasons = []
            if energy < energy_needed_wh * 0.92:
                reasons.append("only %.0f Wh, needs %.0f" % (energy, energy_needed_wh))
            if mass > m_bat * 1.12:
                reasons.append("%.0f g, budget is %.0f g" % (1000 * mass, 1000 * m_bat))
            if KV < 45:
                reasons.append("KV %.0f is below a practical winding" % KV)
            if P_hover / max(mass, 1e-9) > cell["w_kg_cont"]:
                reasons.append("continuous draw %.0f W/kg over the %.0f W/kg rating"
                               % (P_hover / mass, cell["w_kg_cont"]))
            options.append(dict(
                cell=cell["name"], S=S, v_nom=v_nom, v_min=v_min, v_full=v_full,
                KV=KV, Kt=Kt, I_hover=I_hover, I_max=I_max, bemf_hover=bemf,
                throttle_at_min=bemf / v_min, throttle_at_nom=bemf / v_nom,
                mass=mass, energy_wh=energy, ok=not reasons,
                reasons=reasons,
                cont_w_kg=P_hover / max(mass, 1e-9),
                cont_limit=cell["w_kg_cont"],
                burst_w_kg=pr.P_elec_max / max(mass, 1e-9),
                burst_limit=cell["w_kg_burst"],
            ))

    viable = [o for o in options if o["ok"] and o["throttle_at_min"] <= throttle_target]
    chosen = min(viable, key=lambda o: o["mass"]) if viable else None
    if chosen is None:
        viable = [o for o in options if o["ok"]]
        chosen = min(viable, key=lambda o: o["throttle_at_min"]) if viable else None

    note = None
    if chosen:
        # A very low KV in a small stator means a fine wire, many turn winding.
        if chosen["KV"] < 200 and pr.m_motor_each < 0.12:
            note = ("%.0f KV in a %.0f g stator is a fine-wire, many-turn winding: "
                    "expect a custom or rewound motor, not a catalogue part. That is "
                    "the direct price of a rotor turning at %.0f rpm."
                    % (chosen["KV"], 1000 * pr.m_motor_each, hov.rpm))
        chosen["phase_resistance"] = [
            dict(eta=e, loss_w=hov.P_shaft * (1.0 / e - 1.0),
                 r_ohm=hov.P_shaft * (1.0 / e - 1.0) / max(chosen["I_hover"] ** 2, 1e-9))
            for e in (0.80, 0.85, 0.90)
        ]

    return dict(
        chosen=chosen, options=options, note=note,
        rpm_hover=hov.rpm, rpm_max=mx.rpm,
        Q_hover=hov.Q, Q_max=mx.Q,
        P_hover_total=P_hover, P_elec_max=pr.P_elec_max,
        m_motor_each=pr.m_motor_each,
        esc_current_per_channel=mx.Q / chosen["Kt"] if chosen else None,
        energy_needed_wh=energy_needed_wh,
    )
