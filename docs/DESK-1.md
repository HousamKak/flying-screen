# DESK-1: a flying screen that is actually buildable

A design produced by running the engine in this repository against honest,
component-level assumptions, and then flying the result.

Load it in the tool as the starting point **"DESK-1, the designed machine"**.

> This document records the first design pass (a 13.3 inch panel at a 0.70 m
> standoff). The safety analysis that followed moved the standoff to 1.20 m
> and the design to the 15.6 inch DESK-1/W with a thin-client payload, and
> an independent review corrected several of the engine's models. The
> current numbers are in `paper/results/results.json` and the paper in
> `paper/`; where this file and the paper disagree, the paper is current.

---

## 1. What the requirements turned out to be

The original question was whether the mass closure permits a flying screen.
It does, comfortably, at short sessions. That is not what stops the product.

Working the human-facing requirements backwards gives three limits:

| Requirement | Why | Limit |
| --- | --- | --- |
| Readability | A 13.3 in panel at 0.70 m subtends 24 degrees, against 30 for a laptop at desk distance | screen 294 x 166 mm, standoff 0.70 m |
| Noise | You cannot think beside a vacuum cleaner. Quiet office is 45 dBA, speech 60 | under about 55 dBA at 1 m |
| Downwash | Paper starts moving off a desk around 5 m/s | wake under about 6.5 m/s |
| Size | Has to live beside a desk in a room | span under about 1.25 m |

All four pull the same way: **large, slow, lightly loaded rotors**. And the
binding one is noise, not energy. That inverts the conclusion of the original
theory work, where endurance looked like the whole problem.

## 2. Three corrections the design work forced on the engine

The first pass produced designs that looked compliant and were not. Three
model constants were wrong, and each was worth more than any design choice.

**The acoustic anchor was 14 dB optimistic.** EU 2019/945 makes
manufacturers declare a sound *power* level, not a pressure at a distance;
converting with `L_p = L_WA - 10 log10(2 pi r^2)` and running four DJI
aircraft back through the broadband exponents gives an anchor of 82.9 dB,
against the 68 that had been guessed. Four aircraft spanning 3.6x in mass
agree to within 1.4 dB. Everything the engine said about noise before this
was wrong by the width of the entire design space.

**Motor torque density was too pessimistic and guard mass too heavy.**
Calibrating motor mass against the T-Motor MN2806, MN3110 and MN505-S puts
peak torque density at 5.5 N.m/kg for a 100 g motor, scaling as mass^0.3.
Guard rings scale with circumference, not disk area, at about 25 g/m for a
6 mm carbon tube plus mounts.

**Blade profile drag needed a Reynolds correction.** Once the design is
pushed to 25 m/s tip speed for quietness, the blade sits near Re 90,000,
and a constant Cd0 would have been a free lunch that does not exist.

## 3. The design

A four-rotor machine with unusually large, unusually slow, unusually
wide-chord propellers, carrying a bare laptop panel on a gimbal.

### Geometry

| | |
| --- | --- |
| Rotors | 4, flat X |
| Rotor diameter | 490 mm (19.3 in) |
| Blades | 2 per rotor, 82 mm chord, aspect ratio 3.0 |
| Solidity | 0.212 (cascade limit 0.25) |
| Arm length | 381 mm hub to rotor |
| Span | 1.25 m tip to tip, folding for transport |
| Screen | 294 x 166 mm, 13.3 in, on a 3-axis gimbal |

### Mass budget, 1.463 kg gross

| Item | Mass | Note |
| --- | --- | --- |
| Screen panel + driver board | 200 g | bare eDP panel, not a portable monitor |
| Compute, depth camera, FC, radio | 112 g | OAK-D-Lite class, 61 g of it the camera |
| Gimbal | 85 g | three axes, see section 5 |
| **Fixed** | **397 g** | |
| Arms | 74 g | 16 mm carbon tube, 0.6 mm wall |
| Guards | 154 g | full rings, not optional |
| Hub, mounts, gear, wiring | 160 g | |
| **Frame** | **388 g** | implied f_s 0.265 |
| Motors | 212 g | 53 g each |
| ESCs | 19 g | 20 A 4-in-1 |
| Propellers | 219 g | 55 g each |
| **Propulsion** | **450 g** | implied S_m 57 N/kg |
| Battery | 228 g | 4S1P 18650, 50.4 Wh |

### Operating point

| | Hover | Full thrust |
| --- | --- | --- |
| Thrust per rotor | 3.59 N (366 g) | 6.46 N |
| Rotor speed | 963 rpm | 1292 rpm |
| Tip speed | 24.7 m/s | 33.1 m/s |
| Shaft torque | 0.135 N.m | 0.241 N.m |

Blade loading Ct/sigma is 0.120 against a stall limit of 0.14, so 17 percent
margin. Blade Reynolds number 94,000, corrected Cd0 0.023. Figure of merit
comes out at **0.733**, which is good, and is a consequence of the low tip
speed rather than an assumption.

### Power and endurance

| | |
| --- | --- |
| Ideal induced power | 40.0 W |
| Induced with tip loss | 46.0 W |
| Profile power | 8.6 W |
| Shaft power | 54.6 W |
| Electrical, rotors | 70.0 W |
| Screen, compute, sensors | 14.0 W |
| **Total hover** | **84.0 W** |
| Session, nominal | 34 min to 12 percent reserve |
| Session, worst mission | 26 min |

### Human factors

| | |
| --- | --- |
| **Noise at 1 m** | **51 dBA** |
| Disk loading | 19.0 N/m^2 |
| Wake velocity | 5.57 m/s (Beaufort 3) |
| Standoff | 0.70 m, screen subtends 24 degrees |

51 dBA is a desk fan on medium. You notice it, and you can work beside it.
It is roughly 24 dB below a Mavic 3 at the same distance, which is a factor
of 250 in acoustic power, and essentially all of it comes from turning a big
rotor slowly.

## 4. The electrical chain, which the mass model does not see

Torque sets motor size, but voltage and KV decide whether it can be driven
at all, and a 963 rpm rotor needs an unusual motor.

* **Pack: 4S1P 18650 high-energy cells** (Samsung 35E class), 227 g, 50.4 Wh.
  21700 cells store more per gram but come in 70 g steps, and 3S of them has
  too little voltage while 4S is over the mass budget.
* **Motor: about 98 KV**, sized so full rotor speed is reached at the
  *minimum* pack voltage of 13.2 V, not the full 16.8. Hover then sits at 68
  to 74 percent throttle across the discharge.
* **Current: 1.1 A per motor at hover**, 4.4 A total. Continuous pack
  discharge 368 W/kg against a cell rating near 510; peak 734 against 1280.
* A 98 KV wind on a 53 g stator is a fine-wire, many-turn winding of about
  1 to 2 ohm phase resistance. **This is not a catalogue part.** Expect a
  custom or rewound motor. That is the direct price of quietness.

The 490 mm two-blade propeller at 82 mm chord is likewise a paddle, not a
standard 19 inch prop, and will need its own mould.

## 5. What the flight simulation changed

Flying the machine, rather than sizing it, changed two things.

**Gains have to be sized to the control authority, not chosen by habit.**
This machine can produce 19 rad/s^2 in roll and 18 in pitch. The default
attitude gain was demanding 24 at a 30 degree error, so it saturated. Sizing
`K_R = alpha_available / e_max` gives K_R 41, K_w 12.9, and turnaround
saturation drops from 19 percent to 4.6.

**Chasing a face rigidly is the wrong specification.** Asking the screen to
stay 0.7 m in front of someone's eyes while they turn 180 degrees demands
10.4 m/s^2 and whips 1.5 kg past their head. Raising the thrust margin makes
it *worse*: at lambda 2.0 the display goes 15 degrees off upright instead of
7. The fix is a rate-limited reference: cap the reference at 2.5 m/s and
2.5 m/s^2 and let the screen fall behind and catch up.

With that in place:

| Mission | Power | Lag rms | Lag max | Screen tilt | Saturation |
| --- | --- | --- | --- | --- | --- |
| Standing | 84.0 W | 2 mm | 3 mm | 0.00 deg | 0 % |
| Pacing | 84.3 W | 69 mm | 99 mm | 0.21 deg | 0 % |
| Walking a loop | 84.8 W | 109 mm | 165 mm | 0.83 deg | 0 % |
| Jogging | 86.1 W | 116 mm | 202 mm | 1.38 deg | 0 % |
| Turning around | 98.4 W | 685 mm | 1.39 m | 4.62 deg | 4.6 % |
| Sit to stand | 84.0 W | 3 mm | 8 mm | 0.00 deg | 0 % |

Disturbance rejection is a non-issue indoors. Even a 3 m/s breeze, far more
than any room, gives 77 mm of lag and 0.61 degrees of screen tilt while the
airframe leans 12 degrees. The gimbal is doing exactly what it is for.

**The gimbal needs three axes, not two.** Yaw authority is 1.5 rad/s^2,
against 19 and 18 in roll and pitch, because yaw comes from rotor reaction
torque and that is precisely the quantity a quiet slow rotor has least of.
The airframe cannot turn to face you quickly. Put yaw on the gimbal, where
one 30 g motor solves it, and let the airframe keep whatever heading it
likes.

## 6. Two modes, one airframe

The tether turns out to be nearly free, because at 84 W the current is small
enough that the conductor is set by handling, not resistance.

| | Roam | Desk |
| --- | --- | --- |
| Energy source | 228 g battery | 5 m tether, 21 g total, 10 g carried |
| Gross mass | 1.463 kg | 1.222 kg |
| Hover power | 84.0 W | 67.6 W |
| **Noise at 1 m** | **51.0 dBA** | **47.9 dBA** |
| Wake | 5.57 m/s | 5.09 m/s |
| Endurance | 34 min | unlimited |

The tether is **0.13 mm^2 per leg at 48 V**, drawing 1.58 A and losing 8.1 W.
That is thinner than a headphone cable, and 48 V keeps it under the 60 V DC
safety-extra-low-voltage line. In desk mode the battery shrinks to a 90
second landing reserve.

Since the airframe is sized by the heavier roam case, desk mode is strictly
better on every axis: lighter, quieter, gentler, and it never stops. **The
tether is not a fallback, it is the primary mode**, with the battery there
for when you walk away from the desk.

## 7. Which screen

"1905 x 911" is a working area, not a payload: it is the CSS viewport of a
maximised window on a 1920 x 1080 desktop, and half a dozen panels deliver
it. Readability does not choose between them.

| Display | Diagonal | Size | Wide at 0.7 m | px/deg | Areal density |
| --- | --- | --- | --- | --- | --- |
| 13.3 in laptop panel | 13.3 in | 294 x 166 mm | 23.7 deg | 81 | 4.1 kg/m^2 |
| **15.6 in laptop panel** | 15.6 in | 345 x 194 mm | 27.7 deg | **69** | 4.2 kg/m^2 |
| iPad Pro 13, whole | 14.0 in | 282 x 215 mm | 22.7 deg | 121 | 9.6 kg/m^2 |
| iPad Pro 13, display only | 14.0 in | 282 x 215 mm | 22.7 deg | 121 | 5.0 kg/m^2 |
| 21.5 in desktop panel | 21.5 in | 476 x 268 mm | 37.6 deg | 51 | 9.8 kg/m^2 |
| 24 in desktop panel | 24.0 in | 531 x 299 mm | 41.5 deg | 46 | 12.3 kg/m^2 |

Every one clears 40 px/deg, the comfortable threshold, at a 0.70 m standoff.
What separates them is the last column: **a desktop monitor panel is three
times heavier per unit area than a laptop panel**, because it carries thicker
glass, a heavier backlight and a metal chassis.

With the aircraft re-optimised around each, under identical limits:

| Display | Gross | Power | dBA at 1 m | In the room | Span | Verdict |
| --- | --- | --- | --- | --- | --- | --- |
| 13.3 in laptop panel | 1.629 kg | 97.9 W | 51.1 | 52.1 | 1.23 m | closes |
| **15.6 in laptop panel** | **1.602 kg** | **95.9 W** | **52.5** | **53.5** | **1.26 m** | **closes** |
| iPad Pro 13, whole | 2.247 kg | 135.2 W | 56.0 | 57.0 | 1.27 m | closes |
| iPad Pro 13, display only | 1.640 kg | 97.9 W | 52.8 | 53.8 | 1.26 m | closes |
| 21.5 in desktop panel | 4.225 kg | 315.7 W | 62.2 | 63.2 | 1.45 m | fails span and noise |
| 24 in desktop panel | 5.791 kg | 491.8 W | 66.9 | 67.9 | 1.45 m | fails badly |

**You can have the full 1920 x 1080 desktop, as long as it is a laptop panel
and not a monitor panel.** The 15.6 inch version is essentially free against
the 13.3: 27 g lighter, 1.4 dB louder, 30 mm more span, and it gives 27.7
degrees of screen instead of 23.7. It is the one to build, and it is in the
tool as **DESK-1/W**.

The iPad flown whole deserves its own line because it is much the easiest
thing to actually build: the tablet brings its own compute, radio and
battery, deleting the driver board, the video link and most of the companion
computer. That convenience costs 645 g, 3.5 dB and 40 W. Stripping the
display assembly out of it recovers nearly all of that, at the price of
having to build everything the tablet was doing for you.

Desktop panels are out by a wide margin, and the reason is worth keeping: it
is construction, not area. A 21.5 inch panel is 3.7 times the area of a 13.3
but 6.3 times the mass.

## 8. The acoustics, written out

The engine now carries the equations rather than one number, because one
number was hiding two things.

**Source.** Broadband loading noise radiates as a dipole, so acoustic power
goes as the sixth power of the characteristic velocity:

```
L_p(1 m) = C + 60 log10(V_tip / 100) + 10 log10(T_rotor / 5) + 10 log10(N)
         = 82.9   - 35.4   - 1.0   + 6.0   =  52.5 dBA
```

at 25.7 m/s tip speed and 3.93 N per rotor over four rotors. The sixth power
is the entire design lever: halving tip speed is 18 dB.

**Sound power.** A datasheet declares `L_WA`, a property of the source alone.
Over a reflecting plane the pressure at 1 m is

```
L_p(1 m) = L_WA + 10 log10( Q / 4 pi r^2 ) = L_WA - 8.0 dB      (Q = 2)
```

so this machine would declare **60.5 dBA**, against 83 for a Mavic 3.

**The room, which is what the free-field number leaves out.** Indoors the
level is a direct field plus a reverberant field that does not fall off with
distance at all:

```
L_p(r) = L_W + 10 log10( Q / (4 pi r^2)  +  4 / R ),   R = S a / (1 - a)
r_c    = sqrt( Q R / 16 pi )
```

For an ordinary furnished 5 x 4 x 2.4 m room, `S` = 83 m^2, `a` = 0.20,
`R` = 20.8 m^2, and the critical distance is 0.91 m. Then:

| Where | Level |
| --- | --- |
| Free field at 1 m, the figure usually quoted | 52.5 dBA |
| **At your ear, 0.70 m, in the room** | **57.6 dBA** |
| Anywhere else in the room | 53.5 dBA |

**That correction is 5 dB and it goes the wrong way.** You sit just inside
the critical distance, so you get the direct field plus a reverberant field
with nowhere to go. Room treatment barely helps at your ear: carpet and
curtains buy 0.9 dB, full acoustic treatment 1.5 dB, because at 0.70 m the
direct path dominates. What treatment fixes is the rest of the room, for
everyone else in it.

**Tones, which the broadband number does not contain at all.** Blade passage
is

```
f_BPF = n_blades * rpm / 60 = 2 x 998 / 60 = 33 Hz
```

A-weighting discounts 33 Hz by 38 dB, which flatters the dBA figure a lot.
The harmonics at 67, 100 and 133 Hz are discounted by 25, 19 and 15 dB. The
energy has not gone anywhere: it is low frequency sound, which rooms absorb
poorly and people feel as a throb rather than hear as a hiss. Four rotors at
slightly different speeds also beat against one another at the difference
frequency, which is perceptually worse than a steady tone. **None of that is
in 57.6 dBA**, and it is why a measured prototype could still be unpleasant
at a level the model calls acceptable.

## 9. Beyond the actuator disk

Three of the gaps flagged in the first pass are now modelled, and the fourth
is measured and left standing.

**Ground effect**, Cheeseman and Bennett:

```
T_IGE / T_OGE |_P = 1 / (1 - (R / 4z)^2)
```

The rotor plane sits 1.63 m above the floor with a 0.246 m radius, so
`(R/4z)^2` is 0.0014 and the effect is **0.14 %**. Negligible, as the inverse
square dependence guarantees for a small rotor at head height.

**Ceiling effect**, the same form with its own coefficient, because a ceiling
restricts the rotor's inflow rather than its wake and is stronger at equal
spacing. At 0.77 m below a 2.4 m ceiling it is worth **1.3 %** of induced
power. That coefficient is the least certain constant in the engine, so it is
exposed as a parameter.

**Rotor-to-rotor interference.** With `rotor_gap` at 1.10 the tip-to-tip gap
is 49 mm, a tenth of a diameter, which is close. It costs **2.5 %** more
induced power.

Net effect on the design:

| | Gross mass | Hover power | dBA at 1 m |
| --- | --- | --- | --- |
| With both | 1.602 kg | 95.9 W | 52.50 |
| No surface effects | 1.611 kg | 97.5 W | 52.59 |
| No interference | 1.588 kg | 93.2 W | 52.34 |
| Neither | 1.596 kg | 94.7 W | 52.43 |

**They largely cancel, and the whole span is 0.25 dB and 23 g.** The caveat
was worth quantifying, but it was not hiding a different aircraft. Being able
to say that with a number is the point.

**Recirculation is the one that matters, and it is still not modelled.** The
rotors move 2.21 m^3/s. A 5 x 4 x 2.4 m room holds 48 m^3.

> Every cubic metre of air in the room passes through the rotor disk once
> every **22 seconds**.

After the first minute of a thirty minute session the machine is flying in
air it has already been through, and the still undisturbed inflow that
momentum theory assumes has quietly stopped being true. There is also a mean
return flow of about 0.11 m/s across the whole floor plan, which you would
feel over a working session even though it is far below the 5.8 m/s wake
figure. Doing this properly needs CFD, not a correction factor. It is now
the largest remaining aerodynamic gap, alongside the absence of blade element
momentum theory.

## 10. What this design does not answer

* **A quad cannot survive a motor failure.** It loses a quarter of its lift
  and all yaw balance at once, and falls from 1.5 m onto or beside the
  person it was following. A hexacopter can shed the opposite rotor and fly
  on, but it needs lambda 2.0 to do so, and when optimised for the same
  limits it came out heavier, louder, and with *less* pitch authority than
  the quad (11 rad/s^2 against 18) because two of its six rotors contribute
  nothing to pitch. It could not fly the turnaround at all. **The failure
  case needs a different answer than more rotors** - most likely a ballistic
  tether or simply never flying above head height.
* **Rotor aerodynamics are still momentum theory plus one profile term.**
  Ground effect, ceiling effect and rotor-to-rotor interference are now
  modelled (section 9) and between them are worth 0.25 dB, so that part of
  the caveat is closed and quantified. What remains open is blade element
  momentum theory, and recirculation: the machine turns the room's air over
  every 22 seconds, which no correction factor fixes.
* **The acoustic model ranks designs; it does not certify them.** The source
  term is a broadband scaling law fitted to four aircraft, and the room
  model is standard architectural acoustics with a single average absorption
  coefficient, no frequency bands and no modal behaviour below the Schroeder
  frequency, which for a room this size is around 200 Hz and therefore right
  where the blade harmonics live. Tonal content is computed (section 8) but
  not summed into the level.
* **Human tracking is assumed to work.** The mission generator provides
  perfect knowledge of where the person is. Real vision-based tracking has
  latency, dropouts and occlusion, and the reference governor is exactly
  where that error would show up.
* **The motor and the propeller are both custom parts.** Nothing else here
  needs to be.

## 8. Reproducing this

```
python -m flyingscreen.cli serve --port 8077
```

Load **"DESK-1, the designed machine"** from the starting point menu, then:

* **Machine** for the component build-up
* **Flight** with the reference governor on, mission `turnaround`, to see
  the hardest case and the control authority card
* **Tether** for the two modes
* **Optimise** to move the limits and watch the design move

From the command line:

```
python -m flyingscreen.cli design --D_rotor 0.49 --n_blades 2 --blade_AR 3.0 \
    --ct_sigma_design 0.12 --lam 1.8 --m_screen 0.20 --t_f 1800
```
