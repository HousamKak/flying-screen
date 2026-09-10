"""
Display options, as real parts.

A resolution is not a payload.  "1905 x 911" is a working area - the CSS
viewport of a maximised window on a 1920 x 1080 desktop - and it can be
delivered by anything from a 13 inch laptop panel to a 24 inch monitor.
What separates them is mass, and mass per unit area is not constant:

  * laptop eDP panels are built to be thin and light, about 3 kg/m^2
  * tablet display assemblies carry cover glass and a digitiser, about
    4 to 4.5 kg/m^2
  * desktop monitor panels carry thicker glass, a heavier backlight and a
    metal chassis, about 9 to 11 kg/m^2

which is a factor of three, and it decides the aircraft.

Readability is set by angle and pixel density, not by diagonal:

    apparent width  = 2 atan(w / 2d)                       [deg]
    pixels per degree = horizontal pixels / apparent width

Around 60 px/deg is the limit of 20/20 acuity, and text stays comfortable
down to about 40.  A 1920 wide panel therefore stays usable at 0.7 m from
about 13 inches up to about 24, so the choice is not driven by legibility.
It is driven entirely by what the rotors can carry quietly.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

# Installed mass includes the driver board and a carbon backing frame where
# the panel is bare; the tablet is flown whole, so it brings its own.
CATALOGUE: List[Dict[str, Any]] = [
    dict(
        key="laptop13",
        name="13.3 inch laptop panel",
        detail="Bare FHD eDP panel, 140 g, plus driver board and backing frame.",
        w=0.294, h=0.166, px_w=1920, px_h=1080,
        m_screen=0.200, P_screen=6.0,
        m_electronics=0.112, P_computer=5.0, P_sensors=3.0,
        self_powered=False,
    ),
    dict(
        key="laptop156",
        name="15.6 inch laptop panel",
        detail="Bare FHD eDP panel, 215 g, plus driver board and backing frame.",
        w=0.345, h=0.194, px_w=1920, px_h=1080,
        m_screen=0.280, P_screen=7.5,
        m_electronics=0.112, P_computer=5.0, P_sensors=3.0,
        self_powered=False,
    ),
    dict(
        key="laptop156_lid",
        name="15.6 inch laptop lid assembly",
        detail="The whole lid cut off at the hinge: panel, bezel, back cover, "
               "webcam and display cable, 430 g. What you get if you do not "
               "want to strip the panel out of its housing.",
        w=0.345, h=0.194, px_w=1920, px_h=1080,
        m_screen=0.470, P_screen=7.5,
        m_electronics=0.112, P_computer=5.0, P_sensors=3.0,
        self_powered=False,
    ),
    dict(
        key="laptop13_whole",
        name="13.3 inch ultrabook, flown whole",
        detail="A complete MacBook Air class machine, 1.24 kg. Self-contained: "
               "its own compute, battery and radio, and the keyboard half is "
               "dead weight you cannot see.",
        w=0.286, h=0.179, px_w=2560, px_h=1664,
        m_screen=1.240, P_screen=0.0,
        m_electronics=0.077, P_computer=1.0, P_sensors=3.0,
        self_powered=True,
    ),
    dict(
        key="laptop156_whole",
        name="15.6 inch laptop, flown whole",
        detail="A complete 15.6 inch laptop, 1.75 kg.",
        w=0.345, h=0.194, px_w=1920, px_h=1080,
        m_screen=1.750, P_screen=0.0,
        m_electronics=0.077, P_computer=1.0, P_sensors=3.0,
        self_powered=True,
    ),
    dict(
        key="ipad13",
        name="iPad Pro 13 inch, flown whole",
        detail="The complete tablet, 579 g. It brings its own compute, radio "
               "and battery, so it removes the driver board and the video link.",
        w=0.2816, h=0.2152, px_w=2752, px_h=2064,
        m_screen=0.579, P_screen=0.0,          # runs on its own cell
        m_electronics=0.077, P_computer=1.0, P_sensors=3.0,
        self_powered=True,
    ),
    dict(
        key="ipad13_bare",
        name="iPad Pro 13 inch display only",
        detail="Display assembly stripped out of the tablet, 260 g with cover "
               "glass, driven by our own board.",
        w=0.2816, h=0.2152, px_w=2752, px_h=2064,
        m_screen=0.300, P_screen=7.0,
        m_electronics=0.112, P_computer=5.0, P_sensors=3.0,
        self_powered=False,
    ),
    dict(
        key="desktop215",
        name="21.5 inch desktop panel",
        detail="Open-cell FHD monitor panel, 1.15 kg. Desktop panels are "
               "three times heavier per unit area than laptop panels.",
        w=0.476, h=0.268, px_w=1920, px_h=1080,
        m_screen=1.250, P_screen=18.0,
        m_electronics=0.112, P_computer=5.0, P_sensors=3.0,
        self_powered=False,
    ),
    dict(
        key="desktop24",
        name="24 inch desktop panel",
        detail="Open-cell FHD monitor panel, 1.80 kg.",
        w=0.531, h=0.299, px_w=1920, px_h=1080,
        m_screen=1.950, P_screen=22.0,
        m_electronics=0.112, P_computer=5.0, P_sensors=3.0,
        self_powered=False,
    ),
]

CATALOGUE += [
    dict(
        key="pi_thinclient",
        name="15.6 in panel + Pi 4, cloud desktop",
        detail="Bare 15.6 in eDP panel on a matched controller board, driven by "
               "a Raspberry Pi 4 running a remote desktop client against a "
               "cloud VM, with an on-sensor AI camera for following and a "
               "separate flight controller for the fast loops.",
        w=0.345, h=0.194, px_w=1920, px_h=1080,
        m_screen=0.280, P_screen=7.5,
        m_electronics=0.181, P_computer=7.0, P_sensors=1.5,
        self_powered=False,
    ),
    dict(
        key="pi_thinclient_13",
        name="13.3 in panel + Pi 4, cloud desktop",
        detail="The same build on a 13.3 in panel: 80 g lighter, and still the "
               "whole 1920 x 1080 desktop, just at 81 px/deg instead of 69.",
        w=0.294, h=0.166, px_w=1920, px_h=1080,
        m_screen=0.200, P_screen=6.0,
        m_electronics=0.181, P_computer=7.0, P_sensors=1.5,
        self_powered=False,
    ),
]

BY_KEY = {c["key"]: c for c in CATALOGUE}


# ---------------------------------------------------------------------------
# A concrete bill of materials for the thin-client build
# ---------------------------------------------------------------------------
# Masses are weighed parts, not estimates.  Where a part number is given it is
# a commonly stocked item; the point is the class of part and its mass, since
# panel model numbers rotate faster than this model will.
BOM_THIN_CLIENT = [
    dict(group="display", item="15.6 in FHD IPS eDP panel, 30 pin",
         part="BOE NV156FHM-N4M / Innolux N156HCA-EAA / AUO B156HAN02.1",
         mass=0.215, power=6.0,
         note="Standard laptop replacement panel, 1.8 mm, 350 nits. Buy by "
              "panel model number, not by size."),
    dict(group="display", item="eDP controller board",
         part="NT68676 or RTD2556 based, 30 pin eDP to HDMI",
         mass=0.030, power=1.5,
         note="Must be firmware-matched to the exact panel model. Buy the "
              "board from a seller who flashes it for your panel."),
    dict(group="display", item="carbon backing frame and mount",
         part="custom", mass=0.035, power=0.0,
         note="Also carries the gimbal interface."),
    dict(group="compute", item="Raspberry Pi 4 Model B, 2 GB",
         part="Raspberry Pi 4B 2GB", mass=0.046, power=5.5,
         note="Pi 4, not Pi 5: the Pi 5 dropped the hardware H.264 decoder, "
              "and a remote desktop stream is H.264. The Pi 4 decodes it in "
              "silicon at about 2 W instead of on the CPU at about 6."),
    dict(group="compute", item="microSD card and heatsink",
         part="", mass=0.015, power=0.0,
         note="No fan needed: it hangs in a 5.8 m/s downwash, which is the "
              "one place a Pi is never thermally throttled."),
    dict(group="compute", item="camera with on-sensor inference",
         part="Raspberry Pi AI Camera (IMX500), or Luxonis OAK-D-Lite",
         mass=0.025, power=1.5,
         note="Detection runs on the sensor and it outputs coordinates, not "
              "frames. The Pi must not spend CPU on vision, because it is "
              "already decoding 1080p."),
    dict(group="flight", item="flight controller with IMU and barometer",
         part="Holybro Pixhawk 6C mini, ArduCopter or PX4",
         mass=0.035, power=1.2,
         note="Separate from the Pi, and not negotiable: Linux is not real "
              "time, and a stuttering video stream must never become a gap in "
              "the attitude loop. A Matek H743-SLIM does the same job at 12 g "
              "if the 23 g matters, which at this scale it slightly does."),
    dict(group="flight", item="downward ToF rangefinder",
         part="VL53L1X or TFmini-S", mass=0.005, power=0.3,
         note="Altitude hold indoors, where GPS does not exist."),
    dict(group="flight", item="power distribution, 5 V and 12 V regulators",
         part="dual-output BEC, 5 V 4 A and 12 V 3 A", mass=0.030, power=0.0,
         note="The panel controller board wants 12 V and the Pi wants 5 V, "
              "from a pack that swings 9.0 to 12.6 V on 3S. Losses are already "
              "inside the electrical efficiency."),
    dict(group="flight", item="wiring, connectors, vibration mounts",
         part="", mass=0.025, power=0.0, note=""),
]


def bom_totals(bom=None):
    bom = bom or BOM_THIN_CLIENT
    out = {}
    for r in bom:
        g = out.setdefault(r["group"], dict(mass=0.0, power=0.0, items=[]))
        g["mass"] += r["mass"]
        g["power"] += r["power"]
        g["items"].append(r)
    out["_total"] = dict(mass=sum(r["mass"] for r in bom),
                         power=sum(r["power"] for r in bom))
    return out


# Glyph size criterion. Pixel density alone does not decide readability:
# moving a panel away raises its pixels per degree while shrinking every
# glyph. What a reader needs is text of a given angular height, and the
# human-factors guidance (ANSI/HFES 100-2007) asks for a character height of
# at least 16 arcmin, with 20 to 22 preferred. The operating system meets
# that by scaling the interface, which divides the usable desktop.
TEXT_EM_PX = 16.0          # body text em at 100 percent scaling   [px]
CAP_RATIO = 0.70           # cap height / em for common UI faces   [-]
GLYPH_MIN_ARCMIN = 16.0
GLYPH_PREF_ARCMIN = 20.0


def readability(panel: Dict[str, Any], distance: float) -> Dict[str, float]:
    """Apparent size, pixel density, and the desktop left after text scaling."""
    ang_w = 2.0 * math.degrees(math.atan(panel["w"] / (2.0 * distance)))
    ang_h = 2.0 * math.degrees(math.atan(panel["h"] / (2.0 * distance)))
    ppd = panel["px_w"] / ang_w if ang_w > 0 else 0.0
    areal = panel["m_screen"] / (panel["w"] * panel["h"])
    pitch = panel["w"] / panel["px_w"]                       # m per pixel
    cap_m = CAP_RATIO * TEXT_EM_PX * pitch
    cap_arcmin = math.degrees(math.atan(cap_m / distance)) * 60.0
    scale_min = max(1.0, GLYPH_MIN_ARCMIN / cap_arcmin)
    scale_pref = max(1.0, GLYPH_PREF_ARCMIN / cap_arcmin)
    px_h = panel.get("px_h", round(panel["px_w"] * panel["h"] / panel["w"]))
    logical_w = panel["px_w"] / scale_min
    logical_h = px_h / scale_min
    verdict = ("full desktop" if logical_w >= 1280 else
               "reduced desktop" if logical_w >= 1000 else
               "tablet-sized desktop" if logical_w >= 700 else
               "phone-sized desktop")
    return dict(angular_width=ang_w, angular_height=ang_h,
                pixels_per_degree=ppd, areal_density=areal,
                diagonal_in=math.hypot(panel["w"], panel["h"]) / 0.0254,
                area=panel["w"] * panel["h"],
                cap_arcmin_at_100=cap_arcmin, ui_scale_min=scale_min,
                ui_scale_preferred=scale_pref,
                logical_w=logical_w, logical_h=logical_h, verdict=verdict)


def apply(p: Dict[str, float], key: str) -> Dict[str, float]:
    """Return the parameter set with this display fitted."""
    c = BY_KEY[key]
    q = dict(p)
    q.update(screen_w=c["w"], screen_h=c["h"],
             m_screen=c["m_screen"], P_screen=c["P_screen"],
             m_electronics=c["m_electronics"],
             P_computer=c["P_computer"], P_sensors=c["P_sensors"])
    return q
