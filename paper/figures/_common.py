"""
Shared helpers for the figure data generators (gen_*.py).

Each generator does

    from _common import ROOT, write_table, base_params
    def generate(out_dir): ...
    if __name__ == "__main__": generate(default_out())

and writes whitespace-separated tables with a header row, the format
pgfplots reads with \\addplot table [x=..., y=...] {results/fig/name.dat}.
"""

from __future__ import annotations

import os
import sys
from typing import Dict, Iterable, List, Sequence

HERE = os.path.dirname(os.path.abspath(__file__))
PAPER = os.path.dirname(HERE)
ROOT = os.path.dirname(PAPER)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

PRESET = "DESK-1/W, full 1920x1080 desktop"
PAYLOAD = "pi_thinclient"
GOV = dict(v_max=2.5, a_max=2.5, tracking_margin=0.10, yaw_rate_max=1.5)
SIM = dict(t_window=16.0, dt=0.004, settle=3.0, seed=12345)


def base_params() -> Dict[str, float]:
    from flyingscreen import params as P, payloads
    return payloads.apply(P.preset_params(PRESET), PAYLOAD)


def default_out() -> str:
    out = os.path.join(PAPER, "results", "fig")
    os.makedirs(out, exist_ok=True)
    return out


def write_table(out_dir: str, name: str, columns: Sequence[str],
                rows: Iterable[Sequence[float]]) -> str:
    """Write name.dat with a header row; non-finite values become nan."""
    path = os.path.join(out_dir, name + ".dat")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(" ".join(columns) + "\n")
        for r in rows:
            fh.write(" ".join(_fmt(v) for v in r) + "\n")
    return path


def write_macros(out_dir: str, name: str, values: Dict[str, str]) -> str:
    """
    Write name.tex with \\providecommand definitions (letters-only names).

    \\providecommand, not \\newcommand, so that a figure file may \\input its
    macro file itself and several figures may share one without clashing.
    """
    path = os.path.join(out_dir, name + ".tex")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        for k, v in values.items():
            fh.write("\\providecommand{\\%s}{%s}\n" % (k, v))
    return path


def _fmt(v) -> str:
    try:
        x = float(v)
    except (TypeError, ValueError):
        return str(v)
    if x != x or x in (float("inf"), float("-inf")):
        return "nan"
    return "%.8g" % x
