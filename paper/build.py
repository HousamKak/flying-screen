"""
Build the paper.

    python paper/build.py

Runs pdflatex, bibtex, pdflatex, pdflatex on main.tex and writes the result
as "The Flying Screen.pdf" in this directory. No main.pdf is left behind.

The numbers and figure data come from paper/results/, which is committed;
regenerate it first with `python paper/make_results.py` only if the engine
has changed.

pdflatex and bibtex are taken from LATEX_BIN if that is set, otherwise from
PATH, otherwise from the usual per-user MiKTeX location on Windows.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = "main"
OUTPUT = "The Flying Screen.pdf"


def tool(name: str) -> str:
    exe = name + (".exe" if os.name == "nt" else "")
    folder = os.environ.get("LATEX_BIN")
    if folder:
        return os.path.join(folder, exe)
    found = shutil.which(name)
    if found:
        return found
    guess = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "MiKTeX",
                         "miktex", "bin", "x64", exe)
    if os.path.exists(guess):
        return guess
    sys.exit("cannot find %s: install a TeX distribution or set LATEX_BIN" % name)


def run(cmd: list) -> None:
    subprocess.run(cmd, cwd=HERE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> None:
    pdflatex, bibtex = tool("pdflatex"), tool("bibtex")
    latex = [pdflatex, "-interaction=nonstopmode", SOURCE + ".tex"]
    for step in (latex, [bibtex, SOURCE], latex, latex):
        run(step)

    with open(os.path.join(HERE, SOURCE + ".log"), encoding="latin-1") as fh:
        log = fh.read()
    errors = [line for line in log.splitlines() if line.startswith("! ")]
    undefined = re.findall(r"(?:Reference|Citation) `[^']+' on page \d+ undefined", log)
    built = os.path.join(HERE, SOURCE + ".pdf")
    if errors or not os.path.exists(built):
        print("build failed:")
        for line in errors[:20]:
            print("  " + line)
        sys.exit(1)

    os.replace(built, os.path.join(HERE, OUTPUT))
    pages = re.search(r"Output written on \S+ \((\d+) pages", log)
    print("wrote paper/%s (%s pages)" % (OUTPUT, pages.group(1) if pages else "?"))
    if undefined:
        print("warning: %d undefined references or citations" % len(undefined))


if __name__ == "__main__":
    main()
