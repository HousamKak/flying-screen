"""
Flying-screen co-design engine.

Seven layers, matching the derivation:

    1. momentum theory        -> hover energy               (sizing)
    2. mass closure           -> can the design exist       (sizing)
    3. structural mechanics   -> frame mass and stiffness   (components)
    4. Newton-Euler dynamics  -> how the body moves         (dynamics)
    5. motor + battery ODEs   -> actuator and energy states (dynamics)
    6. control                -> stability and following    (control)
    7. optimisation           -> the best machine physics allows (optimize)
"""

__version__ = "0.1.0"

from . import params, sizing  # noqa: F401

__all__ = ["params", "sizing"]
