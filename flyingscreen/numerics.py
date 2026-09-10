"""Bounded scalar root searches with explicit, non-certifying failure status."""
import math
import numpy as np
from scipy.optimize import brentq, minimize_scalar


def first_mass_root(residual, lower=1e-3, upper=1e5):
    """Find a light root, refining sampled local maxima before giving up.

    The component models are piecewise smooth. Failure to locate a root is
    a bounded numerical search failure, not a proof of global infeasibility.
    """
    xs, ys = [lower], [float(residual(lower))]
    while xs[-1] < upper:
        x = min(xs[-1] * 1.35, upper)
        y = float(residual(x))
        if not math.isfinite(y):
            return None, "nonfinite_residual"
        xs.append(x); ys.append(y)
        if y >= 0:
            break
    # Resolve an intervening peak, including a positive island narrower than
    # the grid. Work from low mass upward to select the light branch.
    candidates = list(zip(xs, ys))
    for i in range(1, len(xs)-1):
        if ys[i] >= ys[i-1] and ys[i] >= ys[i+1]:
            opt = minimize_scalar(lambda z: -residual(math.exp(z)),
                                  bounds=(math.log(xs[i-1]), math.log(xs[i+1])),
                                  method="bounded", options={"xatol": 1e-12})
            if opt.success:
                candidates.append((math.exp(opt.x), -float(opt.fun)))
    candidates.sort()
    for (a, fa), (b, fb) in zip(candidates, candidates[1:]):
        if fa <= 0 <= fb:
            return float(brentq(residual, a, b, xtol=1e-11, rtol=1e-12)), "root_found"
    # A tangent root is resolved only to numerical residual tolerance.
    for x, y in candidates:
        if abs(y) <= 1e-10 * max(x, 1.0):
            return x, "near_tangent_root"
    return None, "search_exhausted"


def controllability_rank(A, B, tolerance=1e-9):
    """Orthogonal reachable-subspace iteration, avoiding huge powers of A."""
    A, B = np.asarray(A), np.asarray(B)
    A = A / max(np.linalg.norm(A, 2), 1.0)
    B = B / max(np.linalg.norm(B, 2), 1e-30)
    Q = np.zeros((len(A), 0))
    for _ in range(len(A)):
        M = np.hstack((B, Q, A @ Q))
        U, s, _ = np.linalg.svd(M, full_matrices=False)
        rank = int(np.sum(s > tolerance))
        if rank == Q.shape[1]:
            return rank
        Q = U[:, :rank]
    return Q.shape[1]
