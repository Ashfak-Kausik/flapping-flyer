"""
src/ground_effect.py — surface (ground / wall) effect on the quasi-steady aero.

WHY THIS IS A SEPARATE, ADDED TERM (read this before trusting any result):
Our blade-element aero (src/aero.py) computes the force on each wing strip moving
through STILL air. It has no wake / induced-velocity term, so it physically
cannot know a surface is nearby — a bare geometric wall in the scene would change
nothing. Hovering ground effect is a WAKE effect: the wing drives an induced
downwash that lowers its own effective angle of attack; a nearby surface blocks
that downward wake, the induced downwash falls, effective AoA recovers, and lift
rises. We therefore ADD this effect as a literature-grounded perturbation that is
exactly 1.0 out of ground effect (so Stages 2-4 are untouched) and grows as the
surface approaches, scaling as (R/4d)^2 — the Cheeseman & Bennett (1955) leading
term; the same monotone trend is reported for flapping hover (Gao & Lu 2008;
Truong et al. 2013). The simulation does NOT derive this; we impose it. For
sim-to-real, K_GE is the coefficient to pin down in a wind tunnel.

It is applied PER STRIP from each strip's distance d to the surface, so geometry
gives directionality for free:
  * floor, level  -> all strips at ~equal height -> symmetric lift rise (Fz)
  * wall / tilt   -> strips at different distances -> asymmetric rise -> a torque
                     (a DIRECTIONAL "which way is the surface" signal)
"""
import numpy as np
from src import aero

K_GE = 1.0          # ground-effect strength; 1.0 = Cheeseman-Bennett leading term
KAPPA_MAX = 2.0     # cap per-strip enhancement (the (R/4d)^2 form blows up as d->0)


def strip_distance(pts, surface):
    """Perpendicular distance of each strip point to a planar surface.
    surface = dict(axis, sign, pos): plane at coordinate `pos` along `axis`
    (0=x,1=y,2=z); `sign`=+1 if the flyer sits on the +side. d>0 toward flyer."""
    coord = pts[:, surface["axis"]]
    return np.maximum(surface["sign"] * (coord - surface["pos"]), 1e-6)


def kappa(d, R, k=K_GE, cap=KAPPA_MAX):
    """Per-strip lift-enhancement factor: 1 far away, growing near the surface."""
    return np.minimum(1.0 + k * (R / (4.0 * d)) ** 2, cap)


def _as_list(surface):
    """None -> []; single dict -> [dict]; list/tuple -> list. Lets one or several
    surfaces (e.g. two corridor walls) be handled uniformly."""
    if surface is None:
        return []
    if isinstance(surface, dict):
        return [surface]
    return list(surface)


def kappa_pts(pts, surface, R, k=K_GE, cap=KAPPA_MAX):
    """Per-strip enhancement summed over ALL surfaces: 1 + sum_i k (R/4 d_i)^2,
    capped. Two walls each enhance their near wing; their contributions add."""
    surfs = _as_list(surface)
    if not surfs:
        return np.ones(pts.shape[0])
    enh = np.zeros(pts.shape[0])
    for s in surfs:
        enh += k * (R / (4.0 * strip_distance(pts, s))) ** 2
    return np.minimum(1.0 + enh, cap)


def _trans_ge(model, data, wing_bid, strips, surface, R, rho=aero.RHO):
    """aero.translational_force with each strip's LIFT scaled by kappa(d_strip).
    Drag is unchanged (ground effect acts through the lift/induced mechanism)."""
    k = aero.strip_kinematics(model, data, wing_bid, strips["r"])
    v_perp, sp, alpha = k["v_perp"], k["speed"], k["alpha"]
    normal, dS = k["normal"], strips["dS"]
    cl, cd = aero.coefficients(alpha)
    eps = 1e-9; safe = sp > eps
    u = np.zeros_like(v_perp); u[safe] = v_perp[safe] / sp[safe, None]
    n_lee = -np.sign(u @ normal)[:, None] * normal[None, :]
    lift_hat = n_lee - (np.sum(n_lee * u, axis=1))[:, None] * u
    ln = np.linalg.norm(lift_hat, axis=1); good = ln > eps
    lift_hat[good] /= ln[good, None]
    xpos, xipos = data.xpos[wing_bid], data.xipos[wing_bid]
    pts = xpos[None, :] + np.outer(strips["r"], k["span"])
    kap = kappa_pts(pts, surface, R)
    q = 0.5 * rho * sp ** 2 * dS
    dF = q[:, None] * (kap[:, None] * cl[:, None] * lift_hat - cd[:, None] * u)
    F = dF.sum(axis=0)
    T = np.cross(pts - xipos[None, :], dF).sum(axis=0)
    return F, T, kap


def wing_aero_ge(model, data, wing_bid, strips, vn_prev, dt, surface, R, rho=aero.RHO):
    """Full single-wing aero WITH surface effect: GE-scaled translational lift +
    unchanged rotational and added-mass terms. surface=None -> identical to
    aero.wing_aero."""
    Ft, Tt, kap = _trans_ge(model, data, wing_bid, strips, surface, R, rho)
    Fr, Tr, _ = aero.rotational_force(model, data, wing_bid, strips, rho)
    Fa, Ta, vn, _ = aero.added_mass_force(model, data, wing_bid, strips, vn_prev, dt, rho)
    return Ft + Fr + Fa, Tt + Tr + Ta, vn, dict(kappa=kap)