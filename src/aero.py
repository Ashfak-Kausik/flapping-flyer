"""
src/aero.py — quasi-steady blade-element aerodynamics for the flapping wing.

THIS FILE so far (Stage 2, piece 1): wing DISCRETISATION only.
  - wing_params_from_model() : read the wing geometry out of the MJCF, so the
                               model stays the single source of truth.
  - chord(r)                 : the elliptical chord distribution c(r).
  - build_strips()           : chop the wing into spanwise blade elements.

The force terms (translational / rotational / added-mass) layer onto these
strips in the next pieces. Theory: docs/02_aerodynamics.md, Part 5.
"""
import numpy as np
import mujoco


def wing_params_from_model(model, geom_name="wing_R_g"):
    """Pull the wing ellipsoid's geometry out of the loaded MuJoCo model.

    The wing geom is an ellipsoid with size = (chord/2, span/2, thickness/2)
    in the wing's local frame, centred at local (x_off, y_center, 0).
    Returns everything the blade-element code needs, in metres.
    """
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    size = model.geom_size[gid]          # semi-axes (a_x, a_y, a_z)
    pos = model.geom_pos[gid]            # geom centre in the wing local frame
    c_max = 2.0 * size[0]                # max chord (full extent along x)
    span_semi = size[1]                  # half-span of the ellipse (along y)
    y_center = pos[1]                    # spanwise centre of the wing area
    return dict(
        c_max=c_max,
        span_semi=span_semi,
        y_center=y_center,
        r_root=y_center - span_semi,     # innermost span station with area
        r_tip=y_center + span_semi,      # wing tip (= R, hinge-to-tip)
    )


def chord(r, c_max, y_center, span_semi):
    """Local chord c(r) of the elliptical wing at spanwise station r.

    Ellipse planform:  (x/(c_max/2))^2 + ((r - y_center)/span_semi)^2 = 1.
    Solving for the chord (full x-extent) at span r:
        c(r) = c_max * sqrt(1 - ((r - y_center)/span_semi)^2)
    Outside the wing (|r - y_center| > span_semi) the chord is zero.
    Accepts scalars or NumPy arrays.
    """
    u = (np.asarray(r, dtype=float) - y_center) / span_semi
    return c_max * np.sqrt(np.clip(1.0 - u * u, 0.0, None))


def build_strips(c_max, y_center, span_semi, n_strips=20):
    """Chop the wing into n_strips spanwise blade elements (Part 5).

    Returns a dict of equal-length arrays, one value per strip:
        r  : spanwise centre of the strip, measured from the stroke hinge [m]
        dr : strip width (uniform) [m]
        c  : local chord at r [m]
        dS : strip area = c*dr [m^2]
    """
    r_root = y_center - span_semi
    r_tip = y_center + span_semi
    edges = np.linspace(r_root, r_tip, n_strips + 1)
    r = 0.5 * (edges[:-1] + edges[1:])           # strip centres
    dr = (r_tip - r_root) / n_strips
    c = chord(r, c_max, y_center, span_semi)
    dS = c * dr
    return dict(r=r, dr=dr, c=c, dS=dS)