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


# ---------------------------------------------------------------------------
# Stage 2, piece 2: strip VELOCITY and ANGLE OF ATTACK.
# These need the live MuJoCo state (the wing must be moving to have a velocity),
# so unlike the functions above they take (model, data). Theory: Parts 1, 5, 10.
# ---------------------------------------------------------------------------

def wing_frame(data, wing_bid):
    """The wing's orientation in the world, as three unit vectors.

    MuJoCo stores each body's world rotation as a flat 9-vector (xmat); its
    columns are the body's local axes expressed in world coordinates. By the
    convention baked into flyer.xml:
        local x -> chord direction   (leading<->trailing)
        local y -> span direction    (root->tip)
        local z -> wing-face normal   (the flat face)
    Returns (origin_world, span_hat, chord_hat, normal_hat).
    """
    xpos = data.xpos[wing_bid].copy()
    R = data.xmat[wing_bid].reshape(3, 3)
    chord_hat = R[:, 0].copy()
    span_hat = R[:, 1].copy()
    normal_hat = R[:, 2].copy()
    return xpos, span_hat, chord_hat, normal_hat


def strip_kinematics(model, data, wing_bid, r):
    """Velocity and angle of attack of every blade-element strip.

    For each spanwise station r we take the strip's reference point ON the
    pitch axis (local x=0 == quarter-chord). Evaluating velocity there means
    the wing's own feathering rotation contributes nothing (a point on the
    rotation axis has zero velocity from that rotation), so we get the clean
    TRANSLATIONAL velocity (stroke + body motion) — exactly what sets the
    angle of attack. The pitch RATE is handled separately by later force terms.

    Returns a dict of arrays (one row per strip):
        v        : world velocity of the strip point            (n,3)
        v_perp   : that velocity with the spanwise part removed  (n,3)
        speed    : |v_perp|                                       (n,)
        alpha    : angle of attack in radians, 0..pi/2            (n,)
        span/chord/normal : the wing frame unit vectors (world)   (3,)
    """
    r = np.atleast_1d(np.asarray(r, dtype=float))
    xpos, span, chord, normal = wing_frame(data, wing_bid)
    xipos = data.xipos[wing_bid].copy()       # body CoM (velocity reference!)

    # wing body spatial velocity in world: vel6 = [angular(3), linear(3)].
    # NOTE: mj_objectVelocity references the LINEAR part at the body's CoM,
    # so the rigid-body velocity of any point p is  v_com + omega x (p - CoM).
    vel6 = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                             wing_bid, vel6, 0)
    omega, v_com = vel6[:3], vel6[3:]

    pts = xpos[None, :] + np.outer(r, span)              # strip points (n,3)
    v = v_com[None, :] + np.cross(np.broadcast_to(omega, pts.shape),
                                  pts - xipos[None, :])   # world velocity (n,3)

    v_span = (v @ span)[:, None] * span[None, :]          # spanwise component
    v_perp = v - v_span                                   # in-plane velocity
    speed = np.linalg.norm(v_perp, axis=1)

    # angle of attack: angle of v_perp away from the chord line, in [0, pi/2].
    # |normal-component| / |chord-component|. Sign of lift is handled later by
    # explicit direction vectors, NOT by the sign of alpha (avoids the trap).
    comp_n = np.abs(v_perp @ normal)
    comp_c = np.abs(v_perp @ chord)
    alpha = np.arctan2(comp_n, comp_c)

    return dict(v=v, v_perp=v_perp, speed=speed, alpha=alpha,
                span=span, chord=chord, normal=normal)