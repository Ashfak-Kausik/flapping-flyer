"""
make_model_fig.py — three-panel vehicle figure, built ONLY from models/flyer.xml and src/.

  A  model_iso              MuJoCo offscreen render (orthographic, true isometric) + annotations
  B  model_wing_detail      one wing, top view, N_s blade-element strips + stroke-kinematics inset
  C  model_sensing_geometry top-down schematic: feeler fan, wall, kappa(d) asymmetry, tau_x^wall

Read-only: the model file and all experiments are untouched. The render uses an in-memory
MjSpec copy with VISUAL-only overrides (white skybox, floor hidden, brighter headlight,
orthographic camera, larger offscreen buffer); no geometry, mass or joint is changed.

Every number drawn is read from the model / source at run time and printed first.

Run (from repo root):  MUJOCO_GL=glfw python outputs/report/model_fig/make_model_fig.py
"""
import os, sys, inspect
os.environ.setdefault("MUJOCO_GL", "glfw")
sys.path.insert(0, ".")
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mp
import matplotlib.patheffects as pe

from src import sysid, ground_effect
from src.aero import wing_params_from_model, build_strips, chord
from src.antenna import Antenna
from src.flyer import Flyer
from src.kinematics import FlapKinematics
from src.controller import design

XML = "models/flyer.xml"
OUT = "outputs/report/model_fig"
MM = 1e3

# Things that are NOT in the model/source and therefore are choices, kept in one place:
FEELER_ANGLES = [0, 30, -30, 50, -50, 90, -90]   # copied from experiments/e48_mission.py:77 (asserted below)
EQ_KAPPA, EQ_TAUWALL = 11, 12                    # position of eq:kappa / eq:tauwall among the numbered
                                                 # equations of report_draft.md (counted below, asserted)
SIDE = {+1: "left", -1: "right"}                 # x fwd, z up  =>  +y is LEFT (right-handed frame)

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "mathtext.fontset": "dejavusans",
                     "pdf.fonttype": 42, "svg.fonttype": "none"})
K = "#111111"; G = "#555555"; LG = "#bdbdbd"
HALO = [pe.withStroke(linewidth=2.6, foreground="white")]


# --------------------------------------------------------------------------------------
# 1. read every value from the model / source
# --------------------------------------------------------------------------------------
def read_values():
    fly = Flyer(XML)
    m = fly.model
    V = {}
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    V["geoms"] = {n: dict(pos=m.geom_pos[gid(n)] * MM, size=m.geom_size[gid(n)] * MM)
                  for n in ("thorax_g", "head_g", "tail_g", "wing_R_g", "wing_L_g")}
    V["hinge"] = {w: m.body_pos[bid(f"stroke_{w}")] * MM for w in "RL"}
    V["wing"] = {w: {k: float(v) for k, v in wing_params_from_model(m, f"wing_{w}_g").items()} for w in "RL"}
    p = V["wing"]["R"]
    V["R"] = fly.R * MM
    V["c_max"] = p["c_max"] * MM
    V["x_hat0"] = p["x_hat0"]
    V["r_root"], V["r_tip"] = p["r_root"] * MM, p["r_tip"] * MM
    V["y_center"], V["span_semi"] = p["y_center"] * MM, p["span_semi"] * MM
    V["hinge_y"] = float(V["hinge"]["R"][1])
    V["tip_to_tip"] = 2 * (V["hinge_y"] + V["R"])
    V["two_R"] = 2 * V["R"]
    # strips: N_s is the default of Flyer.__init__ / build_strips, and what fly.strips actually holds
    V["N_s"] = len(fly.strips["R"]["r"])
    assert V["N_s"] == inspect.signature(Flyer.__init__).parameters["n_strips"].default \
                    == inspect.signature(build_strips).parameters["n_strips"].default
    s = build_strips(p["c_max"], p["y_center"], p["span_semi"], V["N_s"])
    assert np.allclose(s["r"], fly.strips["R"]["r"])
    V["strip_r"] = s["r"] * MM; V["strip_c"] = s["c"] * MM; V["dr"] = float(s["dr"]) * MM
    V["edges"] = np.linspace(V["r_root"], V["r_tip"], V["N_s"] + 1)     # same expression as build_strips
    V["com"] = fly.offset_b * MM                                         # CoM relative to thorax origin
    V["mass_mg"] = fly.M * 1e6
    # hover kinematics: design() defaults, then the amplitude solve design() itself performs
    dsig = inspect.signature(design).parameters
    V["f_hz"] = dsig["f_hz"].default; V["feather"] = dsig["feather"].default
    V["amp"] = float(sysid.hover_amplitude(fly, f_hz=V["f_hz"], feather=V["feather"]))
    # sensing
    V["max_range"] = inspect.signature(Antenna.__init__).parameters["max_range"].default * MM
    V["K_GE"], V["KAPPA_MAX"] = ground_effect.K_GE, ground_effect.KAPPA_MAX
    src = open("experiments/e48_mission.py").read()
    assert "ant.feel([0,30,-30,50,-50,90,-90])" in src, "feeler angle list changed in e48_mission.py"
    V["feelers"] = FEELER_ANGLES
    # equation numbers: order of \label{eq:*} in the draft
    import re
    labels = re.findall(r"\\label\{(eq:[^}]+)\}", open("report_draft.md").read())
    assert labels.index("eq:kappa") + 1 == EQ_KAPPA and labels.index("eq:tauwall") + 1 == EQ_TAUWALL
    return V


def report(V):
    L = []
    P = L.append
    P("VALUES READ FROM MODEL / SOURCE (mm unless stated)")
    for n, g in V["geoms"].items():
        P(f"  geom {n:9s} pos={np.round(g['pos'], 4).tolist()} size={np.round(g['size'], 4).tolist()}")
    for w in "RL":
        P(f"  stroke_{w} hinge pos = {np.round(V['hinge'][w], 4).tolist()}   (side: {SIDE[int(np.sign(V['hinge'][w][1]))]})")
    P(f"  R (hinge->tip, wing_params r_tip / Flyer.R) = {V['R']:.4f}")
    P(f"  2R                                          = {V['two_R']:.4f}")
    P(f"  hinge lateral offset |y|                    = {V['hinge_y']:.4f}")
    P(f"  geometric tip-to-tip = 2(|y_hinge| + R)     = {V['tip_to_tip']:.4f}")
    P(f"  c_max                                       = {V['c_max']:.4f}")
    P(f"  x_hat0 (pitch axis / c_max, from LE)        = {V['x_hat0']:.4f}")
    P(f"  r_root, r_tip, y_center, span_semi          = {V['r_root']:.4f}, {V['r_tip']:.4f}, {V['y_center']:.4f}, {V['span_semi']:.4f}")
    P(f"  N_s = {V['N_s']}   dr = {V['dr']:.4f}")
    P(f"  strip edges  = {np.round(V['edges'], 3).tolist()}")
    P(f"  strip centres= {np.round(V['strip_r'], 3).tolist()}")
    P(f"  strip chords = {np.round(V['strip_c'], 4).tolist()}")
    P(f"  CoM rel. thorax origin = {np.round(V['com'], 4).tolist()}   mass = {V['mass_mg']:.2f} mg")
    P(f"  hover: f = {V['f_hz']} Hz, feather amp = {V['feather']} deg, stroke amp Phi = {V['amp']:.4f} deg")
    P(f"  feeler angles (deg, +left) = {V['feelers']}   max_range = {V['max_range']:.0f}")
    P(f"  K_GE = {V['K_GE']}   KAPPA_MAX = {V['KAPPA_MAX']}")
    P(f"  eq:kappa -> ({EQ_KAPPA}), eq:tauwall -> ({EQ_TAUWALL})  [label order in report_draft.md]")
    txt = "\n".join(L); print(txt, flush=True)
    return txt


def save(fig, name, svg=False):
    for ext in ["png", "pdf"] + (["svg"] if svg else []):
        fig.savefig(f"{OUT}/{name}.{ext}", dpi=400, facecolor="white")
    plt.close(fig); print(f"saved {OUT}/{name}.[png,pdf{',svg' if svg else ''}]", flush=True)


def dim(ax, p0, p1, text, off=(0, 0), ha="center", va="center", fs=9, color=K, lw=0.9, txt_kw=None):
    """Dimension line with arrowheads at both ends; label at midpoint + off."""
    ax.annotate("", xy=p0, xytext=p1, arrowprops=dict(arrowstyle="<|-|>", lw=lw, color=color,
                                                      shrinkA=0, shrinkB=0, mutation_scale=8), zorder=8)
    mid = (np.asarray(p0) + np.asarray(p1)) / 2 + np.asarray(off)
    ax.text(*mid, text, ha=ha, va=va, fontsize=fs, color=color, path_effects=HALO, zorder=9, **(txt_kw or {}))


def leader(ax, xy, xytext, text, ha="center", va="center", fs=9, color=K):
    ax.annotate(text, xy=xy, xytext=xytext, ha=ha, va=va, fontsize=fs, color=color, zorder=9,
                path_effects=HALO,
                arrowprops=dict(arrowstyle="-", lw=0.8, color=color, shrinkA=2, shrinkB=0))
    ax.plot(*xy, "o", ms=2.6, color=color, zorder=9)


# --------------------------------------------------------------------------------------
# 2. PANEL A — isometric render + overlay
# --------------------------------------------------------------------------------------
W_PX, H_PX = 3200, 2400
AZ, EL = 45.0, -np.degrees(np.arctan(1 / np.sqrt(2)))     # true isometric elevation (35.264 deg)
EXTENT_V = 31.0                                            # vertical extent of the ortho view (mm)
LOOKAT = np.array([-2.0, 0.0, 0.6])                        # thorax-frame mm; view centre only


def cam_axes():
    a, e = np.radians(AZ), np.radians(EL)
    f = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    r = np.array([np.sin(a), -np.cos(a), 0.0])
    return f, r, np.cross(r, f)


def proj(p):
    """thorax-frame mm -> screen mm (u right, v up), orthographic."""
    _, r, u = cam_axes()
    q = np.atleast_2d(np.asarray(p, float)) - LOOKAT
    return np.squeeze(np.column_stack([q @ r, q @ u]))


def render_iso():
    spec = mujoco.MjSpec.from_file(XML)
    for t in spec.textures:                                   # visual-only: white background
        if t.type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
            t.rgb1 = [1, 1, 1]; t.rgb2 = [1, 1, 1]
    spec.delete(spec.geom("floor"))                           # visual-only: no checker floor in the picture
    spec.visual.global_.offwidth = W_PX; spec.visual.global_.offheight = H_PX
    spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
    spec.visual.headlight.diffuse = [0.55, 0.55, 0.55]
    m = spec.compile(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)   # qpos = 0: reference pose
    origin = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "thorax")]
    m.vis.global_.orthographic = 1
    m.vis.global_.fovy = EXTENT_V / MM                        # orthographic: fovy = vertical extent (m)
    m.vis.map.znear = 1e-4
    cam = mujoco.MjvCamera(); mujoco.mjv_defaultFreeCamera(m, cam)
    cam.lookat[:] = origin + LOOKAT / MM; cam.distance = 0.06; cam.azimuth = AZ; cam.elevation = EL
    r = mujoco.Renderer(m, H_PX, W_PX)
    r.update_scene(d, cam); rgb = r.render().copy()
    r.enable_segmentation_rendering(); r.update_scene(d, cam); seg = r.render()[:, :, 0].copy()
    r.close()
    # verify OUR projection against the renderer: centroid of each wing's pixels vs projected geom centre
    s = EXTENT_V / H_PX
    err = {}
    for w in "RL":
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"wing_{w}_g")
        ys, xs = np.nonzero(seg == g)
        cen = np.array([(xs.mean() + 0.5 - W_PX / 2) * s, (H_PX / 2 - ys.mean() - 0.5) * s])   # pixel centres
        c3 = (d.geom_xpos[g] - origin) * MM
        err[w] = float(np.linalg.norm(cen - proj(c3)) / s)
    print(f"  render/overlay registration error (px): {err}", flush=True)
    assert max(err.values()) < 3.0, "overlay projection does not match the MuJoCo camera"
    return rgb


def panel_a(V):
    rgb = render_iso()
    hw, hh = EXTENT_V * W_PX / H_PX / 2, EXTENT_V / 2
    XL, YL = (-15.4, 18.8), (-9.4, 12.0)                       # crop of the rendered view (screen mm)
    fig = plt.figure(figsize=(8, 8 * (YL[1] - YL[0]) / (XL[1] - XL[0]))); ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(rgb, extent=[-hw, hw, -hh, hh], zorder=0, interpolation="lanczos")
    ax.set_xlim(*XL); ax.set_ylim(*YL); ax.axis("off")

    zh = V["hinge"]["R"][2]; yh = V["hinge_y"]; R = V["R"]
    xc = V["geoms"]["wing_R_g"]["pos"][0]                 # chordwise centre of the wing ellipse
    ytip = yh + R

    def line3(a, b, **kw):
        a2, b2 = proj(a), proj(b); ax.plot([a2[0], b2[0]], [a2[1], b2[1]], **kw)

    def dim3(a, b, off, text, toff, **kw):
        a, b, off = map(np.asarray, (a, b, off))
        for p in (a, b):
            line3(p, p + off * 1.06, color=G, lw=0.7, zorder=7)
        pa, pb = proj(a + off), proj(b + off)
        ang = np.degrees(np.arctan2(*(pb - pa)[::-1])); ang = (ang + 90) % 180 - 90
        n = np.array([-np.sin(np.radians(ang)), np.cos(np.radians(ang))])
        dim(ax, pa, pb, text, off=toff * n, txt_kw=dict(rotation=ang, rotation_mode="anchor"), **kw)

    # ---- tip-to-tip span, R (hinge->tip), c_max ------------------------------------------
    dim3([xc, -ytip, zh], [xc, ytip, zh], [9.6, 0, 0],
         rf"tip-to-tip span $b = 2(y_h + R)$ = {V['tip_to_tip']:.1f} mm", 0.75)
    dim3([0, yh, zh], [0, ytip, zh], [5.6, 0, 0], rf"$R$ = {R:.1f} mm", 0.75)
    # chord at the max-chord station of the -y wing
    yc = -(yh + V["y_center"]); le = xc + V["c_max"] / 2; te = xc - V["c_max"] / 2
    a2, b2 = proj([le, yc, zh]), proj([te, yc, zh])
    ax.annotate("", xy=a2, xytext=b2, arrowprops=dict(arrowstyle="<|-|>", lw=1.0, color=K, shrinkA=0,
                                                      shrinkB=0, mutation_scale=7), zorder=8)
    leader(ax, (a2 + b2) / 2, (a2 + b2) / 2 + np.array([0.6, -4.9]),
           rf"$c_{{\max}}$ = {V['c_max']:.1f} mm", va="top")

    # ---- body frame at the CoM -----------------------------------------------------------
    c = V["com"]; o2 = proj(c); AX = 4.2
    for vec, name, toff, ha in (([1, 0, 0], r"$x_b$ (forward)", (1.0, 0.0), "left"),
                                ([0, 1, 0], r"$y_b$ (left)", (-0.2, -0.85), "center"),
                                ([0, 0, 1], r"$z_b$ (up)", (0.0, 0.6), "center")):
        t2 = proj(c + AX * np.asarray(vec, float))
        ax.annotate("", xy=t2, xytext=o2, zorder=10,
                    arrowprops=dict(arrowstyle="-|>", lw=1.7, color=K, shrinkA=0, shrinkB=0, mutation_scale=11,
                                    path_effects=[pe.withStroke(linewidth=3.4, foreground="white")]))
        ax.text(t2[0] + toff[0], t2[1] + toff[1], name, fontsize=9, ha=ha, va="center",
                path_effects=HALO, zorder=11)
    ax.plot(*o2, "o", ms=6, mfc="white", mec=K, mew=1.3, zorder=12)
    leader(ax, o2, o2 + np.array([0.15, -4.4]), "CoM", va="top")

    # ---- leader labels -------------------------------------------------------------------
    tail = V["geoms"]["tail_g"]
    leader(ax, proj([tail["pos"][0] - 1.5, 0, tail["size"][2]]), proj([tail["pos"][0] - 1.5, 0, 0]) + np.array([4.5, -5.0]),
           "body", va="top")
    sgn = {"R": int(np.sign(V["hinge"]["R"][1])), "L": int(np.sign(V["hinge"]["L"][1]))}
    for w, toff in (("R", (-5.0, 3.6)), ("L", (5.8, -2.4))):
        s = sgn[w]
        leader(ax, proj([xc - 0.6, s * (yh + 9.6), zh]), proj([xc - 0.6, s * (yh + 9.6), zh]) + np.array(toff),
               f"{SIDE[s]} wing", va="bottom" if toff[1] > 0 else "top")
    leader(ax, proj([0, -yh, zh]), proj([0, -yh, zh]) + np.array([4.3, -0.55]),
           "wing root\n(stroke / pitch hinge)", ha="left")
    ax.plot(*proj([0, yh, zh]), "o", ms=2.6, color=K, zorder=9)
    save(fig, "model_iso")


# --------------------------------------------------------------------------------------
# 3. PANEL B — one wing, top view, strips + kinematics inset
# --------------------------------------------------------------------------------------
def panel_b(V):
    R, cm, rc, a = V["R"], V["c_max"], V["y_center"], V["span_semi"]
    xc = V["geoms"]["wing_L_g"]["pos"][0]                    # chordwise centre of the ellipse (local x)
    c_at = lambda r: chord(np.asarray(r, float) / MM, cm / MM, rc / MM, a / MM) * MM   # src.aero.chord
    le = lambda r: xc + c_at(r) / 2; te = lambda r: xc - c_at(r) / 2
    side = SIDE[int(np.sign(V["hinge"]["L"][1]))]

    fig = plt.figure(figsize=(7.6, 5.75))
    ax = fig.add_axes([0.03, 0.375, 0.94, 0.615]); ax.set_aspect("equal"); ax.axis("off")
    ax.set_xlim(-2.8, 19.2); ax.set_ylim(-5.5, 5.4)

    ax.add_patch(mp.Ellipse((rc, xc), 2 * a, cm, fc="#ececec", ec=K, lw=1.3, zorder=2))
    for e in V["edges"][1:-1]:                                # interior strip boundaries
        ax.plot([e, e], [te(e), le(e)], color=G, lw=0.7, zorder=3)
    for e in (V["edges"][0], V["edges"][-1]):                 # root / tip stations (chord = 0 there)
        ax.plot([e, e], [xc - 0.35, xc + 0.35], color=G, lw=0.7, zorder=3)

    # representative strip, drawn as the rectangle the blade-element sum actually uses: c(r_i) x dr
    i = 13; ri, ci, dr = V["strip_r"][i], V["strip_c"][i], V["dr"]
    assert np.isclose(ci, c_at(ri))
    ax.add_patch(mp.Rectangle((ri - dr / 2, xc - ci / 2), dr, ci, fc="#9a9a9a", ec=K, lw=1.1, hatch="////", zorder=4))
    xo = R + 1.0                                               # chord dimension, outboard of the tip
    for xx in (xc + ci / 2, xc - ci / 2):
        ax.plot([ri + dr / 2, xo + 0.25], [xx, xx], color=G, lw=0.6, ls=(0, (3, 2)), zorder=5)
    dim(ax, (xo, xc - ci / 2), (xo, xc + ci / 2), "", lw=0.9)
    ax.text(xo + 0.35, xc, "representative strip $i$\n" rf"$c(r_i)$ = {ci:.2f} mm" "\n" rf"at $r_i$ = {ri:.1f} mm" "\n"
            r"$\mathrm{d}S_i = c(r_i)\,\Delta r$", ha="left", va="center", fontsize=9, linespacing=1.45)

    # pitch axis (local x = 0) and hinge
    ax.plot([-1.0, R + 0.7], [0, 0], color=K, lw=1.1, ls=(0, (7, 2, 1.5, 2)), zorder=6)
    ax.plot(0, 0, "o", ms=6.5, mfc="white", mec=K, mew=1.4, zorder=7)
    ax.annotate("stroke hinge\n(wing root)", xy=(0, 0), xytext=(-1.2, 1.2), ha="center", va="bottom", fontsize=9,
                arrowprops=dict(arrowstyle="-", lw=0.8, color=K, shrinkB=4))
    ax.annotate(rf"pitch axis at quarter-chord of $c_{{\max}}$ ($\hat{{x}}_0$ = {V['x_hat0']:.2f})",
                xy=(2.1, 0), xytext=(0.6, 4.05), ha="left", va="center", fontsize=9,
                arrowprops=dict(arrowstyle="-", lw=0.8, color=K, shrinkB=0, relpos=(0.08, 0)))
    ax.plot(2.1, 0, "o", ms=2.6, color=K, zorder=8)
    # planform label
    rr = 5.1
    ax.annotate("ellipsoid wing geom → elliptical planform\n"
                r"$c(r) = c_{\max}\sqrt{1-\left((r-r_c)/a\right)^2}$",
                xy=(rr, le(rr)), xytext=(3.6, 2.45), ha="left", va="center", fontsize=9,
                arrowprops=dict(arrowstyle="-", lw=0.8, color=K, shrinkB=0, relpos=(0.2, 0)))
    ax.text(13.6, 2.45, rf"$N_s$ = {V['N_s']} strips" "\n" rf"$r_c$ = {rc:.1f} mm, $a$ = {a:.1f} mm",
            ha="left", va="center", fontsize=9)
    ax.text(3.3, le(3.3) + 0.12, "LE", fontsize=8, color=G, ha="center", va="bottom", zorder=6)
    ax.text(1.9, te(1.9) - 0.15, "TE", fontsize=8, color=G, ha="center", va="top", zorder=6)
    # c_max at r = r_c
    dim(ax, (rc, xc - cm / 2), (rc, xc + cm / 2), "", lw=0.9)
    ax.text(rc - 0.25, xc - 0.75, rf"$c_{{\max}}$ = {cm:.1f} mm", ha="right", va="center", fontsize=9, zorder=9,
            bbox=dict(boxstyle="round,pad=0.15", fc="#ececec", ec="none"))
    # Delta r and R
    e0, e1 = V["edges"][4], V["edges"][5]; yt = -3.05
    for xx in (e0, e1):
        ax.plot([xx, xx], [te(xx), yt - 0.2], color=G, lw=0.6, ls=(0, (3, 2)), zorder=1)
    for x0, dx in ((e0, -0.9), (e1, 0.9)):
        ax.annotate("", xy=(x0, yt), xytext=(x0 + dx, yt),
                    arrowprops=dict(arrowstyle="-|>", lw=0.9, color=K, shrinkA=0, shrinkB=0, mutation_scale=7))
    ax.text(e1 + 1.05, yt, rf"$\Delta r$ = {V['dr']:.1f} mm", ha="left", va="center", fontsize=9)
    yb = -4.2
    for xx in (0, R):
        ax.plot([xx, xx], [-0.25 if xx == 0 else xc - 0.4, yb - 0.25], color=G, lw=0.6, ls=(0, (3, 2)), zorder=1)
    dim(ax, (0, yb), (R, yb), rf"$R$ = {R:.1f} mm", off=(0, -0.5))
    ax.text(-2.7, 5.3, f"{side} wing, top view", fontsize=9.5, fontweight="bold", ha="left", va="top")
    # orientation glyph
    ox, oy = 16.0, -4.6
    ax.annotate("", xy=(ox, oy + 1.5), xytext=(ox, oy), arrowprops=dict(arrowstyle="-|>", lw=1.0, color=K, shrinkA=0, shrinkB=0))
    ax.annotate("", xy=(ox + 1.5, oy), xytext=(ox, oy), arrowprops=dict(arrowstyle="-|>", lw=1.0, color=K, shrinkA=0, shrinkB=0))
    ax.text(ox, oy + 1.6, "forward", fontsize=8, ha="center", va="bottom")
    ax.text(ox + 1.65, oy, r"$r$ (outboard)", fontsize=8, ha="left", va="center")

    # ---- kinematics inset: stroke plane seen from above --------------------------------------
    ins = fig.add_axes([0.05, 0.01, 0.40, 0.355]); ins.set_aspect("equal"); ins.axis("off")
    ins.set_xlim(-0.75, 1.5); ins.set_ylim(-1.22, 1.22)
    Phi = V["amp"]; ph = np.radians(Phi); L = 1.0
    ins.add_patch(mp.Wedge((0, 0), L, -Phi, Phi, fc="#eeeeee", ec="none", zorder=0))
    ins.add_patch(mp.Arc((0, 0), 2 * L, 2 * L, theta1=-Phi, theta2=Phi, color=G, lw=0.7, ls=(0, (3, 2))))
    for s in (+1, -1):
        ins.plot([0, L * np.cos(s * ph)], [0, L * np.sin(s * ph)], color=G, lw=1.0, ls=(0, (4, 2)))
    ins.plot([0, 1.3], [0, 0], color=K, lw=0.8, ls=(0, (7, 2, 1.5, 2)))            # mid-stroke
    cur = np.radians(38.0)                                                         # illustrative instant
    ins.plot([0, L * np.cos(cur)], [0, L * np.sin(cur)], color=K, lw=2.6, solid_capstyle="round", zorder=4)
    ins.add_patch(mp.FancyArrowPatch((0.50, 0), (0.50 * np.cos(cur), 0.50 * np.sin(cur)), connectionstyle="arc3,rad=0.2",
                                     arrowstyle="-|>", mutation_scale=8, lw=0.9, color=K))
    ins.text(0.60, 0.20, r"$\phi(t)$", fontsize=9.5, ha="left", va="center")
    ra = 0.80
    for s in (+1, -1):
        ins.add_patch(mp.FancyArrowPatch((ra, 0), (ra * np.cos(s * ph), ra * np.sin(s * ph)),
                                         connectionstyle=f"arc3,rad={s * 0.33}", arrowstyle="-|>", mutation_scale=7,
                                         lw=0.8, color=G, zorder=2))
    ins.text(0.72, -0.44, r"$\Phi$", fontsize=9.5, ha="center", va="center", color=K)
    ins.text(0.50, 0.66, r"$\Phi$", fontsize=9.5, ha="center", va="center", color=K)
    ins.plot(0, 0, "o", ms=6.5, mfc="white", mec=K, mew=1.4, zorder=5)
    ins.text(-0.08, 0, "stroke\nhinge", fontsize=8, ha="right", va="center")
    ins.text(1.33, 0.05, "mid-stroke", fontsize=8, ha="left", va="bottom")
    ins.annotate("", xy=(-0.55, 1.05), xytext=(-0.55, 0.55), arrowprops=dict(arrowstyle="-|>", lw=1.0, color=K))
    ins.text(-0.55, 1.08, "forward", fontsize=8, ha="center", va="bottom")
    fig.text(0.53, 0.30, "Stroke kinematics (hover)", fontsize=9.5, fontweight="bold", ha="left", va="center")
    fig.text(0.53, 0.175,
             r"$\phi(t) = \Phi\cos(2\pi f t)$" "\n"
             rf"stroke amplitude  $\Phi$ = {Phi:.1f}°   (sweep $2\Phi$ = {2 * Phi:.1f}°)" "\n"
             rf"flapping frequency  $f$ = {V['f_hz']:g} Hz",
             fontsize=9.5, ha="left", va="center", linespacing=1.7)
    save(fig, "model_wing_detail")


# --------------------------------------------------------------------------------------
# 4. PANEL C — sensing geometry schematic (top-down, drawn; monochrome)
# --------------------------------------------------------------------------------------
def panel_c(V):
    # page coords: X = -y_body (so body-left is page-left), Y = x_body (nose up). View from above.
    T = lambda xb, yb: (-yb, xb)
    yh, R = V["hinge_y"], V["R"]
    D = 21.0             # schematic wall distance in drawing mm (illustrative; d is a variable)
    LRAY = 30.0          # drawn ray length (schematic; true range printed as text)
    com = V["com"]; O = np.array(T(com[0], com[1]))

    fig = plt.figure(figsize=(7.4, 7.4 * 68.5 / 77)); ax = fig.add_axes([0.01, 0.01, 0.98, 0.98])
    ax.set_aspect("equal"); ax.axis("off"); ax.set_xlim(-31, 46); ax.set_ylim(-28.5, 40)

    # wall on the +y (left) side
    XW = -D; y0w, y1w = -16, 39
    ax.add_patch(mp.Rectangle((XW - 2.6, y0w), 2.6, y1w - y0w, fc="white", ec=G, lw=0, hatch="/////", zorder=1))
    ax.plot([XW, XW], [y0w, y1w], color=K, lw=1.8, zorder=2)
    ax.text(XW - 3.4, y1w - 2, "wall", rotation=90, ha="right", va="top", fontsize=9.5)

    # vehicle, true proportions from the model (top view)
    g = V["geoms"]
    for n, fc in (("tail_g", "#8c8c8c"), ("thorax_g", "#4d4d4d"), ("head_g", "#4d4d4d")):
        p, s = g[n]["pos"], g[n]["size"]
        sx, sy = (s[0], s[0]) if n == "head_g" else (s[0], s[1])
        ax.add_patch(mp.Ellipse(T(p[0], p[1]), 2 * sy, 2 * sx, fc=fc, ec=K, lw=0.6, zorder=5))
    for w in "RL":
        p, s, h = g[f"wing_{w}_g"]["pos"], g[f"wing_{w}_g"]["size"], V["hinge"][w]
        ax.add_patch(mp.Ellipse(T(h[0] + p[0], h[1] + p[1]), 2 * s[1], 2 * s[0], ec=K, lw=0.9, zorder=4,
                                fc="#b3b3b3" if h[1] > 0 else "#f0f0f0"))        # near (wall-side) wing darker

    # feeler rays from the CoM (src/antenna.py casts from x_com), +angle = left
    for a in V["feelers"]:
        th = np.radians(a); dirp = np.array([-np.sin(th), np.cos(th)])      # page direction
        hit = (O[0] - XW) / np.sin(th) if a > 0 else np.inf                 # range to the wall plane
        if hit <= LRAY + 14:
            end = O + hit * dirp
            ax.plot([O[0], end[0]], [O[1], end[1]], color=K, lw=1.0, ls=(0, (5, 2.2)), zorder=6)
            ax.plot(*end, "o", ms=4.5, mfc=K, mec=K, zorder=7)
            lab = O + min(hit, LRAY) * 0.80 * dirp
        else:
            end = O + LRAY * dirp
            ax.annotate("", xy=end, xytext=O, zorder=6,
                        arrowprops=dict(arrowstyle="-|>", lw=1.0, color=K, ls=(0, (5, 2.2)), shrinkA=0, shrinkB=0,
                                        mutation_scale=9))
            lab = O + (LRAY + 2.6) * dirp
        txt = "0°" if a == 0 else f"{a:+d}°".replace("-", "−")
        if hit <= LRAY + 14:
            nrm = np.array([dirp[1], -dirp[0]]) * (1.9 if a != 90 else 0)
            ax.text(*(lab + nrm + (np.array([0, -1.6]) if a == 90 else 0)), txt, fontsize=9, ha="center", va="center",
                    path_effects=HALO, zorder=8)
        else:
            ax.text(*lab, txt, fontsize=9, ha="center", va="center", zorder=8)
    ax.plot(*O, "o", ms=5.5, mfc="white", mec=K, mew=1.2, zorder=9)

    # wall distance d (body centreline -> wall) and a per-strip distance d_i
    yd = -13.0
    ax.plot([O[0], O[0]], [-10.8, yd - 1.0], color=G, lw=0.6, ls=(0, (3, 2)))
    dim(ax, (XW, yd), (O[0], yd), r"$d$", off=(0, -1.4), fs=10.5)
    xs = -(yh + V["strip_r"][15]); ysi = g["wing_R_g"]["pos"][0]; ydi = 3.4
    ax.plot([xs, xs], [ysi, ydi + 0.5], color=G, lw=0.6, ls=(0, (3, 2)), zorder=7)
    ax.plot(xs, ysi, "s", ms=3.4, color=K, zorder=8)
    dim(ax, (XW, ydi), (xs, ydi), r"$d_i$", off=(0, 1.3), fs=9.5, lw=0.8)
    ax.text(xs + 0.7, ydi + 0.3, r"strip $i$", fontsize=8, ha="left", va="center", path_effects=HALO, zorder=8)

    xn, xf = -(yh + V["y_center"]), (yh + V["y_center"]); yl = g["wing_R_g"]["pos"][0] - g["wing_R_g"]["size"][0]
    ax.annotate("near wing\n" r"lift scaled by $\kappa(d_i) > 1$", xy=(xn - 1.5, yl + 0.3), xytext=(xn - 1.2, -5.2),
                ha="center", va="top", fontsize=9, arrowprops=dict(arrowstyle="-", lw=0.8, color=K, shrinkB=0))
    ax.annotate("far wing\n" r"$\kappa \approx 1$", xy=(xf, yl + 0.3), xytext=(xf + 1.5, -5.2), ha="center", va="top",
                fontsize=9, arrowprops=dict(arrowstyle="-", lw=0.8, color=K, shrinkB=0))
    ax.annotate("nose", xy=T(g["head_g"]["pos"][0] + 0.5, -0.4), xytext=(2.9, 11.0), fontsize=8.5, ha="center", va="center",
                arrowprops=dict(arrowstyle="-", lw=0.7, color=K, shrinkB=1))

    # legend-style notes
    ax.text(45.5, 39.5,
            f"{len(V['feelers'])} feeler rays (ray-casts) from the CoM\n"
            "angles from the nose, + = left\n"
            f"max range {V['max_range']:.0f} mm (not to scale)\n"
            "● ray–wall hit  → range returned",
            fontsize=8.5, ha="right", va="top", linespacing=1.5)

    # ---- rear-view inset: lift asymmetry -> roll torque -----------------------------------
    bx = (16.5, -28.0, 29.0, 19.5)
    ax.add_patch(mp.Rectangle(bx[:2], bx[2], bx[3], fc="white", ec=G, lw=0.7, zorder=12))
    cx, cy = bx[0] + 15.5, bx[1] + 5.4; sc = 0.72
    half = (yh + R) * sc
    ax.plot([cx - half, cx + half], [cy, cy], color=K, lw=2.0, solid_capstyle="round", zorder=14)
    ax.add_patch(mp.Circle((cx, cy), V["geoms"]["thorax_g"]["size"][1] * sc * 1.25, fc="#4d4d4d", ec=K, zorder=15))
    wxr = bx[0] + 2.3
    ax.add_patch(mp.Rectangle((wxr - 1.5, bx[1] + 1.2), 1.5, bx[3] - 6.6, fc="white", ec=G, lw=0, hatch="/////", zorder=13))
    ax.plot([wxr, wxr], [bx[1] + 1.2, bx[1] + bx[3] - 5.4], color=K, lw=1.6, zorder=14)
    arm = (yh + V["y_center"]) * sc
    for sx, ln in ((-1, 6.4), (+1, 3.9)):
        ax.annotate("", xy=(cx + sx * arm, cy + ln), xytext=(cx + sx * arm, cy + 0.2), zorder=15,
                    arrowprops=dict(arrowstyle="-|>", lw=1.6, color=K, shrinkA=0, shrinkB=0, mutation_scale=11))
    ax.text(cx - arm - 0.7, cy + 4.3, r"$\kappa L$", fontsize=9, ha="right", va="center", zorder=15)
    ax.text(cx + arm + 0.7, cy + 3.0, r"$L$", fontsize=9, ha="left", va="center", zorder=15)
    ax.add_patch(mp.FancyArrowPatch((cx - 2.9, cy + 2.0), (cx + 2.9, cy + 2.0), connectionstyle="arc3,rad=-0.75",
                                    arrowstyle="-|>", mutation_scale=10, lw=1.3, color=K, zorder=16))
    ax.text(cx, cy + 6.3, r"$\tau_x^{\mathrm{wall}}$", fontsize=10.5, ha="center", va="center", zorder=16)
    ax.text(bx[0] + 0.8, bx[1] + bx[3] - 0.9, "rear view (looking along $x_b$)", fontsize=8.5, fontweight="bold",
            ha="left", va="top", zorder=16)
    ax.text(bx[0] + bx[2] - 0.8, bx[1] + bx[3] - 3.6, "rolls away\nfrom wall", fontsize=8, ha="right", va="top",
            color=G, zorder=16)

    # equation references
    ax.text(45.5, 7.0,
            rf"$\kappa(d)$:  wing-wash lift enhancement, Eq. ({EQ_KAPPA})" "\n"
            rf"$\tau_x^{{\mathrm{{wall}}}}$:  resulting roll torque, Eq. ({EQ_TAUWALL})",
            fontsize=9, ha="right", va="top", linespacing=1.6)

    # body axes glyph
    gx, gy = 38.5, 14.5
    ax.annotate("", xy=(gx, gy + 5), xytext=(gx, gy), arrowprops=dict(arrowstyle="-|>", lw=1.1, color=K, shrinkA=0, shrinkB=0))
    ax.annotate("", xy=(gx - 5, gy), xytext=(gx, gy), arrowprops=dict(arrowstyle="-|>", lw=1.1, color=K, shrinkA=0, shrinkB=0))
    ax.add_patch(mp.Circle((gx, gy), 0.75, fc="white", ec=K, lw=1.0, zorder=5)); ax.add_patch(mp.Circle((gx, gy), 0.2, fc=K, zorder=6))
    ax.text(gx, gy + 5.4, r"$x_b$", fontsize=9, ha="center", va="bottom")
    ax.text(gx - 5.4, gy, r"$y_b$", fontsize=9, ha="right", va="center")
    ax.text(gx + 1.2, gy - 1.2, r"$z_b$", fontsize=9, ha="left", va="top")
    save(fig, "model_sensing_geometry", svg=True)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    V = read_values()
    open(f"{OUT}/values_read.txt", "w").write(report(V) + "\n")
    which = sys.argv[1:] or ["a", "b", "c"]
    if "a" in which: panel_a(V)
    if "b" in which: panel_b(V)
    if "c" in which: panel_c(V)
