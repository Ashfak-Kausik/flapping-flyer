"""
fig_h_force_decomposition.py — force_decomposition.png: translational / rotational /
added-mass / total vertical force over two wingbeat cycles.

Source (read-only, no re-simulation): outputs/e07_decomposition.csv
(t_ms,trans_uN,rot_uN,added_uN,total_uN) -- the exact trace e07_full_aero.py computed
at F_REF=40Hz, symmetric rotation (d=0.0), last 2 of 4 simulated cycles (transient-settled),
re-zeroed to t=0. This is the SAME data behind the existing outputs/e07_full_aero.png
panel (A); this script re-plots just that decomposition, full-width, as its own
figure with no second (frequency-sweep) panel.

Weight/trim-thrust reference line: outputs/e10_stability_derivatives.csv, second
block (trim_wrench_Fx..Tz_SI), Fz entry = 7.592941e-04 N = 759.2941 uN -- the trim
thrust e10 solved for, which balances body weight at hover. Not recomputed here.

Run: python experiments/report_figs/fig_h_force_decomposition.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SRC_CSV = "outputs/e07_decomposition.csv"
WEIGHT_CSV = "outputs/e10_stability_derivatives.csv"
OUT_PNG = "outputs/report/fig_h_force_decomposition.png"

COLORS = {"trans": "#1d6fb8", "rot": "#c0392b", "added": "#2e9e4f", "total": "#111111"}
LABELS = {"trans": "translational", "rot": "rotational (Kramer)", "added": "added mass", "total": "total"}


def load_decomposition(path):
    with open(path) as fh:  # universal-newline mode: this CSV is CRLF
        rows = list(csv.DictReader(fh))
    return rows


def load_weight_uN(path):
    """block 2 of e10_stability_derivatives.csv is a single line:
    'trim_wrench_Fx..Tz_SI,<Fx>,<Fy>,<Fz>,<Tx>,<Ty>,<Tz>' (label + 6 values, no
    separate header/data rows -- Fz is the 3rd numeric field)."""
    with open(path) as fh:
        text = fh.read()
    blocks = [b for b in text.split("\n\n") if b.strip()]
    line = blocks[1].strip().splitlines()[0]
    label, *values = next(csv.reader([line]))
    assert label == "trim_wrench_Fx..Tz_SI", f"unexpected label {label!r} in {path}"
    fz_n = float(values[2])  # Fx, Fy, Fz, Tx, Ty, Tz
    return fz_n * 1e6  # -> uN


def main():
    rows = load_decomposition(SRC_CSV)
    weight_uN = load_weight_uN(WEIGHT_CSV)

    print(f"Source CSV: {SRC_CSV} ({len(rows)} rows)")
    print(f"Weight/trim reference: {WEIGHT_CSV} block 2, trim_wrench Fz = {weight_uN:.4f} uN")
    print("\nt_ms, trans_uN, rot_uN, added_uN, total_uN")
    t = np.array([float(r["t_ms"]) for r in rows])
    trans = np.array([float(r["trans_uN"]) for r in rows])
    rot = np.array([float(r["rot_uN"]) for r in rows])
    added = np.array([float(r["added_uN"]) for r in rows])
    total = np.array([float(r["total_uN"]) for r in rows])
    for r in rows:
        print(f"  {float(r['t_ms']):6.2f}  {float(r['trans_uN']):9.4f}  {float(r['rot_uN']):9.4f}  "
              f"{float(r['added_uN']):9.4f}  {float(r['total_uN']):9.4f}")

    print(f"\ncycle-mean (over the {len(rows)} rows, 2 cycles @ 40 Hz):")
    for k, arr in (("trans", trans), ("rot", rot), ("added", added), ("total", total)):
        print(f"  {k:6s} mean = {arr.mean():+9.4f} uN  ({arr.mean()/weight_uN*100:+6.2f}% of weight)")

    fig, ax = plt.subplots(figsize=(11, 5.5))
    fig.patch.set_facecolor("white")

    for k in ("trans", "rot", "added", "total"):
        arr = {"trans": trans, "rot": rot, "added": added, "total": total}[k]
        ax.plot(t, arr, color=COLORS[k], lw=2.2 if k == "total" else 1.4,
               label=LABELS[k], zorder=3 if k == "total" else 2)

    ax.axhline(weight_uN, color="gray", ls="--", lw=1.2,
              label=f"weight / trim thrust ({weight_uN:.1f} µN)")
    ax.axhline(0, color="black", lw=0.5)

    # mark the two cycle boundaries (25 ms each at 40 Hz)
    for cyc_t in (t[0], t[0] + 25.0, t[-1]):
        ax.axvline(cyc_t, color="#999999", lw=0.6, ls=":")

    ax.set_xlabel("time (ms)")
    ax.set_ylabel("vertical force (µN)")
    ax.set_title("Quasi-steady force decomposition over two wingbeat cycles\n"
                 "(F=40 Hz, symmetric rotation, per-strip terms summed over both wings)")
    ax.legend(fontsize=9, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.14))
    ax.grid(alpha=0.25)
    fig.tight_layout(rect=[0, 0.05, 1, 1])
    fig.savefig(OUT_PNG, dpi=300)
    print(f"\nsaved -> {OUT_PNG}")


if __name__ == "__main__":
    main()
