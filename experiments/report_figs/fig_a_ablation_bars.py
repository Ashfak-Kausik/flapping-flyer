"""
fig_a_ablation_bars.py — FIGURE A: ablation spine, 4 variants x 3 headline metrics.

Source (read-only, no re-simulation): outputs/e51_baselines.csv, AGGREGATE block
(the block after the blank line, columns: variant,runs,completed,comp_rate,hits,
tot_openings,detect_rate,courses_found,n_with_breach,per_passage_rate,...).

Plots, per variant (FULL, NO_WINGWASH, NO_FEELERS, OPEN_LOOP):
  - completion rate      = completed / runs
  - per-passage detection = courses_found / n_with_breach
  - per-opening detection = hits / tot_openings

NO_FEELERS and OPEN_LOOP show 0% detection because use_feelers=False structurally
disables the breach logger in e48_mission.py (the `if use_feelers and (not open_loop)
and ...` guard at e48_mission.py:86) -- these are not observed misses, detection was
never attempted. Annotated on the figure.

Run: python experiments/report_figs/fig_a_ablation_bars.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SRC_CSV = "outputs/e51_baselines.csv"
OUT_PNG = "outputs/report/fig_a_ablation_bars.png"

VARIANT_ORDER = ["FULL", "NO_WINGWASH", "NO_FEELERS", "OPEN_LOOP"]
COLORS = {"FULL": "#1d6fb8", "NO_WINGWASH": "#f59e0b",
          "NO_FEELERS": "#7c3aed", "OPEN_LOOP": "#c0392b"}


def load_aggregate_block(path):
    """Read the second (aggregate) comma-block of e51_baselines.csv -- the block
    that starts with the header 'variant,runs,completed,comp_rate,...'."""
    with open(path) as fh:  # universal-newline mode: this CSV is CRLF (csv.writer default)
        text = fh.read()
    blocks = [b for b in text.split("\n\n") if b.strip()]
    assert len(blocks) == 2, f"expected 2 blocks in {path}, found {len(blocks)}"
    rows = list(csv.DictReader(blocks[1].strip().splitlines()))
    return {r["variant"]: r for r in rows}


def main():
    agg = load_aggregate_block(SRC_CSV)

    print(f"Source CSV: {SRC_CSV} (aggregate block)")
    print(f"{'variant':13s} {'completed/runs':>16s} {'comp_rate':>10s} "
          f"{'found/breach':>14s} {'passage_rate':>13s} {'hits/openings':>15s} {'opening_rate':>13s}")
    rows_plotted = []
    for v in VARIANT_ORDER:
        r = agg[v]
        completed, runs = int(r["completed"]), int(r["runs"])
        found, breach = int(r["courses_found"]), int(r["n_with_breach"])
        hits, openings = int(r["hits"]), int(r["tot_openings"])
        comp_rate = float(r["comp_rate"])
        passage_rate = float(r["per_passage_rate"])
        opening_rate = float(r["detect_rate"])
        print(f"{v:13s} {completed:>7d}/{runs:<7d} {comp_rate:>10.4f} "
              f"{found:>6d}/{breach:<6d}  {passage_rate:>13.4f} "
              f"{hits:>7d}/{openings:<7d} {opening_rate:>13.4f}")
        rows_plotted.append(dict(variant=v, completed=completed, runs=runs,
                                  comp_rate=comp_rate, found=found, breach=breach,
                                  passage_rate=passage_rate, hits=hits,
                                  openings=openings, opening_rate=opening_rate))

    metrics = ["comp_rate", "passage_rate", "opening_rate"]
    metric_labels = ["Completion\n(completed / runs)",
                      "Per-passage detection\n(courses found ≥1 / courses with breach)",
                      "Per-opening detection\n(openings hit / total openings)"]
    x = np.arange(len(metrics))
    width = 0.2

    fig, ax = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor("white")

    for i, v in enumerate(VARIANT_ORDER):
        r = next(rr for rr in rows_plotted if rr["variant"] == v)
        vals = [r["comp_rate"], r["passage_rate"], r["opening_rate"]]
        offs = x + (i - 1.5) * width
        bars = ax.bar(offs, [val * 100 for val in vals], width=width,
                      color=COLORS[v], label=v, edgecolor="white", linewidth=0.6)
        counts = [f"{r['completed']}/{r['runs']}", f"{r['found']}/{r['breach']}",
                  f"{r['hits']}/{r['openings']}"]
        for i_metric, (bar, val, cnt) in enumerate(zip(bars, vals, counts)):
            if val == 0.0 and i_metric in (1, 2) and v in ("NO_FEELERS", "OPEN_LOOP"):
                # single combined label for structural zeros, placed clear of the axis
                ax.annotate(f"{cnt}\n0%\n(structural)*",
                            (bar.get_x() + bar.get_width()/2, 0),
                            textcoords="offset points", xytext=(0, 6), ha="center",
                            fontsize=6.8, color="#555555", style="italic")
            else:
                ax.annotate(f"{cnt}\n{val*100:.0f}%", (bar.get_x() + bar.get_width()/2, bar.get_height()),
                            textcoords="offset points", xytext=(0, 3), ha="center", fontsize=7.5)

    ax.set_xticks(x)
    ax.set_xticklabels(metric_labels, fontsize=9)
    ax.set_ylabel("rate (%)")
    ax.set_ylim(0, 108)
    ax.set_title("e51 ablation: completion & detection across 4 variants\n"
                  "(40-course primary ensemble, level=1.0 noise, rate=1000 Hz)")
    ax.legend(fontsize=9, ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.16))
    ax.grid(axis="y", alpha=0.25)
    fig.text(0.01, 0.005,
             "* NO_FEELERS (use_feelers=False) and OPEN_LOOP (open_loop=True, which also forces "
             "use_feelers=False) structurally disable the breach\n"
             "  logger in e48_mission.py (the `if use_feelers and (not open_loop) and ...` guard, "
             "e48_mission.py:86) -- 0% is an untried metric, not an observed miss.",
             fontsize=6.8, color="#444444")
    fig.tight_layout(rect=[0, 0.06, 1, 1])
    fig.savefig(OUT_PNG, dpi=300)
    print(f"\nsaved -> {OUT_PNG}")


if __name__ == "__main__":
    main()
