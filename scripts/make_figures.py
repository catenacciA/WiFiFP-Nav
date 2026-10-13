#!/usr/bin/env python3
"""Render the README figures (light and dark variants) from wififp outputs.

    ./build/wififp build-map -t data/office/trajectory.tum -s data/office/scans.csv -o build/map
    ./build/wififp benchmark -t data/office/trajectory.tum -s data/office/scans.csv -o build/results/loo.csv
    python scripts/make_figures.py
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.patches import Ellipse  # noqa: E402
from PIL import Image  # noqa: E402


@dataclass(frozen=True)
class Theme:
    name: str
    surface: str
    text: str
    text2: str
    muted: str
    grid: str
    wall: str
    blue: str
    orange: str
    aqua: str


LIGHT = Theme("light", "#fcfcfb", "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#b4b3ad",
              "#2a78d6", "#eb6834", "#1baf7a")
DARK = Theme("dark", "#1a1a19", "#ffffff", "#c3c2b7", "#8a8984", "#2e2e2c", "#55544f",
             "#3987e5", "#d95926", "#199e70")

METHOD_LABEL = {"centroid": "Map centroid (baseline)", "wknn": "WKNN fingerprinting",
                "pathloss": "Path-loss factor graph", "hybrid": "Hybrid (WKNN init + factor graph)"}


def method_color(t: Theme, m: str) -> str:
    return {"hybrid": t.blue, "wknn": t.orange, "pathloss": t.aqua, "centroid": t.muted}[m]


def style(t: Theme) -> None:
    plt.rcParams.update({
        "figure.facecolor": t.surface, "axes.facecolor": t.surface, "savefig.facecolor": t.surface,
        "text.color": t.text, "axes.labelcolor": t.text2, "axes.edgecolor": t.grid,
        "xtick.color": t.text2, "ytick.color": t.text2, "grid.color": t.grid, "grid.linewidth": 0.8,
        "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold", "axes.titlelocation": "left",
        "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    })


# ---------------------------------------------------------------- data

def load_floorplan(data: Path):
    meta = {}
    for line in (data / "floorplan.yaml").read_text().splitlines():
        if ":" in line and not line.startswith("#"):
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = (float(v) for v in meta["origin"].strip("[]").split(",")[:2])
    occ = np.array(Image.open(data / meta["image"])) < 128
    h, w = occ.shape
    return occ, (ox, ox + w * res, oy, oy + h * res)


def load_trajectory(data: Path) -> np.ndarray:
    return np.loadtxt(data / "trajectory.tum")[:, 1:4]


# ---------------------------------------------------------------- helpers

def draw_floorplan(ax, t: Theme, occ, extent, traj):
    rgba = np.zeros((*occ.shape, 4))
    rgba[occ] = matplotlib.colors.to_rgba(t.wall)
    ax.imshow(rgba, extent=extent, origin="upper", interpolation="nearest", zorder=0)
    ax.plot(traj[:, 0], traj[:, 1], color=t.muted, lw=1.2, alpha=0.8, zorder=1, label="SLAM trajectory")
    lo, hi = traj[:, :2].min(0) - 3.0, traj[:, :2].max(0) + 3.0
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.axis("off")
    # 5 m scale bar, bottom-left
    x0, y0 = lo[0] + 0.8, lo[1] + 0.8
    ax.plot([x0, x0 + 5], [y0, y0], color=t.text2, lw=2, solid_capstyle="butt", zorder=5)
    ax.text(x0 + 2.5, y0 + 0.4, "5 m", ha="center", va="bottom", color=t.text2, fontsize=10)


def save(fig, out: Path, name: str, t: Theme) -> None:
    path = out / f"{name}-{t.name}.png"
    fig.savefig(path, dpi=200, bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)
    print("wrote", path)


# ---------------------------------------------------------------- figures

def fig_localization(t, occ, extent, traj, loo, out):
    d = loo[loo.method == "hybrid"].dropna(subset=["est_x"])
    fig, ax = plt.subplots(figsize=(7.5, 8.2))
    draw_floorplan(ax, t, occ, extent, traj)
    for _, r in d.iterrows():
        ax.plot([r.true_x, r.est_x], [r.true_y, r.est_y], color=t.text2, lw=1.0, zorder=2)
    ax.scatter(d.true_x, d.true_y, s=42, facecolor=t.surface, edgecolor=t.text, lw=1.4, zorder=3,
               label="Ground truth (LiDAR SLAM)")
    ax.scatter(d.est_x, d.est_y, s=42, color=t.blue, edgecolor=t.surface, lw=1.2, zorder=4,
               label="WiFi estimate")
    med = np.median(d.error_m)
    ax.set_title(f"Held-out scans localized from WiFi only\nmedian error {med:.1f} m, "
                 f"90% under {np.percentile(d.error_m, 90):.1f} m", fontsize=13)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=3, fontsize=10, handlelength=1.6,
              labelcolor=t.text2, columnspacing=1.2)
    save(fig, out, "localization", t)


def fig_access_points(t, occ, extent, traj, aps, out):
    fig, ax = plt.subplots(figsize=(7.5, 8.2))
    draw_floorplan(ax, t, occ, extent, traj)
    for band, color, label in ((aps.freq_mhz < 3000, t.blue, "2.4 GHz"), (aps.freq_mhz >= 3000, t.orange, "5 GHz")):
        sub = aps[band]
        for _, a in sub.iterrows():
            cov = np.array([[a.cov_xx, a.cov_xy], [a.cov_xy, a.cov_yy]])
            vals, vecs = np.linalg.eigh(cov)
            if vals.max() > 25:  # skip ellipses wider than 5 m sigma: they only add clutter
                continue
            angle = np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))
            ax.add_patch(Ellipse((a.x, a.y), 2 * np.sqrt(vals[1]), 2 * np.sqrt(vals[0]), angle=angle,
                                 facecolor=color, alpha=0.10, edgecolor=color, lw=0.8, zorder=2))
        ax.scatter(sub.x, sub.y, s=26, color=color, edgecolor=t.surface, lw=1.0, zorder=3,
                   label=f"{label} access point ({len(sub)})")
    ax.set_title(f"Radio map: {len(aps)} access points with 1σ position uncertainty", fontsize=13)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 0.0), ncol=3, fontsize=10, handlelength=1.6,
              labelcolor=t.text2, columnspacing=1.2)
    save(fig, out, "access-points", t)


def fig_error_cdf(t, loo, out):
    # Path-loss and hybrid are near-identical curves (see the README table), so the
    # chart shows the factor-graph method once, as hybrid, to keep every line visible.
    fig, ax = plt.subplots(figsize=(8, 4.6))
    series = [("centroid", "Map centroid (baseline)", "Centroid"), ("wknn", "WKNN fingerprinting", "WKNN"),
              ("hybrid", "Factor graph (hybrid)", "Factor graph")]
    for m, legend, short in series:
        e = np.sort(loo[loo.method == m].error_m.dropna().to_numpy())
        y = np.arange(1, len(e) + 1) / len(e)
        xs, ys = np.r_[0, np.repeat(e, 2)], np.r_[0, 0, np.repeat(y, 2)[:-1]]
        c = method_color(t, m)
        med = np.median(e)
        ax.plot(xs, ys, color=c, lw=2, ls="--" if m == "centroid" else "-", label=f"{legend}, median {med:.1f} m",
                zorder=3 if m == "hybrid" else 2)
        ax.scatter([med], [0.5], s=40, color=c, edgecolor=t.surface, lw=1.2, zorder=4)
        # end labels: the factor-graph curve ends next to WKNN, so its label goes on the left
        left = m == "hybrid"
        ax.text(e[-1] + (-0.15 if left else 0.15), 1.035, short, color=t.text2, fontsize=10, va="center",
                ha="right" if left else "left")
    ax.axhline(0.5, color=t.grid, lw=1, zorder=1)
    ax.set_xlim(0, 12.5)
    ax.set_ylim(0, 1.08)
    ax.set_xlabel("Horizontal error [m]")
    ax.set_ylabel("Held-out scans with error ≤ x")
    ax.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1.0))
    ax.grid(axis="x")
    ax.set_title("Localization error, leave-one-scan-out (dots mark the medians)")
    ax.legend(loc="lower right", fontsize=10, labelcolor=t.text2)
    save(fig, out, "error-cdf", t)


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", type=Path, default=root / "data/office")
    ap.add_argument("--map", type=Path, default=root / "build/map")
    ap.add_argument("--results", type=Path, default=root / "build/results/loo.csv")
    ap.add_argument("--out", type=Path, default=root / "docs/img")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    occ, extent = load_floorplan(args.data)
    traj = load_trajectory(args.data)
    aps = pd.read_csv(args.map / "access_points.csv", keep_default_na=False)
    loo = pd.read_csv(args.results)
    for t in (LIGHT, DARK):
        style(t)
        fig_localization(t, occ, extent, traj, loo, args.out)
        fig_access_points(t, occ, extent, traj, aps, args.out)
        fig_error_cdf(t, loo, args.out)


if __name__ == "__main__":
    main()
