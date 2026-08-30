"""Shared matplotlib style for the figures in docs/figures.

Every figure is rendered twice, light and dark, so the README can serve the
right one with <picture>. The dark palette is a separate set of steps chosen
for the dark surface, not an inversion of the light one.

Output is SVG. Partly because it stays sharp at any zoom in the README, and
partly because matplotlib's PNG writer segfaults in this conda env - libpng
collides with something in the mujoco/pinocchio stack. Pillow's PNG encoder is
unaffected, which is what tools/record_media.py uses for the GIFs.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Categorical slots are used in this fixed order and never cycled. The order is
# what keeps adjacent pairs distinguishable under colour-vision deficiency.
THEME = {
    "light": {
        "surface":  "#fcfcfb",
        "text":     "#0b0b0b",
        "text2":    "#52514e",
        "muted":    "#898781",
        "grid":     "#e1e0d9",
        "baseline": "#c3c2b7",
        "series":   ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"],
    },
    "dark": {
        "surface":  "#1a1a19",
        "text":     "#ffffff",
        "text2":    "#c3c2b7",
        "muted":    "#898781",
        "grid":     "#2c2c2a",
        "baseline": "#383835",
        "series":   ["#3987e5", "#d95926", "#199e70", "#c98500"],
    },
}


def apply(mode: str):
    """Set rcParams for one mode and return its colour dict."""
    t = THEME[mode]
    plt.rcParams.update({
        "figure.facecolor":  t["surface"],
        "axes.facecolor":    t["surface"],
        "savefig.facecolor": t["surface"],
        "text.color":        t["text"],
        "axes.labelcolor":   t["text2"],
        "axes.edgecolor":    t["baseline"],
        "xtick.color":       t["muted"],
        "ytick.color":       t["muted"],
        "xtick.labelcolor":  t["text2"],
        "ytick.labelcolor":  t["text2"],
        "grid.color":        t["grid"],
        "grid.linewidth":    0.8,
        "axes.grid":         True,
        "axes.axisbelow":    True,
        "axes.spines.top":   False,
        "axes.spines.right": False,
        "lines.linewidth":   2.0,
        "font.size":         10,
        "axes.titlesize":    12,
        "axes.titleweight":  "bold",
        "legend.frameon":    False,
        "figure.dpi":        140,
    })
    return t


def save(fig, stem: str, mode: str, outdir="docs/figures"):
    """Write <outdir>/<stem>-<mode>.svg and close the figure."""
    import os
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, f"{stem}-{mode}.svg")
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {path}")
    return path
