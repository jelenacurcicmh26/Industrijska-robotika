"""A very small SVG chart writer.

The figures are emitted directly as SVG rather than through matplotlib, for two
reasons. Matplotlib's compiled extensions fail to load in this conda env
(delay-load error 0xc06d007f inside matplotlib._path, so every draw call kills
the interpreter), and keeping it out means the whole repo runs on the four
packages in environment.yml. SVG also stays sharp at any zoom in the README.

Only what the four figures in benchmark.py need is implemented here.
"""

import math

FONT = "'Segoe UI', system-ui, -apple-system, Helvetica, Arial, sans-serif"

# Categorical slots, used in this fixed order and never cycled. The order is
# what keeps adjacent pairs apart under colour-vision deficiency.
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


def esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def nice_ticks(lo, hi, target=6):
    """Round tick positions covering [lo, hi]."""
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / max(target, 2)
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            stepv = m * mag
            break
    start = math.floor(lo / stepv) * stepv
    out, v = [], start
    while v <= hi + stepv * 1e-9:
        if v >= lo - stepv * 1e-9:
            out.append(round(v, 12))
        v += stepv
    return out


def fmt(v):
    if v == 0:
        return "0"
    a = abs(v)
    if a >= 1000 or a < 0.001:
        return f"{v:g}"
    if a < 0.01:
        return f"{v:.3f}".rstrip("0").rstrip(".")
    if a < 1:
        return f"{v:.2f}".rstrip("0").rstrip(".")
    if a < 10:
        return f"{v:.1f}".rstrip("0").rstrip(".")
    return f"{v:.0f}"


class Figure:
    """A canvas holding one or more stacked panels."""

    def __init__(self, width, height, mode):
        self.w, self.h = width, height
        self.mode = mode
        self.c = THEME[mode]
        self.parts = []

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, s, color=None, size=11, anchor="start",
             weight="normal", baseline="middle"):
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-family="{FONT}" '
                 f'font-size="{size}" fill="{color or self.c["text"]}" '
                 f'text-anchor="{anchor}" font-weight="{weight}" '
                 f'dominant-baseline="{baseline}">{esc(s)}</text>')

    def title(self, x, y, s, sub=None):
        self.text(x, y, s, self.c["text"], 14, weight="600")
        if sub:
            self.text(x, y + 18, sub, self.c["text2"], 11)

    def legend(self, x, y, entries, gap=176):
        """entries: list of (label, color). A swatch carries identity, the
        label stays in ink."""
        for i, (label, color) in enumerate(entries):
            cx = x + i * gap
            self.add(f'<rect x="{cx:.1f}" y="{y - 5:.1f}" width="10" height="10" '
                     f'rx="2.5" fill="{color}"/>')
            self.text(cx + 16, y, label, self.c["text2"], 11)

    def save(self, path):
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
                f'height="{self.h}" viewBox="0 0 {self.w} {self.h}">'
                f'<rect width="{self.w}" height="{self.h}" fill="{self.c["surface"]}"/>')
        with open(path, "w", encoding="utf-8") as f:
            f.write(head + "".join(self.parts) + "</svg>")
        print(f"  wrote {path}")


class Panel:
    """One plot rectangle with its own scales."""

    def __init__(self, fig, x, y, w, h, xlim, ylim, yscale="linear"):
        self.f, self.c = fig, fig.c
        self.x, self.y, self.w, self.h = x, y, w, h
        self.x0, self.x1 = xlim
        self.y0, self.y1 = ylim
        self.yscale = yscale

    def sx(self, v):
        return self.x + (v - self.x0) / (self.x1 - self.x0) * self.w

    def sy(self, v):
        if self.yscale == "log":
            v = max(v, self.y0)
            t = (math.log10(v) - math.log10(self.y0)) / \
                (math.log10(self.y1) - math.log10(self.y0))
        else:
            t = (v - self.y0) / (self.y1 - self.y0)
        return self.y + self.h - t * self.h

    # ── decoration ────────────────────────────────────────────────────────────

    def grid_y(self, ticks=None, label=True):
        ticks = ticks if ticks is not None else self._yticks()
        for v in ticks:
            yy = self.sy(v)
            if not (self.y - 0.5 <= yy <= self.y + self.h + 0.5):
                continue
            self.f.add(f'<line x1="{self.x:.1f}" y1="{yy:.1f}" '
                       f'x2="{self.x + self.w:.1f}" y2="{yy:.1f}" '
                       f'stroke="{self.c["grid"]}" stroke-width="1"/>')
            if label:
                self.f.text(self.x - 8, yy, fmt(v), self.c["text2"], 10, anchor="end")

    def _yticks(self):
        if self.yscale == "log":
            out, k = [], math.floor(math.log10(self.y0))
            while 10 ** k <= self.y1 * 1.0001:
                if 10 ** k >= self.y0 * 0.9999:
                    out.append(10 ** k)
                k += 1
            return out
        return nice_ticks(self.y0, self.y1)

    def axis_x(self, ticks=None, label=None, tick_fmt=fmt):
        ticks = ticks if ticks is not None else nice_ticks(self.x0, self.x1)
        yb = self.y + self.h
        self.f.add(f'<line x1="{self.x:.1f}" y1="{yb:.1f}" '
                   f'x2="{self.x + self.w:.1f}" y2="{yb:.1f}" '
                   f'stroke="{self.c["baseline"]}" stroke-width="1"/>')
        for v in ticks:
            xx = self.sx(v)
            if not (self.x - 0.5 <= xx <= self.x + self.w + 0.5):
                continue
            self.f.text(xx, yb + 14, tick_fmt(v), self.c["text2"], 10, anchor="middle")
        if label:
            self.f.text(self.x + self.w / 2, yb + 34, label, self.c["text2"], 11,
                        anchor="middle")

    def ylabel(self, s):
        cx, cy = self.x - 46, self.y + self.h / 2
        self.f.add(f'<text x="{cx:.1f}" y="{cy:.1f}" font-family="{FONT}" '
                   f'font-size="11" fill="{self.c["text2"]}" text-anchor="middle" '
                   f'dominant-baseline="middle" '
                   f'transform="rotate(-90 {cx:.1f} {cy:.1f})">{esc(s)}</text>')

    def vline(self, v, color=None, dash=None, width=1.0):
        xx = self.sx(v)
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.f.add(f'<line x1="{xx:.1f}" y1="{self.y:.1f}" x2="{xx:.1f}" '
                   f'y2="{self.y + self.h:.1f}" stroke="{color or self.c["grid"]}" '
                   f'stroke-width="{width}"{d}/>')

    def hline(self, v, color=None, dash=None, width=1.0):
        yy = self.sy(v)
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.f.add(f'<line x1="{self.x:.1f}" y1="{yy:.1f}" '
                   f'x2="{self.x + self.w:.1f}" y2="{yy:.1f}" '
                   f'stroke="{color or self.c["muted"]}" stroke-width="{width}"{d}/>')

    # ── marks ─────────────────────────────────────────────────────────────────

    def line(self, xs, ys, color, width=2.0, decimate=1):
        pts = []
        for i in range(0, len(xs), decimate):
            px, py = self.sx(xs[i]), self.sy(ys[i])
            if math.isfinite(px) and math.isfinite(py):
                pts.append(f"{px:.1f},{py:.1f}")
        self.f.add(f'<polyline fill="none" stroke="{color}" stroke-width="{width}" '
                   f'stroke-linejoin="round" stroke-linecap="round" '
                   f'points="{" ".join(pts)}"/>')

    def bar(self, xv, yv, wpx, color, base=0.0):
        xx, y1, y0 = self.sx(xv), self.sy(yv), self.sy(base)
        top, hgt = min(y0, y1), abs(y0 - y1)
        self.f.add(f'<rect x="{xx - wpx / 2:.1f}" y="{top:.1f}" width="{wpx:.1f}" '
                   f'height="{hgt:.1f}" rx="2.5" fill="{color}" '
                   f'stroke="{self.c["surface"]}" stroke-width="1"/>')

    def clip_rect(self):
        """A faint frame so an empty panel still reads as a panel."""
        self.f.add(f'<rect x="{self.x:.1f}" y="{self.y:.1f}" width="{self.w:.1f}" '
                   f'height="{self.h:.1f}" fill="none" '
                   f'stroke="{self.c["grid"]}" stroke-width="1"/>')
