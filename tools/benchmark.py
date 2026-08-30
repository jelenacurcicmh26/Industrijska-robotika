"""Run the three controllers headless and write docs/results.csv + docs/figures.

    python tools/benchmark.py

Every number and every curve in the README comes from here, so the figures can
be regenerated from whatever state the controllers are currently in.
"""

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import _sim                          # noqa: E402
import _svg                          # noqa: E402
import step3_computed_torque as s3   # noqa: E402

DOCS = "docs"
FIGS = os.path.join(DOCS, "figures")
JOINT = 1          # joint2 carries the most load, so it is the one plotted
MODES = [s3.MODE_PD, s3.MODE_CT_RIGID, s3.MODE_CT_FULL]
LABELS = {
    s3.MODE_PD:       "PD + gravity",
    s3.MODE_CT_RIGID: "Computed torque, rigid body",
    s3.MODE_CT_FULL:  "Computed torque, full model",
}


def err_deg(run):
    """Worst joint error at each instant, in degrees."""
    return np.degrees(np.max(np.abs(run["e"]), axis=1))


def segment_bounds(t_end):
    t, out = s3.T_MOVE + s3.DWELL_TIME, []
    while t < t_end - 1e-9:
        out.append(t)
        t += s3.T_MOVE + s3.DWELL_TIME
    return out


# ── Figure 1: tracking error ──────────────────────────────────────────────────

def fig_tracking_error(runs, mode):
    f = _svg.Figure(860, 430, mode)
    c = f.c
    f.title(64, 30, "Tracking error over one lap of the waypoint cycle",
            "Worst joint error at each instant. Log scale - the three controllers "
            "are two orders of magnitude apart.")

    t = runs[MODES[0]]["t"]
    p = _svg.Panel(f, 64, 92, 720, 250, (0, float(t[-1])), (1e-3, 20), yscale="log")

    for b in segment_bounds(float(t[-1])):
        p.vline(b)
    p.grid_y()

    for i, m in enumerate(MODES):
        r = runs[m]
        p.line(r["t"], np.maximum(err_deg(r), 1e-3), c["series"][i], decimate=2)

    p.axis_x(label="time [s]")
    p.ylabel("worst joint error [deg]")

    f.legend(64, 372, [(LABELS[m], c["series"][i]) for i, m in enumerate(MODES)],
             gap=250)
    for i, m in enumerate(MODES):
        f.text(80 + i * 250, 396, f"RMS {runs[m]['rms_deg']:.3f}°  ·  "
               f"peak {runs[m]['peak_deg']:.2f}°", c["muted"], 10)
    return f.save(os.path.join(FIGS, f"tracking-error-{mode}.svg"))


# ── Figure 2: where the torque actually comes from ────────────────────────────

def fig_torque_breakdown(runs, mode):
    f = _svg.Figure(880, 430, mode)
    c = f.c
    r = runs[s3.MODE_CT_FULL]
    names = [("inertial", "M(q)·a_ref"),
             ("bias",     "C(q,q̇)·q̇ + g(q)"),
             ("damping",  "viscous damping D·q̇"),
             ("coulomb",  "Coulomb friction")]

    rms = {k: np.sqrt(np.mean(r["parts"][k] ** 2, axis=0)) for k, _ in names}
    ymax = max(v.max() for v in rms.values()) * 1.18

    f.title(64, 30, "Friction is not a small correction",
            "RMS contribution of each term over one lap. Only the first two are in "
            "the rigid-body model; on joints 1, 3, 5 and 7 the other two are larger.")

    p = _svg.Panel(f, 64, 92, 750, 250, (-0.5, s3.N_ARM - 0.5), (0, ymax))
    p.grid_y()

    slot = 750 / s3.N_ARM
    bw = slot * 0.19
    for i, (key, _) in enumerate(names):
        for j in range(s3.N_ARM):
            v = rms[key][j]
            xv = j + (i - 1.5) * (bw / slot) * 1.06
            p.bar(xv, v, bw, c["series"][i])
            f.text(p.sx(xv), p.sy(v) - 6, f"{v:.1f}", c["muted"], 8, anchor="middle")

    p.axis_x(ticks=list(range(s3.N_ARM)), tick_fmt=lambda v: f"joint {int(v) + 1}")
    p.ylabel("RMS torque [N·m]")

    f.legend(64, 378, [(lab, c["series"][i]) for i, (_, lab) in enumerate(names)],
             gap=196)
    return f.save(os.path.join(FIGS, f"torque-breakdown-{mode}.svg"))


# ── Figure 3: why the reference has to be smooth ──────────────────────────────

def fig_step_vs_quintic(step_run, quintic_run, mode):
    f = _svg.Figure(860, 520, mode)
    c = f.c
    f.title(64, 30, "A step reference asks for infinite acceleration",
            "Same goal, same controller, same gains. Only the shape of the "
            "reference differs.")

    t = quintic_run["t"]
    xlim = (0, float(t[-1]))
    series = ((step_run, "step reference", c["series"][1]),
              (quintic_run, "quintic reference", c["series"][0]))

    qs = np.concatenate([r["q_d"][:, JOINT] for r, _, _ in series])
    p1 = _svg.Panel(f, 64, 88, 720, 130, xlim,
                    (float(qs.min()) - 0.15, float(qs.max()) + 0.15))
    for b in segment_bounds(float(t[-1])):
        p1.vline(b)
    p1.grid_y()
    for r, _, col in series:
        p1.line(r["t"], r["q_d"][:, JOINT], col, decimate=2)
    p1.axis_x(tick_fmt=lambda v: "")
    p1.ylabel("q_d joint 2 [rad]")

    p2 = _svg.Panel(f, 64, 258, 720, 175, xlim, (-70, 70))
    for b in segment_bounds(float(t[-1])):
        p2.vline(b)
    p2.grid_y(ticks=[-50, -25, 0, 25, 50])
    for lim in (50, -50):
        p2.hline(lim, c["muted"], dash="4 4", width=1.2)
    for r, _, col in series:
        p2.line(r["t"], np.clip(r["tau"][:, JOINT], -70, 70), col, decimate=2)
    p2.axis_x(label="time [s]")
    p2.ylabel("commanded τ joint 2 [N·m]")
    f.text(788, p2.sy(50), "±50 N·m limit", c["muted"], 9)

    f.legend(64, 466, [(lab, col) for _, lab, col in series], gap=210)
    f.text(64, 492, f"Step reference saturates the actuators on "
                    f"{step_run['sat_pct']:.0f}% of steps and peaks at "
                    f"{step_run['peak_deg']:.0f}° of error; the quintic never "
                    f"saturates.", c["text2"], 10)
    return f.save(os.path.join(FIGS, f"step-vs-quintic-{mode}.svg"))


# ── Figure 4: the quintic itself ──────────────────────────────────────────────

def fig_quintic_profile(mode):
    f = _svg.Figure(720, 520, mode)
    c = f.c
    seg = s3.QuinticSegment(s3.HOME_QPOS, np.zeros(7), np.zeros(7),
                            s3.WAYPOINTS[1], s3.T_MOVE)
    t = np.linspace(0, s3.T_MOVE, 400)
    q, qd, qdd = (np.array(x) for x in zip(*[seg.at(float(ti)) for ti in t]))

    f.title(64, 30, "One quintic segment",
            "Velocity and acceleration both start and end at zero, so no term "
            "of the control law jumps.")

    rows = ((q[:, JOINT],   "q_d [rad]",      "position"),
            (qd[:, JOINT],  "q̇_d [rad/s]",   "velocity"),
            (qdd[:, JOINT], "q̈_d [rad/s²]",  "acceleration"))

    for i, (y, ylab, tag) in enumerate(rows):
        lo, hi = float(y.min()), float(y.max())
        pad = max((hi - lo) * 0.18, 1e-3)
        p = _svg.Panel(f, 78, 88 + i * 140, 590, 105, (0, s3.T_MOVE),
                       (lo - pad, hi + pad))
        p.grid_y()
        p.hline(0, c["baseline"], width=1.0)
        p.line(t, y, c["series"][0])
        p.ylabel(ylab)
        if i == len(rows) - 1:
            p.axis_x(label="time within the segment [s]")
        else:
            p.axis_x(tick_fmt=lambda v: "")
        f.text(668, 100 + i * 140, tag, c["muted"], 10, anchor="end")

    return f.save(os.path.join(FIGS, f"quintic-profile-{mode}.svg"))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(FIGS, exist_ok=True)
    print(f"Running {_sim.LAP_SECONDS:.1f} s lap for each controller...")

    runs = {m: _sim.run(m) for m in MODES}
    step_run = _sim.run(s3.MODE_CT_FULL, trajectory="step")

    path = os.path.join(DOCS, "results.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["controller", "rms_error_deg", "peak_error_deg", "saturated_pct"])
        for m in MODES:
            r = runs[m]
            w.writerow([LABELS[m], f"{r['rms_deg']:.4f}",
                        f"{r['peak_deg']:.4f}", f"{r['sat_pct']:.1f}"])
        w.writerow(["Computed torque, full model, step reference",
                    f"{step_run['rms_deg']:.4f}", f"{step_run['peak_deg']:.4f}",
                    f"{step_run['sat_pct']:.1f}"])
    print(f"  wrote {path}")

    for m in MODES:
        r = runs[m]
        print(f"  {LABELS[m]:<30s} RMS {r['rms_deg']:7.4f}  "
              f"peak {r['peak_deg']:7.4f}  saturated {r['sat_pct']:.1f}%")

    for mode in ("light", "dark"):
        fig_tracking_error(runs, mode)
        fig_torque_breakdown(runs, mode)
        fig_step_vs_quintic(step_run, runs[s3.MODE_CT_FULL], mode)
        fig_quintic_profile(mode)


if __name__ == "__main__":
    main()
