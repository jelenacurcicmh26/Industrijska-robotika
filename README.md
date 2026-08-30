# xArm7 — Torque-Level Control in MuJoCo

Three joint-torque controllers of increasing sophistication on a 7-DOF UFACTORY xArm7,
built up one step at a time. **MuJoCo** is the plant, **Pinocchio** supplies the rigid-body
model the controllers reason with, and the two are cross-checked against each other at
every step.

<p align="center">
  <img src="docs/media/hero.gif" width="620" alt="xArm7 tracking a waypoint cycle under inverse dynamics control">
</p>

![Python](https://img.shields.io/badge/python-3.12-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.12-orange)
![Pinocchio](https://img.shields.io/badge/Pinocchio-4.1-green)

Course project — *Industrijska robotika*, doctoral studies, Faculty of Technical Sciences,
University of Novi Sad.

---

## Sažetak (srpski)

Projekat pokazuje upravljanje robotskom rukom xArm7 na nivou momenata u zglobovima, kroz
tri koraka rastuće složenosti. MuJoCo služi kao simulator (realni sistem), dok Pinocchio
računa dinamički model koji upravljački zakon koristi.

- **1. korak** — bez upravljanja, `τ = 0`. Ruka pada pod dejstvom gravitacije. Služi za
  proveru da se gravitacioni momenti izračunati Pinocchio-om poklapaju sa MuJoCo-vim
  `qfrc_bias`.
- **2. korak** — PD regulator sa kompenzacijom gravitacije. Radi, ali pojačanja moraju da
  se podešavaju za svaki zglob posebno, jer efektivna inercija zavisi od konfiguracije.
- **3. korak** — upravljanje inverznom dinamikom (*computed torque*). Zakon računa moment
  koji je zaista potreban, pa se greška svodi na linearnu jednačinu koja je ista za sve
  zglobove — jedno pojačanje umesto sedam parova.

**Glavni nalaz:** udžbenički oblik zakona inverzne dinamike otkazuje na ovom modelu.
Model `xarm7.xml` sadrži i viskozno trenje i Kulonovo trenje u zglobovima, a ti članovi
nisu deo krutog modela `M·q̈ + C·q̇ + g`. Bez njihove kompenzacije upravljanje inverznom
dinamikom prati **lošije** od običnog PD regulatora iz 2. koraka (2.98° naspram 0.99°
srednje kvadratne greške). Tek kada se dodaju, greška pada na 0.03°. Zaključak: zakon
zasnovan na modelu vredi tačno onoliko koliko vredi model.

---

## The three steps

| Step | Script | Control law |
|---|---|---|
| 1 | [`step1_gravity_only.py`](step1_gravity_only.py) | `τ = 0` |
| 2 | [`step2_PD_with_gravity_compensation.py`](step2_PD_with_gravity_compensation.py) | `τ = g(q) + Kp·e + Kd·ė` |
| 3 | [`step3_computed_torque.py`](step3_computed_torque.py) | `τ = M(q)·a_ref + C(q,q̇)·q̇ + g(q) + friction` |

## Quick start

```bash
conda env create -f environment.yml
conda activate robotika

python step1_gravity_only.py
python step2_PD_with_gravity_compensation.py
python step3_computed_torque.py
```

Pinocchio has no Windows wheel on PyPI and must come from conda-forge. The PyPI package
called `pinocchio` is an unrelated abandoned project — installing it with pip gives an
empty module with no `buildModelFromMJCF`.

Keys, once the viewer window has focus:

| Key | Action |
|---|---|
| `SPACE` | pause / unpause |
| `R` | reset to home pose |
| `A` | toggle auto-cycle through the waypoints |
| `1` `2` `3` | move to pose 1 / 2 / 3 |
| `C` | cycle the controller (step 3 only) |
| `E` | reset the error statistics (step 3 only) |
| `ESC` | quit |

---

## Step 1 — gravity only

<p align="center">
  <img src="docs/media/step1-gravity.gif" width="520" alt="The arm collapsing under gravity with zero control torque">
</p>

Zero torque on every joint, so the arm falls. The point is not the falling — it is that
each step compares the gravity torque Pinocchio computes from the MJCF against MuJoCo's
own `qfrc_bias`. They agree to ~1e-13 N·m, which is what licenses everything after this.

## Step 2 — PD with gravity compensation

$$\tau = g(q) + K_p (q_d - q) + K_d(\dot q_d - \dot q)$$

Gravity is cancelled by feedforward, and a spring-damper handles the rest. It works, but
the effective inertia a joint presents depends on the configuration — an outstretched arm
is far harder to swing than a folded one — so a single gain is too soft in one pose and
too twitchy in another. Hence seven hand-tuned gain pairs.

## Step 3 — inverse dynamics (computed torque)

$$\tau = M(q)\,a_{\text{ref}} + C(q,\dot q)\dot q + g(q) + D\dot q + f_c \tanh(\dot q / \varepsilon)$$

$$a_{\text{ref}} = \ddot q_d + K_d(\dot q_d - \dot q) + K_p(q_d - q)$$

Instead of pushing harder when the error grows, the controller computes the torque the arm
actually needs for the motion it wants, and adds a small correction. Substituting into the
robot dynamics leaves

$$\ddot e + K_d \dot e + K_p e = 0$$

which is the same second-order system for every joint, in every pose. The gains stop being
seven tuned pairs and become a pole placement: one natural frequency `OMEGA = 20 rad/s`,
critically damped, and `Kp = OMEGA²`, `Kd = 2·OMEGA` follow.

---

## Results

All three controllers, same trajectory, same gains, side by side:

<p align="center">
  <img src="docs/media/comparison.gif" width="900" alt="Three controllers running the same trajectory side by side">
</p>

| Controller | RMS error | Peak error |
|---|---:|---:|
| PD + gravity (step 2) | 0.992° | 3.29° |
| Computed torque, rigid body only | 2.979° | 11.66° |
| **Computed torque, full model** | **0.029°** | **0.20°** |

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/tracking-error-dark.svg">
    <img src="docs/figures/tracking-error-light.svg" width="860" alt="Tracking error over one lap, log scale">
  </picture>
</p>

Raw numbers live in [`docs/results.csv`](docs/results.csv).

## The part that did not go as expected

The middle row of that table is the interesting one. A *more* sophisticated,
model-based controller tracked **three times worse** than the simple PD law it was
supposed to improve on.

The textbook derivation assumes the robot is rigid bodies with mass and nothing else. But
`xarm7.xml` also gives every arm joint viscous damping (10/10/5/5/5/2/2 N·m·s/rad) and
Coulomb `frictionloss` of 1 N·m. Those forces are real in the simulation, but they are not
part of $M\ddot q + C\dot q + g$ — MuJoCo keeps them in `qfrc_passive`, not `qfrc_bias` —
so the rigid-body model Pinocchio builds from the same file cannot see them, and the
textbook law never cancels them.

They are not a rounding error:

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/torque-breakdown-dark.svg">
    <img src="docs/figures/torque-breakdown-light.svg" width="880" alt="RMS torque contribution of each term, per joint">
  </picture>
</p>

On joints 1, 3, 5 and 7 — the roll joints, whose axes stay roughly along the arm and so
carry almost no gravity load — the friction terms are *larger* than the entire rigid-body
contribution. Only joints 2 and 4 are genuinely dominated by gravity.
Feedback linearization is unforgiving in a way a PD law is not: its whole
premise is that it cancels the plant exactly, so anything left uncancelled goes straight
into the tracking error. A spring-damper never claimed to know the plant, and quietly
absorbs the same friction as extra damping.

> Worth stating plainly: the 0.029° figure comes from a simulation where the controller's
> model and the plant are generated from the same XML. It demonstrates that feedback
> linearization works; it is not a claim about hardware.

## Why the reference has to be smooth

Computed torque feeds the desired acceleration $\ddot q_d$ forward, so the reference cannot
be a step — step 2's jumps between poses command an infinite acceleration and simply
saturate the motors.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/step-vs-quintic-dark.svg">
    <img src="docs/figures/step-vs-quintic-light.svg" width="860" alt="Commanded torque under a step reference versus a quintic reference">
  </picture>
</p>

With a step reference the same controller saturates the actuators on 7% of steps and peaks
at 74° of error. Waypoints are therefore joined by quintic polynomials, which start and end
at rest with zero acceleration, and take general initial conditions so that retargeting
mid-move stays $C^2$ continuous:

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/quintic-profile-dark.svg">
    <img src="docs/figures/quintic-profile-light.svg" width="720" alt="Position, velocity and acceleration of one quintic segment">
  </picture>
</p>

---

## Repository layout

```
step1_gravity_only.py                    τ = 0, and the Pinocchio/MuJoCo gravity check
step2_PD_with_gravity_compensation.py    PD + gravity compensation
step3_computed_torque.py                 inverse dynamics, quintic trajectories, 3-way compare
controllers/impedance_control.py         Cartesian impedance (work in progress)
tools/
    _sim.py            headless driver wrapping the step-3 controller
    _svg.py            minimal SVG chart writer
    benchmark.py       runs the controllers -> docs/results.csv + docs/figures
    record_media.py    offscreen rendering  -> docs/media
docs/
    figures/           the plots, light and dark
    media/             GIFs and stills
    results.csv        the numbers behind the tables
models/xarm7/          the robot description (see Attribution)
```

## Reproducing everything

```bash
python tools/benchmark.py       # results.csv + all figures
python tools/record_media.py    # all GIFs and stills
```

Both are headless — no viewer window opens. Every number and curve in this README comes
out of those two scripts, so nothing here can drift away from what the controllers
actually do.

## Next steps

- **Operational-space control** — add the task-space inertia $\Lambda = (JM^{-1}J^\top)^{-1}$
  and a dynamically consistent nullspace projector, finishing
  [`controllers/impedance_control.py`](controllers/impedance_control.py).
- **Adaptive control (Slotine–Li)** — exploit linearity in the inertial parameters via
  Pinocchio's `computeJointTorqueRegressor`, so the robot identifies what this project
  currently reads out of an XML file. Given the friction result above, this is the obvious
  continuation.
- **Momentum-based external torque observer** — collision detection and hand-guiding with
  no force/torque sensor.
- **QP-based control** — solve for τ subject to the real actuator limits, joint limits, and
  control-barrier constraints.

## Attribution

The robot description in [`models/xarm7/`](models/xarm7/) is **not my work**. It comes from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) and is derived from
UFACTORY's publicly available [xArm7 URDF](https://github.com/xArm-Developer/xarm_ros).
Copyright © 2018 UFACTORY Inc.; see [`models/xarm7/LICENSE`](models/xarm7/LICENSE) for the
full terms, which apply to that directory. Everything outside `models/` is my own.

## References

- Siciliano, Sciavicco, Villani, Oriolo — *Robotics: Modelling, Planning and Control*, ch. 8
  (computed torque and inverse dynamics control).
- Khatib — *A Unified Approach for Motion and Force Control of Robot Manipulators: The
  Operational Space Formulation*, IEEE J. Robotics and Automation, 1987.
- Slotine, Li — *On the Adaptive Control of Robot Manipulators*, IJRR, 1987.
- [MuJoCo documentation](https://mujoco.readthedocs.io/) — computation pipeline, `qfrc_bias`
  vs `qfrc_passive`.
- [Pinocchio documentation](https://stack-of-tasks.github.io/pinocchio/) — `crba`,
  `nonLinearEffects`, `rnea`.
