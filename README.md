# xArm7 — Torque-Level Control in MuJoCo

Three joint-torque controllers on a 7-DOF UFACTORY xArm7, built one on top of the other.
MuJoCo runs the physics and stands in for the real robot. Pinocchio computes the
rigid-body model that the controllers use. Each step checks the two against each other
before relying on anything.

<p align="center">
  <img src="docs/media/hero.gif" width="620" alt="xArm7 tracking a waypoint cycle under inverse dynamics control">
</p>

![Python](https://img.shields.io/badge/python-3.12-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.12-orange)
![Pinocchio](https://img.shields.io/badge/Pinocchio-4.1-green)

Course project for *Industrijska robotika*, doctoral studies, Faculty of Technical
Sciences, University of Novi Sad.

**🇷🇸 [Ova stranica na srpskom](README.sr.md)**

---

## The three steps

| Step | Script | Control law |
|---|---|---|
| 1 | [`step1_gravity_only.py`](step1_gravity_only.py) | `τ = 0` |
| 2 | [`step2_PD_with_gravity_compensation.py`](step2_PD_with_gravity_compensation.py) | `τ = g(q) + Kp·e + Kd·ė` |
| 3 | [`step3_computed_torque.py`](step3_computed_torque.py) | `τ = M(q)·a_ref + C(q,q̇)·q̇ + g(q) + friction` |

## Running it

```bash
conda env create -f environment.yml
conda activate robotika

python step1_gravity_only.py
python step2_PD_with_gravity_compensation.py
python step3_computed_torque.py
```

Pinocchio has to come from conda-forge; there is no Windows wheel on PyPI. Watch out for
the PyPI package called `pinocchio`, which is a different, abandoned project. Installing
that one with pip gives you an empty module with no `buildModelFromMJCF`.

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

Zero torque on every joint, so the arm falls over. The falling is not really the point.
What this step is for is comparing the gravity torque Pinocchio computes from the MJCF
file against MuJoCo's own `qfrc_bias`. They match to about 1e-13 N·m. Steps 2 and 3 both
assume that Pinocchio's model describes the same robot MuJoCo is simulating, so it is
worth confirming once.

## Step 2 — PD with gravity compensation

$$\tau = g(q) + K_p (q_d - q) + K_d(\dot q_d - \dot q)$$

Feedforward cancels gravity and a spring-damper handles the rest. This works well enough,
but the gains are awkward to tune. How much inertia a joint sees depends on the
configuration: an outstretched arm is much harder to swing than a folded one. One gain
value ends up too soft in one pose and too twitchy in another, which is why there are
seven hand-tuned pairs in the file.

## Step 3 — inverse dynamics (computed torque)

$$\tau = M(q)\,a_{\text{ref}} + C(q,\dot q)\dot q + g(q) + D\dot q + f_c \tanh(\dot q / \varepsilon)$$

$$a_{\text{ref}} = \ddot q_d + K_d(\dot q_d - \dot q) + K_p(q_d - q)$$

Rather than pushing harder as the error grows, this computes the torque the arm actually
needs for the motion being asked of it, then adds a small correction on top. Substituting
it into the robot dynamics leaves

$$\ddot e + K_d \dot e + K_p e = 0$$

the same second-order system for every joint in every pose. So the gains are no longer
seven tuned pairs but a pole placement. Pick one natural frequency, `OMEGA = 20 rad/s`,
make it critically damped, and `Kp = OMEGA²` and `Kd = 2·OMEGA` follow from that.

---

## Results

All three controllers on the same trajectory with the same gains:

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

The raw numbers are in [`docs/results.csv`](docs/results.csv).

## The friction problem

The middle row of that table is the interesting one. The model-based controller, which was
supposed to be the improvement, tracked three times worse than the PD law from step 2.

The textbook derivation treats the robot as rigid bodies with mass and nothing else. But
`xarm7.xml` also gives every arm joint viscous damping (10/10/5/5/5/2/2 N·m·s/rad) and
Coulomb `frictionloss` of 1 N·m. Those forces act in the simulation, but they are not part
of $M\ddot q + C\dot q + g$. MuJoCo keeps them in `qfrc_passive` rather than `qfrc_bias`,
so the rigid-body model Pinocchio builds from the same file never sees them, and the
control law never cancels them.

They turn out to be large:

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/torque-breakdown-dark.svg">
    <img src="docs/figures/torque-breakdown-light.svg" width="880" alt="RMS torque contribution of each term, per joint">
  </picture>
</p>

On joints 1, 3, 5 and 7 the friction terms are bigger than the whole rigid-body
contribution. Those are the roll joints, whose axes point roughly along the arm, so they
carry almost no gravity load. Only joints 2 and 4 are really dominated by gravity.

Feedback linearization is much less forgiving about this than PD is. It works by
cancelling the dynamics exactly, so whatever it fails to cancel goes straight into the
tracking error. A spring-damper never assumed it knew the plant in the first place, and
just absorbs the same friction as extra damping.

One caveat on the 0.029°: the controller's model and the simulated plant are generated
from the same XML file here. That number shows the method works, but it is not a claim
about how this would behave on hardware.

## Why the reference has to be smooth

The control law feeds the desired acceleration $\ddot q_d$ forward, so the reference cannot
be a step. Step 2 jumped straight from one pose to the next, which asks for infinite
acceleration and just saturates the motors.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/figures/step-vs-quintic-dark.svg">
    <img src="docs/figures/step-vs-quintic-light.svg" width="860" alt="Commanded torque under a step reference versus a quintic reference">
  </picture>
</p>

With a step reference the same controller saturates the actuators on 7% of steps and peaks
at 74° of error. Waypoints are joined by quintic polynomials instead. They start and end at
rest with zero acceleration, and they take arbitrary initial conditions, so pressing `2`
in the middle of a move to pose 3 re-plans from the current reference state without a jump.

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

## Regenerating the figures

```bash
python tools/benchmark.py       # results.csv and all figures
python tools/record_media.py    # all GIFs and stills
```

Neither opens a window. Every number and plot in this file comes from those two scripts,
so nothing here can quietly drift out of sync with the controllers.

There is no plotting library in `environment.yml`, which is deliberate. matplotlib's
compiled extensions will not load next to this mujoco/pinocchio stack on Windows: any draw
call dies with delay-load error `0xc06d007f` in `matplotlib._path`, with no traceback. So
`tools/_svg.py` writes the SVGs directly.

## Next steps

- **Operational-space control.** Add the task-space inertia $\Lambda = (JM^{-1}J^\top)^{-1}$
  and a dynamically consistent nullspace projector, to finish
  [`controllers/impedance_control.py`](controllers/impedance_control.py).
- **Adaptive control (Slotine–Li).** The dynamics are linear in the inertial parameters, and
  Pinocchio has `computeJointTorqueRegressor`, so the robot could identify for itself what
  this project currently reads out of an XML file. After the friction result above, this is
  the obvious thing to try next.
- **Momentum-based external torque observer,** for collision detection and hand-guiding
  without a force/torque sensor.
- **QP-based control,** solving for τ subject to the real actuator limits, joint limits and
  obstacle-avoidance constraints.

## Attribution

The robot description in [`models/xarm7/`](models/xarm7/) is not my work. It comes from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), derived from
UFACTORY's public [xArm7 URDF](https://github.com/xArm-Developer/xarm_ros).
Copyright © 2018 UFACTORY Inc. The full terms are in
[`models/xarm7/LICENSE`](models/xarm7/LICENSE) and apply to that directory. Everything
outside `models/` is mine.

## References

- Siciliano, Sciavicco, Villani, Oriolo, *Robotics: Modelling, Planning and Control*, ch. 8.
- Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The
  Operational Space Formulation", IEEE J. Robotics and Automation, 1987.
- Slotine and Li, "On the Adaptive Control of Robot Manipulators", IJRR, 1987.
- [MuJoCo documentation](https://mujoco.readthedocs.io/), on the computation pipeline and
  the difference between `qfrc_bias` and `qfrc_passive`.
- [Pinocchio documentation](https://stack-of-tasks.github.io/pinocchio/), for `crba`,
  `nonLinearEffects` and `rnea`.
