"""
Projekat industrijska robotika: 3. korak - inverzna dinamika (computed torque)
xArm7 MuJoCo Simulation
========================
Control law:  τ = M(q)·a_ref + C(q,q̇)·q̇ + g(q) + D·q̇ + f_c·tanh(q̇/V_EPS)
              a_ref = q̈_d + Kd·(q̇_d − q̇) + Kp·(q_d − q)

With an exact model this leaves ë + Kd·ė + Kp·e = 0, so all 7 joints share one
gain pair (Kp = OMEGA², Kd = 2·ZETA·OMEGA) instead of the per-joint gains that
korak 2 needed. M and C·q̇ + g come from Pinocchio, not from MuJoCo.

Two things the textbook law leaves out and that matter here:

  - q̈_d is fed forward, so the reference has to be C². Step 2 jumped between
    poses; here the waypoints are joined by quintic polynomials.

  - xarm7.xml also gives every arm joint viscous damping (10/10/5/5/5/2/2) and
    frictionloss (1 N·m). Neither is part of M·q̈ + C·q̇ + g, so neither gets
    cancelled, and without the D·q̇ + friction terms the tracking comes out
    worse than step 2. Armature is already inside Pinocchio's M.

Key C cycles the three controllers. RMS over one auto-cycle lap:
korak 2 ≈ 0.99°, rigid body only ≈ 2.98°, full model ≈ 0.03°.
(tools/benchmark.py regenerates those numbers into docs/results.csv.)

Controls (viewer window must have focus):
  SPACE     - pause / unpause
  R         - reset to home pose
  A         - toggle auto-cycle mode (cycles through WAYPOINTS)
  1 / 2 / 3 - move to pose 1 / 2 / 3 and switch to manual mode
  C         - cycle controller (PD -> CT rigid body -> CT full model)
  E         - reset the tracking-error statistics
  ESC       - quit
"""

import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin
import time

# ── Scene ─────────────────────────────────────────────────────────────────────
MODEL_PATH     = "models/xarm7/scene.xml"
PIN_MODEL_PATH = "models/xarm7/xarm7.xml"

# ── Robot config ───────────────────────────────────────────────────────────────
ARM_JOINT_NAMES   = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7"]
N_ARM             = len(ARM_JOINT_NAMES)
END_EFFECTOR_SITE = "link_tcp"

HOME_QPOS = np.array([0.0, -0.3, 0.0, 1.0, 0.0, 1.3, 0.0])

# ── Named poses (keys 1/2/3) ──────────────────────────────────────────────────
POSES = {
    49: ("home",   np.array([ 0.0, -0.3,  0.0, 1.0,  0.0, 1.3,  0.0])),  # key 1
    50: ("pose A", np.array([ 0.5,  0.5, -0.3, 1.2, -0.5, 0.8,  0.3])),  # key 2
    51: ("pose B", np.array([-0.8,  0.2,  0.5, 0.8,  0.3, 1.5, -0.4])),  # key 3
}

# ── Trajectory ────────────────────────────────────────────────────────────────
WAYPOINTS  = [pose for _, pose in POSES.values()]
T_MOVE     = 1.5   # seconds to travel between two waypoints
DWELL_TIME = 1.0   # seconds held at a waypoint before the next move starts

# ── Computed-torque gains ─────────────────────────────────────────────────────
# Error dynamics are ë + Kd·ė + Kp·e = 0, so the gains are just a pole
# placement and the same numbers work for all 7 joints.
OMEGA = 20.0   # natural frequency of the closed-loop error [rad/s]
ZETA  = 1.0    # damping ratio (1.0 = critically damped, no overshoot)

KP = OMEGA**2 * np.eye(N_ARM)
KD = 2.0 * ZETA * OMEGA * np.eye(N_ARM)

# ── Friction compensation ─────────────────────────────────────────────────────
# tanh(q̇/V_EPS) instead of sign(q̇), otherwise the term chatters at zero
# crossings. Too small and the discontinuity is back, too large and low-speed
# friction stays uncompensated; 0.01 was the best of 0.001 / 0.005 / 0.01 /
# 0.05 / 0.1 here.
V_EPS          = 0.01   # [rad/s] smoothing width
FRICTION_COMP  = 1.0    # fraction of the Coulomb torque to cancel
# 1.0 only works because frictionloss is read from the model being simulated.
# On a real robot f_c is never known that well and overcompensation gives a
# limit cycle around standstill, so 0.8-0.9 is the usual choice.

# ── korak-2 gains, kept for the side-by-side comparison (key C) ───────────────
KP_PD = np.diag([300, 400, 300, 300, 100, 100,  50])
KD_PD = np.diag([ 20,  30,  20,  20,  10,  10,   5])

# ── Controllers cycled by key C ───────────────────────────────────────────────
MODE_PD, MODE_CT_RIGID, MODE_CT_FULL = 0, 1, 2
MODE_NAMES = {
    MODE_PD:       "PD + gravity (korak 2)",
    MODE_CT_RIGID: "computed torque, rigid body only",
    MODE_CT_FULL:  "computed torque, full model",
}
MODE_TAGS = {MODE_PD: "PD  ", MODE_CT_RIGID: "CT-R", MODE_CT_FULL: "CT-F"}

# GLFW key codes
KEY_SPACE, KEY_R, KEY_ESC, KEY_A, KEY_C, KEY_E = 32, 82, 256, 65, 67, 69


# ── Helpers ───────────────────────────────────────────────────────────────────

def _int(arr):
    return int(np.asarray(arr).flat[0])


def build_robot_info(model: mujoco.MjModel):
    """Pre-compute joint/dof/qpos indices for the arm."""
    joint_ids = [model.joint(n).id for n in ARM_JOINT_NAMES]
    dof_ids   = [_int(model.joint(jid).dofadr)  for jid in joint_ids]
    qpos_ids  = [_int(model.joint(jid).qposadr) for jid in joint_ids]
    return joint_ids, np.array(dof_ids), np.array(qpos_ids)


def build_pinocchio(mjcf_path: str):
    """Load xArm7 from MJCF into Pinocchio."""
    pin_model = pin.buildModelFromMJCF(mjcf_path)
    pin_data  = pin_model.createData()
    return pin_model, pin_data


def arm_dynamics_pin(pin_model, pin_data, q_arm: np.ndarray, dq_arm: np.ndarray):
    """Return (M, n) for the arm: inertia matrix (7x7) and n = C(q,q̇)·q̇ + g(q).

    The model includes the gripper (nq = nv = 13) but joint1-7 are at indices
    0-6, so the arm block is the leading 7x7 / 7x1 slice. Gripper velocity and
    acceleration are left at zero, so the M[:7, 7:] coupling never shows up.
    """
    q  = np.zeros(pin_model.nq)
    dq = np.zeros(pin_model.nv)
    q[:N_ARM]  = q_arm
    dq[:N_ARM] = dq_arm

    M = pin.crba(pin_model, pin_data, q)[:N_ARM, :N_ARM].copy()
    n = pin.nonLinearEffects(pin_model, pin_data, q, dq)[:N_ARM].copy()
    return M, n


def gravity_torques_pin(pin_model, pin_data, q_arm: np.ndarray) -> np.ndarray:
    """Return arm gravity torques [N·m] computed by Pinocchio."""
    q = np.zeros(pin_model.nq)
    q[:N_ARM] = q_arm
    return pin.computeGeneralizedGravity(pin_model, pin_data, q)[:N_ARM].copy()


def passive_torques(damping: np.ndarray, friction: np.ndarray,
                    dq_arm: np.ndarray) -> np.ndarray:
    """Torque that cancels the joint damping and Coulomb friction.

    MuJoCo keeps these in qfrc_passive rather than qfrc_bias, which is why the
    Pinocchio model built from the same MJCF does not see them.
    """
    return damping * dq_arm + FRICTION_COMP * friction * np.tanh(dq_arm / V_EPS)


# ── Quintic trajectory ────────────────────────────────────────────────────────

class QuinticSegment:
    """5th-order polynomial from (q0, v0, a0) to (qf, vf, af) in T seconds.

    Quintic and not cubic: a cubic has a jump in acceleration at the segment
    ends, and since q̈_d is fed forward that jump lands straight on the torque.
    Free initial conditions so a new segment can start from the current
    reference state, which keeps retargeting mid-move C² as well.
    """

    def __init__(self, q0, v0, a0, qf, T, vf=None, af=None):
        T = self.T = float(T)
        q0, v0, a0, qf = (np.asarray(x, float) for x in (q0, v0, a0, qf))
        vf = np.zeros_like(qf) if vf is None else np.asarray(vf, float)
        af = np.zeros_like(qf) if af is None else np.asarray(af, float)

        # c0..c2 come straight from the initial conditions. A, B, C are what
        # that part still gets wrong at t = T, and c3..c5 cancel exactly that.
        c0, c1, c2 = q0, v0, a0 / 2.0
        A = qf - (q0 + v0 * T + 0.5 * a0 * T**2)   # position left over
        B = vf - (v0 + a0 * T)                     # velocity left over
        C = af - a0                                # acceleration left over

        self.c = [
            c0, c1, c2,
            ( 20 * A -  8 * B * T +     C * T**2) / (2 * T**3),
            (-30 * A + 14 * B * T - 2 * C * T**2) / (2 * T**4),
            ( 12 * A -  6 * B * T +     C * T**2) / (2 * T**5),
        ]
        self.qf = qf

    def at(self, t: float):
        """Return (q_d, q̇_d, q̈_d) at time t; holds qf at rest once t >= T."""
        if t >= self.T:
            z = np.zeros_like(self.qf)
            return self.qf.copy(), z, z.copy()

        t = max(t, 0.0)
        c = self.c
        q   = c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5
        qd  = c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
        qdd = 2*c[2] + 6*c[3]*t + 12*c[4]*t**2 + 20*c[5]*t**3
        return q, qd, qdd


class TrackingStats:
    """Running RMS / peak joint error, so the controllers can be compared."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.sq_sum = 0.0
        self.peak   = 0.0
        self.n      = 0

    def add(self, err: np.ndarray):
        self.sq_sum += float(err @ err)
        self.peak    = max(self.peak, float(np.max(np.abs(err))))
        self.n      += 1

    @property
    def rms_deg(self) -> float:
        if self.n == 0:
            return 0.0
        return np.degrees(np.sqrt(self.sq_sum / (self.n * N_ARM)))

    @property
    def peak_deg(self) -> float:
        return np.degrees(self.peak)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    joint_ids, dof_ids, qpos_ids = build_robot_info(model)
    ee_id = model.site(END_EFFECTOR_SITE).id
    dt    = model.opt.timestep

    # Torque limits (act1-7 drive joint1-7) and the passive joint parameters.
    # Read from the model, so editing the XML does not silently break them.
    tau_lim  = model.actuator_ctrlrange[:N_ARM, 1].copy()
    damping  = model.dof_damping[dof_ids].copy()
    friction = model.dof_frictionloss[dof_ids].copy()

    pin_model, pin_data = build_pinocchio(PIN_MODEL_PATH)
    print(f"Pinocchio model: nq={pin_model.nq}  nv={pin_model.nv}"
          f"  gravity={pin_model.gravity.linear}")

    data.qpos[qpos_ids] = HOME_QPOS
    mujoco.mj_forward(model, data)

    # ── Model self-check ──────────────────────────────────────────────────────
    # Korak 1 only checked gravity. Here M and the Coriolis term are used too,
    # so compare both against MuJoCo at a non-zero velocity before trusting them.
    dq_test = np.array([0.3, -0.2, 0.4, 0.1, -0.3, 0.2, 0.1])
    data.qvel[dof_ids] = dq_test
    mujoco.mj_forward(model, data)

    M_pin, n_pin = arm_dynamics_pin(pin_model, pin_data, data.qpos[qpos_ids], dq_test)
    M_mj_full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, M_mj_full)
    M_mj = M_mj_full[np.ix_(dof_ids, dof_ids)]

    print(f"Model check   max|M_pin − M_mj|     = {np.max(np.abs(M_pin - M_mj)):.2e} kg·m²")
    print(f"Model check   max|n_pin − qfrc_bias| = "
          f"{np.max(np.abs(n_pin - data.qfrc_bias[dof_ids])):.2e} N·m")
    print(f"Unmodelled passive terms at q̇={np.round(dq_test, 2)}:")
    print(f"    viscous damping  = {np.round(damping * dq_test, 2)} N·m")
    print(f"    Coulomb friction = {np.round(friction, 2)} N·m")

    data.qvel[dof_ids] = 0.0
    mujoco.mj_forward(model, data)

    print(f"Gains: OMEGA={OMEGA} rad/s  ZETA={ZETA}  ->  Kp={OMEGA**2:.0f}  "
          f"Kd={2*ZETA*OMEGA:.0f}  (same for all 7 joints)")

    # ── Mutable controller state ──────────────────────────────────────────────
    seg       = QuinticSegment(HOME_QPOS, np.zeros(N_ARM), np.zeros(N_ARM),
                               HOME_QPOS, T_MOVE)
    t_seg     = 0.0            # sim time at which the current segment started
    paused    = False
    auto_mode = False
    wp_index  = 0
    mode      = MODE_CT_FULL
    stats     = TrackingStats()
    saturated = False

    def start_segment(q_goal, t_now):
        """Begin a new quintic to q_goal, continuous with the current reference."""
        nonlocal seg, t_seg
        q_d, qd_d, qdd_d = seg.at(t_now - t_seg)
        seg   = QuinticSegment(q_d, qd_d, qdd_d, np.asarray(q_goal, float), T_MOVE)
        t_seg = t_now

    def key_callback(keycode):
        nonlocal paused, auto_mode, wp_index, mode, seg, t_seg
        if keycode == KEY_SPACE:
            paused = not paused
        elif keycode == KEY_R:
            data.qpos[qpos_ids] = HOME_QPOS
            data.qvel[dof_ids]  = 0.0
            mujoco.mj_forward(model, data)
            seg   = QuinticSegment(HOME_QPOS, np.zeros(N_ARM), np.zeros(N_ARM),
                                   HOME_QPOS, T_MOVE)
            t_seg = data.time
            stats.reset()
            print("\n  → Reset to home pose")
        elif keycode == KEY_A:
            auto_mode = not auto_mode
            # stavlja Auto Connect na false da ne bi prikazivao plavo oko joint-ova
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_AUTOCONNECT] = 0
            if auto_mode:
                wp_index = 0
                start_segment(WAYPOINTS[wp_index], data.time)
            print(f"\n  → Auto-cycle {'ON' if auto_mode else 'OFF'}")
        elif keycode == KEY_C:
            mode = (mode + 1) % len(MODE_NAMES)
            stats.reset()
            print(f"\n  → Controller: {MODE_NAMES[mode]}   (error stats reset)")
        elif keycode == KEY_E:
            stats.reset()
            print("\n  → Error statistics reset")
        elif keycode in POSES:
            name, pose = POSES[keycode]
            start_segment(pose, data.time)
            auto_mode = False
            print(f"\n  → Moving to '{name}'  (auto-cycle OFF)")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.azimuth   = 150
        viewer.cam.elevation = -20
        viewer.cam.distance  = 1.5
        viewer.cam.lookat[:] = [0.2, 0.0, 0.4]

        print(f"\nSimulation running: {MODE_NAMES[mode]}.")
        print("SPACE=pause  R=reset  A=auto-cycle  1/2/3=pose  C=controller  E=reset stats  ESC=quit\n")

        t_print = time.time()
        while viewer.is_running():
            if not paused:
                # ── auto-cycle: next waypoint once the move + dwell is over ──
                if auto_mode and (data.time - t_seg) >= T_MOVE + DWELL_TIME:
                    wp_index = (wp_index + 1) % len(WAYPOINTS)
                    start_segment(WAYPOINTS[wp_index], data.time)

                # ── reference at the current sim time ────────────────────────
                q_d, qd_d, qdd_d = seg.at(data.time - t_seg)

                # ── current state ────────────────────────────────────────────
                q_cur  = data.qpos[qpos_ids]
                dq_cur = data.qvel[dof_ids]

                pos_err = q_d  - q_cur
                vel_err = qd_d - dq_cur

                if mode == MODE_PD:
                    # ── korak 2, running on the same trajectory ─────────────
                    # q̇_d is given to it here even though korak 2 never had it
                    # (its reference was a step, so q̇_d was always 0). Keeps
                    # the comparison from being rigged in favour of korak 3.
                    g   = gravity_torques_pin(pin_model, pin_data, q_cur)
                    tau = g + KP_PD @ pos_err + KD_PD @ vel_err
                else:
                    #   a_ref = q̈_d + Kd·ė + Kp·e
                    #   τ     = M(q)·a_ref + C(q,q̇)·q̇ + g(q)
                    M, n  = arm_dynamics_pin(pin_model, pin_data, q_cur, dq_cur)
                    a_ref = qdd_d + KD @ vel_err + KP @ pos_err
                    tau   = M @ a_ref + n
                    if mode == MODE_CT_FULL:
                        tau += passive_torques(damping, friction, dq_cur)

                # ── actuator limits ─────────────────────────────────────────
                # MuJoCo clips to ctrlrange on its own; doing it here too is
                # what makes the SAT! flag possible.
                tau_clipped = np.clip(tau, -tau_lim, tau_lim)
                saturated   = bool(np.any(tau_clipped != tau))
                data.ctrl[:N_ARM] = tau_clipped

                mujoco.mj_step(model, data)
                stats.add(pos_err)

            now = time.time()
            if now - t_print > 0.5:
                q_cur = data.qpos[qpos_ids]
                x_cur = data.site_xpos[ee_id]
                q_d, _, _ = seg.at(data.time - t_seg)
                err_deg   = np.degrees(np.linalg.norm(q_d - q_cur))
                mode_str  = "AUTO" if auto_mode else "MAN "
                sat_str   = " SAT!" if saturated else "     "
                print(f"\r  [{MODE_TAGS[mode]}|{mode_str}]"
                      f"  EE=[{x_cur[0]:+.3f},{x_cur[1]:+.3f},{x_cur[2]:+.3f}]"
                      f"  |e|={err_deg:6.3f}°"
                      f"  RMS={stats.rms_deg:6.3f}°  peak={stats.peak_deg:6.3f}°"
                      f"{sat_str}",
                      end="", flush=True)
                t_print = now

            viewer.sync()
            time.sleep(dt)


if __name__ == "__main__":
    main()
