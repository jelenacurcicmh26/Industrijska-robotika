"""
Projekat industrijska robotika: 2. korak - PD + gravitaciona kompenzacija
xArm7 MuJoCo Simulation
========================
Control law:  τ = g(q) + Kp·(q_des − q) + Kd·(0 − q̇)
Gravity torques g(q) are computed by Pinocchio (not MuJoCo qfrc_bias).

Controls (viewer window must have focus):
  SPACE     - pause / unpause
  R         - reset to home pose
  A         - toggle auto-cycle mode (cycles through WAYPOINTS every DWELL_TIME s)
  1 / 2 / 3 - jump to pose 1 / 2 / 3 and switch to manual mode
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

# ── Auto-cycle waypoints ──────────────────────────────────────────────────────
WAYPOINTS  = [pose for _, pose in POSES.values()]
DWELL_TIME = 3.0   # seconds at each waypoint before moving to the next

# ── PD gains ──────────────────────────────────────────────────────────────────
# Joints closer to the base carry more load → higher gains.
KP = np.diag([300, 400, 300, 300, 100, 100,  50])
KD = np.diag([ 20,  30,  20,  20,  10,  10,   5])

# GLFW key codes
KEY_SPACE, KEY_R, KEY_ESC, KEY_A = 32, 82, 256, 65


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


def gravity_torques_pin(pin_model, pin_data, q_arm: np.ndarray) -> np.ndarray:
    """Return arm gravity torques [N·m] computed by Pinocchio."""
    q = np.zeros(pin_model.nq)
    q[:N_ARM] = q_arm
    return pin.computeGeneralizedGravity(pin_model, pin_data, q)[:N_ARM]


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    joint_ids, dof_ids, qpos_ids = build_robot_info(model)
    ee_id = model.site(END_EFFECTOR_SITE).id

    pin_model, pin_data = build_pinocchio(PIN_MODEL_PATH)
    print(f"Pinocchio model: nq={pin_model.nq}  nv={pin_model.nv}"
          f"  gravity={pin_model.gravity.linear}")

    data.qpos[qpos_ids] = HOME_QPOS
    mujoco.mj_forward(model, data)

    # ── Mutable controller state ───────────────────────────────────────────────
    Q_DES     = HOME_QPOS.copy()
    paused    = False
    auto_mode = False          # True → cycle waypoints automatically
    t_auto    = time.time()    # timestamp of last waypoint switch

    def key_callback(keycode):
        nonlocal paused, auto_mode, t_auto
        if keycode == KEY_SPACE:
            paused = not paused
        elif keycode == KEY_R:
            data.qpos[qpos_ids] = HOME_QPOS
            data.qvel[dof_ids]  = 0.0
            Q_DES[:] = HOME_QPOS
            mujoco.mj_forward(model, data)
            print("\n  → Reset to home pose")
        elif keycode == KEY_A:
            auto_mode = not auto_mode
            t_auto    = time.time()
            print(f"\n  → Auto-cycle {'ON' if auto_mode else 'OFF'}")
        elif keycode in POSES:
            name, pose = POSES[keycode]
            Q_DES[:]  = pose
            auto_mode = False
            print(f"\n  → Goal set to '{name}'  (auto-cycle OFF)")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.azimuth   = 150
        viewer.cam.elevation = -20
        viewer.cam.distance  = 1.5
        viewer.cam.lookat[:] = [0.2, 0.0, 0.4]

        print("\nSimulation running — PD + gravity compensation (Pinocchio).")
        print("SPACE=pause  R=reset  A=auto-cycle  1/2/3=pose  ESC=quit\n")

        t_print = time.time()
        while viewer.is_running():
            if not paused:
                # ── auto-cycle: advance waypoint every DWELL_TIME seconds ───
                if auto_mode:
                    elapsed  = time.time() - t_auto
                    wp_index = int(elapsed / DWELL_TIME) % len(WAYPOINTS)
                    Q_DES[:] = WAYPOINTS[wp_index]

                # ── current state ──────────────────────────────────────────
                q_cur = data.qpos[qpos_ids]
                q_vel = data.qvel[dof_ids]

                # ── gravity compensation via Pinocchio ─────────────────────
                g = gravity_torques_pin(pin_model, pin_data, q_cur)

                # ── PD law:  τ = g(q) + Kp·(q_des − q) + Kd·(0 − q̇) ────
                pos_err          = Q_DES - q_cur
                tau              = g + KP @ pos_err + KD @ (-q_vel)
                data.ctrl[:N_ARM] = tau

                mujoco.mj_step(model, data)

            now = time.time()
            if now - t_print > 0.5:
                q_cur   = data.qpos[qpos_ids]
                x_cur   = data.site_xpos[ee_id]
                pos_err = Q_DES - q_cur
                mode_str = "AUTO" if auto_mode else "MANUAL"
                print(f"\r  [{mode_str}]  EE=[{x_cur[0]:.3f},{x_cur[1]:.3f},{x_cur[2]:.3f}]"
                      f"  |e|={np.linalg.norm(pos_err)*180/np.pi:.2f}°"
                      f"  τ_g=[{' '.join(f'{t:+6.1f}' for t in gravity_torques_pin(pin_model, pin_data, q_cur))}]   ",
                      end="", flush=True)
                t_print = now

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
