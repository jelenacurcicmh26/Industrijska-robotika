"""
Projekat industrijska robotika: 1. korak - samo gravitacija
xArm7 MuJoCo Simulation
========================
Currently running in gravity-only mode: tau = 0 on all joints.

Controls (viewer window must have focus):
  SPACE - pause / unpause
  R     - reset to home pose
  ESC   - quit
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

# ── Scene ─────────────────────────────────────────────────────────────────────
MODEL_PATH = "models/xarm7/scene.xml"

# ── Robot config ───────────────────────────────────────────────────────────────
ARM_JOINT_NAMES   = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7"]
N_ARM             = len(ARM_JOINT_NAMES)
END_EFFECTOR_SITE = "link_tcp"

HOME_QPOS = np.array([0.0, -0.3, 0.0, 1.0, 0.0, 1.3, 0.0])

# GLFW key codes
KEY_SPACE, KEY_R, KEY_ESC = 32, 82, 256


# ── Helpers ───────────────────────────────────────────────────────────────────

def _int(arr):
    return int(np.asarray(arr).flat[0])


def build_robot_info(model: mujoco.MjModel):
    """Pre-compute joint/dof/qpos indices for the arm."""
    joint_ids = [model.joint(n).id for n in ARM_JOINT_NAMES]
    dof_ids   = [_int(model.joint(jid).dofadr)  for jid in joint_ids]
    qpos_ids  = [_int(model.joint(jid).qposadr) for jid in joint_ids]
    return joint_ids, np.array(dof_ids), np.array(qpos_ids)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    joint_ids, dof_ids, qpos_ids = build_robot_info(model)
    ee_id = model.site(END_EFFECTOR_SITE).id

    data.qpos[qpos_ids] = HOME_QPOS
    mujoco.mj_forward(model, data)

    paused = False

    def key_callback(keycode):
        nonlocal paused
        if keycode == KEY_SPACE:
            paused = not paused
        elif keycode == KEY_R:
            data.qpos[qpos_ids] = HOME_QPOS
            data.qvel[dof_ids]  = 0.0
            mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.azimuth   = 150
        viewer.cam.elevation = -20
        viewer.cam.distance  = 1.5
        viewer.cam.lookat[:] = [0.2, 0.0, 0.4]

        print("Simulation running (gravity only — no control torques).")
        print("SPACE=pause  R=reset  ESC=quit\n")

        t_print = time.time()
        while viewer.is_running():
            if not paused:
                tau = np.zeros(N_ARM)
                data.ctrl[:N_ARM] = tau

                mujoco.mj_step(model, data)

            now = time.time()
            if now - t_print > 0.5:
                x_cur = data.site_xpos[ee_id]
                print(f"\r  EE=[{x_cur[0]:.3f},{x_cur[1]:.3f},{x_cur[2]:.3f}]   ",
                      end="", flush=True)
                t_print = now

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
