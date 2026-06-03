"""
Projekat industrijska robotika: 1. korak - samo gravitacija
xArm7 MuJoCo Simulation
========================
Gravity-only mode: tau = 0 on all joints (robot falls).
Pinocchio 4 is used to compute gravity torques from MJCF and compare
them against MuJoCo's qfrc_bias reference each step.

Controls (viewer window must have focus):
  SPACE - pause / unpause
  R     - reset to home pose
  ESC   - quit
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


def build_pinocchio(mjcf_path: str):
    """Load xArm7 from MJCF into Pinocchio (gravity read from XML)."""
    pin_model = pin.buildModelFromMJCF(mjcf_path)
    pin_data  = pin_model.createData()
    return pin_model, pin_data


def gravity_torques_pin(pin_model, pin_data, q_arm: np.ndarray) -> np.ndarray:
    """Return arm gravity torques [N·m] computed by Pinocchio."""
    q = np.zeros(pin_model.nq)
    q[:N_ARM] = q_arm           # joint1-7 are at idx_q 0-6 in the MJCF model
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

        print("\nSimulation running (gravity only — no control torques).")
        print("Columns: τ_pin = Pinocchio gravity  |  τ_muj = MuJoCo qfrc_bias  |  err = max |Δ|")
        print("SPACE=pause  R=reset  ESC=quit\n")

        t_print = time.time()
        while viewer.is_running():
            if not paused:
                data.ctrl[:N_ARM] = np.zeros(N_ARM)
                mujoco.mj_step(model, data)

            # Compute gravity torques from both libraries
            q_arm          = data.qpos[qpos_ids]
            tau_grav_pin   = gravity_torques_pin(pin_model, pin_data, q_arm)
            tau_grav_mujoco = data.qfrc_bias[dof_ids]
            max_err        = np.max(np.abs(tau_grav_pin - tau_grav_mujoco))

            now = time.time()
            if now - t_print > 0.5:
                x_cur   = data.site_xpos[ee_id]
                pin_str = " ".join(f"{t:+7.2f}" for t in tau_grav_pin)
                muj_str = " ".join(f"{t:+7.2f}" for t in tau_grav_mujoco)
                print(f"\r  EE=[{x_cur[0]:.3f},{x_cur[1]:.3f},{x_cur[2]:.3f}]"
                      f"  τ_pin=[{pin_str}]"
                      f"  τ_muj=[{muj_str}]"
                      f"  err={max_err:.4f}   ",
                      end="", flush=True)
                t_print = now

            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
