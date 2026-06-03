"""
Cartesian Impedance Control for xArm7.

    F_cart = K_p*(x_d - x) + K_d*(0 - dx)   [translational]
           + K_r*e_rot + D_r*(0 - w)          [rotational]

    tau = J^T * F_cart  +  N * tau_null  +  tau_gravity

where N = I - J^+ J is the arm's nullspace projector.
"""

import mujoco
import numpy as np

# ── Impedance gains ────────────────────────────────────────────────────────────
Kp = np.diag([800., 800., 800.])     # translational stiffness  [N/m]
Dp = np.diag([80.,  80.,  80. ])     # translational damping     [N·s/m]
Kr = np.diag([30.,  30.,  30. ])     # rotational stiffness      [N·m/rad]
Dr = np.diag([6.,   6.,   6.  ])     # rotational damping        [N·m·s/rad]

K_null = 5.0    # nullspace stiffness
D_null = 1.0    # nullspace damping


def skew(v: np.ndarray) -> np.ndarray:
    """3×3 skew-symmetric matrix for cross-product."""
    return np.array([[0,    -v[2],  v[1]],
                     [v[2],  0,    -v[0]],
                     [-v[1], v[0],  0   ]])


def rotation_error(R_cur: np.ndarray, R_des: np.ndarray) -> np.ndarray:
    """Orientation error as axis-angle vector (world frame)."""
    R_err = R_des @ R_cur.T
    return 0.5 * np.array([R_err[2,1] - R_err[1,2],
                            R_err[0,2] - R_err[2,0],
                            R_err[1,0] - R_err[0,1]])


def impedance_torques(model:    mujoco.MjModel,
                      data:     mujoco.MjData,
                      dof_ids:  np.ndarray,
                      qpos_ids: np.ndarray,
                      ee_id:    int,
                      x_des:    np.ndarray,
                      R_des:    np.ndarray,
                      q_home:   np.ndarray) -> np.ndarray:
    """Return joint torques (n_arm,) for Cartesian impedance control."""
    nv    = model.nv
    n_arm = len(dof_ids)

    # ── End-effector state ────────────────────────────────────────────────────
    x_cur = data.site_xpos[ee_id].copy()
    R_cur = data.site_xmat[ee_id].reshape(3, 3)

    # ── Full Jacobian ─────────────────────────────────────────────────────────
    Jfull_p = np.zeros((3, nv))
    Jfull_r = np.zeros((3, nv))
    mujoco.mj_jacSite(model, data, Jfull_p, Jfull_r, ee_id)

    Jp = Jfull_p[:, dof_ids]   # (3, n_arm)
    Jr = Jfull_r[:, dof_ids]   # (3, n_arm)
    J  = np.vstack([Jp, Jr])   # (6, n_arm)

    # ── Joint state ───────────────────────────────────────────────────────────
    q        = data.qpos[qpos_ids]
    dq       = data.qvel[dof_ids]
    ee_vel_p = Jp @ dq
    ee_vel_r = Jr @ dq

    # ── Errors ────────────────────────────────────────────────────────────────
    pos_err = x_des - x_cur
    rot_err = rotation_error(R_cur, R_des)

    # ── Cartesian wrench ──────────────────────────────────────────────────────
    F_p = Kp @ pos_err - Dp @ ee_vel_p
    F_r = Kr @ rot_err - Dr @ ee_vel_r
    F   = np.concatenate([F_p, F_r])   # (6,)

    # ── Primary torques ───────────────────────────────────────────────────────
    tau = J.T @ F

    # ── Nullspace: attract to home pose ──────────────────────────────────────
    J_pinv = np.linalg.pinv(J)
    N      = np.eye(n_arm) - J_pinv @ J
    tau   += N @ (K_null * (q_home - q) - D_null * dq)

    # ── Gravity compensation ──────────────────────────────────────────────────
    tau += data.qfrc_bias[dof_ids]

    return tau
