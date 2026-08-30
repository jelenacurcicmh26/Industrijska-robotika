"""Headless driver around the step-3 controller.

Both tools/benchmark.py and tools/record_media.py run the simulation through
this, so the figures and the GIFs come from exactly the control code that
step3_computed_torque.py runs interactively. Nothing is reimplemented here.
"""

import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import step3_computed_torque as s3   # noqa: E402


class ArmSim:
    """One MuJoCo instance running one of the three controllers."""

    def __init__(self, mode, trajectory="quintic", start_index=1, gravity_only=False):
        self.mode = mode
        self.trajectory = trajectory
        self.gravity_only = gravity_only

        self.model = mujoco.MjModel.from_xml_path(s3.MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        _, self.dof_ids, self.qpos_ids = s3.build_robot_info(self.model)
        self.pin_model, self.pin_data = s3.build_pinocchio(s3.PIN_MODEL_PATH)

        self.tau_lim = self.model.actuator_ctrlrange[:s3.N_ARM, 1].copy()
        self.damping = self.model.dof_damping[self.dof_ids].copy()
        self.friction = self.model.dof_frictionloss[self.dof_ids].copy()
        self.dt = self.model.opt.timestep

        self.data.qpos[self.qpos_ids] = s3.HOME_QPOS
        mujoco.mj_forward(self.model, self.data)

        # Start on the first waypoint that is not the home pose, otherwise the
        # first segment is 2.5 s of the arm sitting where it already is.
        self.wp_index = start_index
        self.seg = self._segment_to(s3.WAYPOINTS[self.wp_index],
                                    s3.HOME_QPOS, np.zeros(7), np.zeros(7))
        self.t_seg = 0.0
        self.stats = s3.TrackingStats()
        self.parts = {}

    def _segment_to(self, goal, q0, v0, a0):
        return s3.QuinticSegment(q0, v0, a0, goal, s3.T_MOVE)

    def reference(self):
        """(q_d, q̇_d, q̈_d) at the current sim time."""
        if self.trajectory == "step":
            # What korak 2 did: jump to the goal, no velocity or acceleration.
            z = np.zeros(s3.N_ARM)
            return self.seg.qf.copy(), z, z.copy()
        return self.seg.at(self.data.time - self.t_seg)

    def advance_waypoint_if_due(self):
        if (self.data.time - self.t_seg) >= s3.T_MOVE + s3.DWELL_TIME:
            q_d, qd_d, qdd_d = self.seg.at(self.data.time - self.t_seg)
            self.wp_index = (self.wp_index + 1) % len(s3.WAYPOINTS)
            self.seg = self._segment_to(s3.WAYPOINTS[self.wp_index], q_d, qd_d, qdd_d)
            self.t_seg = self.data.time
            return True
        return False

    def control(self):
        """Compute tau and record its individual contributions in self.parts."""
        q_d, qd_d, qdd_d = self.reference()
        q = self.data.qpos[self.qpos_ids]
        dq = self.data.qvel[self.dof_ids]
        e, de = q_d - q, qd_d - dq

        if self.gravity_only:
            tau = np.zeros(s3.N_ARM)
            self.parts = {}
        elif self.mode == s3.MODE_PD:
            g = s3.gravity_torques_pin(self.pin_model, self.pin_data, q)
            tau = g + s3.KP_PD @ e + s3.KD_PD @ de
            self.parts = {"gravity": g, "feedback": s3.KP_PD @ e + s3.KD_PD @ de}
        else:
            M, n = s3.arm_dynamics_pin(self.pin_model, self.pin_data, q, dq)
            a_ref = qdd_d + s3.KD @ de + s3.KP @ e
            tau_inertial = M @ a_ref
            tau = tau_inertial + n
            self.parts = {"inertial": tau_inertial, "bias": n,
                          "damping": np.zeros(s3.N_ARM),
                          "coulomb": np.zeros(s3.N_ARM)}
            if self.mode == s3.MODE_CT_FULL:
                d_t = self.damping * dq
                c_t = s3.FRICTION_COMP * self.friction * np.tanh(dq / s3.V_EPS)
                tau = tau + d_t + c_t
                self.parts["damping"] = d_t
                self.parts["coulomb"] = c_t

        return tau, e

    def step(self):
        """One control tick + one physics step. Returns (error, tau, saturated)."""
        self.advance_waypoint_if_due()
        tau, e = self.control()
        tau_c = np.clip(tau, -self.tau_lim, self.tau_lim)
        saturated = bool(np.any(tau_c != tau))
        self.data.ctrl[:s3.N_ARM] = tau_c
        mujoco.mj_step(self.model, self.data)
        self.stats.add(e)
        return e, tau, saturated


LAP_SECONDS = len(s3.WAYPOINTS) * (s3.T_MOVE + s3.DWELL_TIME)


def run(mode, seconds=LAP_SECONDS, **kwargs):
    """Run one controller and return a dict of logged time series."""
    sim = ArmSim(mode, **kwargs)
    n = int(round(seconds / sim.dt))
    log = {k: [] for k in ("t", "q", "q_d", "e", "tau", "sat")}
    parts = {}

    for _ in range(n):
        q_d, _, _ = sim.reference()
        q = sim.data.qpos[sim.qpos_ids].copy()
        t = sim.data.time
        e, tau, sat = sim.step()
        log["t"].append(t)
        log["q"].append(q)
        log["q_d"].append(q_d)
        log["e"].append(e.copy())
        log["tau"].append(tau.copy())
        log["sat"].append(sat)
        for k, v in sim.parts.items():
            parts.setdefault(k, []).append(v.copy())

    out = {k: np.array(v) for k, v in log.items()}
    out["parts"] = {k: np.array(v) for k, v in parts.items()}
    out["rms_deg"] = sim.stats.rms_deg
    out["peak_deg"] = sim.stats.peak_deg
    out["sat_pct"] = 100.0 * float(np.mean(out["sat"]))
    return out
