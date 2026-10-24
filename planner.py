"""Motion planning primitives for the RoboCasa worker.

This module reimplements the parts of a standard motion-planning stack
(MoveIt's mathematical core) on top of MuJoCo, which already provides
forward kinematics, Jacobians and a collision model:

    kinematics   mj_forward  -> site_xpos / site_xmat
    jacobian     mj_jacSite  -> 6 x nv
    collision    contact scan after mj_forward (same model the sim uses)

We add what MuJoCo does not provide: damped least squares inverse
kinematics, a joint-space RRT-Connect planner, path shortcutting and time
parameterisation.

The planner never mutates the simulation: it saves and restores qpos
around every query, so callers can plan while a scene is live.
"""

from __future__ import annotations

import math
import time

import mujoco
import numpy as np


class RobotModel:
    """Joint addresses and limits for one arm of a robosuite robot."""

    def __init__(self, env, part: str = "right"):
        self.env = env
        self.sim = env.sim
        self.model = getattr(self.sim.model, "_model", self.sim.model)
        self.data = getattr(self.sim.data, "_data", self.sim.data)
        self.part = part
        robot = env.robots[0]
        self.prefix = robot.robot_model.naming_prefix

        names = [f"{self.prefix}joint{i}" for i in range(1, 8)]
        self.joint_ids = [self.model.joint(n).id for n in names]
        self.qpos_adr = np.array([self.model.jnt_qposadr[j] for j in self.joint_ids], dtype=int)
        self.dof_adr = np.array([self.model.jnt_dofadr[j] for j in self.joint_ids], dtype=int)
        self.lower = np.array([self.model.jnt_range[j][0] for j in self.joint_ids], dtype=float)
        self.upper = np.array([self.model.jnt_range[j][1] for j in self.joint_ids], dtype=float)

        site = robot.eef_site_id
        self.eef_site = int(site[part]) if isinstance(site, dict) else int(site)
        # The IK site and the observation's eef frame differ by a fixed rotation
        # (90 deg about z for the Panda hand). Calibrate it once from a live
        # observation so IK targets and observation-based checks agree.
        self.frame_offset = np.eye(3)
        self.nv = self.model.nv
        self._saved = None

    # ---- state -----------------------------------------------------------
    def get_q(self) -> np.ndarray:
        return self.data.qpos[self.qpos_adr].copy()

    def set_q(self, q: np.ndarray) -> None:
        self.data.qpos[self.qpos_adr] = np.clip(np.asarray(q, dtype=float), self.lower, self.upper)
        mujoco.mj_forward(self.model, self.data)

    def save(self) -> None:
        self._saved = self.data.qpos.copy()

    def restore(self) -> None:
        if self._saved is not None:
            self.data.qpos[:] = self._saved
            mujoco.mj_forward(self.model, self.data)

    # ---- kinematics ------------------------------------------------------
    def calibrate(self, observed_rot: np.ndarray) -> None:
        """Record the rotation between the IK site frame and the observed eef frame."""
        _, site_rot = self.fk_raw()
        offset = site_rot.T @ np.asarray(observed_rot, dtype=float)
        # Snap near-axis-aligned offsets so tiny numerical noise cannot drift.
        offset = np.round(offset, 6)
        self.frame_offset = offset

    def fk_raw(self) -> tuple[np.ndarray, np.ndarray]:
        pos = self.data.site_xpos[self.eef_site].copy()
        rot = self.data.site_xmat[self.eef_site].reshape(3, 3).copy()
        return pos, rot

    def fk(self) -> tuple[np.ndarray, np.ndarray]:
        """End-effector pose in the frame the observations report."""
        pos, rot = self.fk_raw()
        return pos, rot @ self.frame_offset

    def fk_of(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.set_q(q)
        return self.fk()

    def within_limits(self, q: np.ndarray, margin: float = 0.15) -> bool:
        """True when every joint keeps a safety margin from its bounds."""
        q = np.asarray(q, dtype=float)
        return bool(np.all(q >= self.lower + margin) and np.all(q <= self.upper - margin))

    def ik_multi(self, target_pos, target_rot=None, tries: int = 12, margin: float = 0.15,
                 seed: int = 0, collect: int = 5, **kwargs) -> tuple[np.ndarray | None, dict]:
        """Multi-start IK that keeps solutions away from limits and near the start.

        Several valid seeds usually give different branches (elbow up/down). The
        one closest to the current configuration needs the least motion, which
        matters when the arm is servo-driven: a far branch swings through a long
        arc and may not settle within the step budget.
        """
        q_ref = self.get_q().copy()
        rng = np.random.default_rng(seed)
        low = self.lower + margin
        high = self.upper - margin
        seeds = [q_ref]
        for _ in range(max(0, tries - 1)):
            seeds.append(rng.uniform(low, high))
        solutions = []
        last = {}
        for index, q_init in enumerate(seeds):
            q, info = self.ik(target_pos, target_rot=target_rot, q_init=q_init, **kwargs)
            last = info
            if q is None:
                continue
            if not self.within_limits(q, margin):
                continue
            solutions.append((float(np.linalg.norm(q - q_ref)), index, q, info))
            if len(solutions) >= collect:
                break
        if not solutions:
            return None, {**last, "within_limits": False,
                          "error": "no solution away from joint limits"}
        solutions.sort(key=lambda item: item[0])
        dist, index, q, info = solutions[0]
        return q, {**info, "seed_index": index, "within_limits": True,
                   "joint_travel": round(dist, 3), "candidates": len(solutions)}

    def jacobian(self) -> np.ndarray:
        jacp = np.zeros((3, self.nv))
        jacr = np.zeros((3, self.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.eef_site)
        return np.vstack([jacp[:, self.dof_adr], jacr[:, self.dof_adr]])

    def ik(self, target_pos, target_rot=None, q_init=None, iters: int = 80,
           tol_pos: float = 2e-3, tol_rot: float = 0.03, damping: float = 0.08,
           max_step: float = 0.4) -> tuple[np.ndarray | None, dict]:
        """Damped least squares IK; returns the solution and diagnostics."""
        self.save()
        try:
            return self._ik_inner(target_pos, target_rot, q_init, iters, tol_pos, tol_rot, damping, max_step)
        finally:
            self.restore()

    def _ik_inner(self, target_pos, target_rot, q_init, iters, tol_pos, tol_rot, damping, max_step):
        q = (self.get_q() if q_init is None else np.array(q_init, dtype=float)).copy()
        q = np.clip(q, self.lower, self.upper)
        target_pos = np.asarray(target_pos, dtype=float)
        best = (math.inf, q.copy(), math.inf, math.inf)
        for _ in range(iters):
            self.set_q(q)
            pos, rot = self.fk_raw()
            err_pos = target_pos - pos
            if target_rot is None:
                err_rot = 0.0
                err_vec = err_pos
            else:
                err_mat = np.asarray(target_rot, dtype=float) @ self.frame_offset.T @ rot.T
                err_vec_rot = _rotvec(err_mat)
                err_rot = float(np.linalg.norm(err_vec_rot))
                err_vec = np.concatenate([err_pos, err_vec_rot])
            err_norm = float(np.linalg.norm(err_pos))
            if err_norm < best[0]:
                best = (err_norm, q.copy(), err_rot, err_norm)
            if err_norm < tol_pos and err_rot < tol_rot:
                break
            jac = self.jacobian()
            if target_rot is None:
                jac = jac[:3]
            jjt = jac @ jac.T + (damping ** 2) * np.eye(jac.shape[0])
            dq = jac.T @ np.linalg.solve(jjt, err_vec)
            scale = float(np.max(np.abs(dq))) if dq.size else 0.0
            if scale > max_step:
                dq *= max_step / scale
            q = np.clip(q + dq, self.lower, self.upper)
        err_norm, q_best, err_rot, _ = best
        ok = err_norm < tol_pos * 3 and (target_rot is None or err_rot < tol_rot * 3)
        return (q_best if ok else None), {
            "residual": round(err_norm, 5),
            "rot_residual": round(err_rot, 5) if target_rot is not None else None,
            "iterations": iters,
        }


class CollisionChecker:
    """Contact-based collision queries against the live MuJoCo model."""

    def __init__(self, robot: RobotModel, allowed_geom_names=(), depth_tolerance: float = 0.005):
        self.robot = robot
        # Contacts shallower than this are treated as free space: the sim
        # allows small penetrations and a planner that rejects 2 mm would
        # call reachable shelves unreachable.
        self.depth_tolerance = depth_tolerance
        self.model = robot.model
        self.data = robot.data
        self.prefix = robot.prefix
        self.allowed = set()
        # Geoms of the object in the gripper: colliding with the world counts as a
        # collision, but they must not be reported as robot-vs-object touches.
        self.attached = set()
        for name in allowed_geom_names:
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if gid >= 0:
                self.allowed.add(int(gid))
        self.checks = 0

    def set_allowed_geoms(self, geom_ids) -> None:
        """Treat these geoms as free space, e.g. the object being carried."""
        self.allowed = set(int(g) for g in geom_ids)

    def add_allowed_geoms(self, geom_ids) -> None:
        self.allowed.update(int(g) for g in geom_ids)

    def set_attached_geoms(self, geom_ids) -> None:
        """Mark carried geoms: they collide with the world like part of the robot."""
        self.attached = set(int(g) for g in geom_ids)

    def _is_robot(self, geom_id: int) -> bool:
        if int(geom_id) in self.attached:
            return True
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        return name.startswith(self.prefix)

    def in_collision(self, q: np.ndarray) -> bool:
        self.robot.set_q(q)
        self.checks += 1
        data = self.data
        for i in range(data.ncon):
            contact = data.contact[i]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            r1, r2 = self._is_robot(g1), self._is_robot(g2)
            if r1 == r2:
                continue  # self-collision or environment-environment
            other = g2 if r1 else g1
            if other in self.allowed:
                continue
            if float(contact.dist) > -self.depth_tolerance:
                continue
            return True
        return False

    def contacts(self, q: np.ndarray) -> list[tuple[str, str, float]]:
        """Colliding pairs at a configuration, for diagnostics."""
        self.robot.set_q(q)
        data = self.data
        out = []
        for i in range(data.ncon):
            contact = data.contact[i]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            r1, r2 = self._is_robot(g1), self._is_robot(g2)
            if r1 == r2:
                continue
            other = g2 if r1 else g1
            if other in self.allowed:
                continue
            name1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g1) or str(g1)
            name2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g2) or str(g2)
            out.append((name1, name2, float(contact.dist)))
        return out

    def segment_free(self, q1: np.ndarray, q2: np.ndarray, step: float = 0.05) -> bool:
        q1 = np.asarray(q1, dtype=float)
        q2 = np.asarray(q2, dtype=float)
        dist = float(np.max(np.abs(q2 - q1)))
        count = max(1, int(math.ceil(dist / step)))
        for i in range(1, count + 1):
            if self.in_collision(q1 + (q2 - q1) * (i / count)):
                return False
        return True


class Planner:
    """RRT-Connect in the 7-dof arm space with shortcut smoothing."""

    def __init__(self, robot: RobotModel, checker: CollisionChecker, seed: int = 0):
        self.robot = robot
        self.checker = checker
        self.rng = np.random.default_rng(seed)

    def plan(self, q_start, q_goal, max_iters: int = 3000, step: float = 0.12,
             check_step: float = 0.06) -> tuple[list[np.ndarray] | None, dict]:
        self.robot.save()
        try:
            return self._plan_inner(q_start, q_goal, max_iters, step, check_step)
        finally:
            self.robot.restore()

    def _plan_inner(self, q_start, q_goal, max_iters, step, check_step):
        t0 = time.time()
        start_checks = self.checker.checks
        q_start = np.asarray(q_start, dtype=float)
        q_goal = np.asarray(q_goal, dtype=float)
        info = {"checks": 0, "tree_a": 0, "tree_b": 0, "iters": 0}

        if self.checker.in_collision(q_start):
            return None, {**info, "error": "start in collision",
                          "checks": self.checker.checks - start_checks}
        if self.checker.in_collision(q_goal):
            return None, {**info, "error": "goal in collision",
                          "checks": self.checker.checks - start_checks}
        if self.checker.segment_free(q_start, q_goal, check_step):
            return [q_start, q_goal], {
                **info, "nodes": 2, "seconds": round(time.time() - t0, 3),
                "checks": self.checker.checks - start_checks, "direct": True,
            }

        tree_a = [(q_start, -1)]
        tree_b = [(q_goal, -1)]
        # The trees swap every iteration, so remember which one is rooted at
        # the start: the joined path comes out end-to-start otherwise.
        a_is_start = True
        lower, upper = self.robot.lower, self.robot.upper

        for iteration in range(max_iters):
            q_rand = self.rng.uniform(lower, upper)
            if self._extend(tree_a, q_rand, step, check_step):
                idx_a = len(tree_a) - 1
                q_new = tree_a[idx_a][0]
                reached = self._connect(tree_b, q_new, step, check_step)
                if reached is not None:
                    path = self._join(tree_a, idx_a, tree_b, reached)
                    if not a_is_start:
                        path.reverse()
                    path = self._shortcut(path, check_step)
                    return path, {
                        "nodes": len(path), "iters": iteration + 1,
                        "checks": self.checker.checks - start_checks,
                        "seconds": round(time.time() - t0, 3), "direct": False,
                        "tree_a": len(tree_a), "tree_b": len(tree_b),
                    }
            tree_a, tree_b = tree_b, tree_a
            a_is_start = not a_is_start
            info["iters"] = iteration + 1

        return None, {**info, "error": "no path", "checks": self.checker.checks - start_checks,
                      "seconds": round(time.time() - t0, 3)}

    def _nearest(self, tree, q):
        dists = [float(np.linalg.norm(node[0] - q)) for node in tree]
        idx = int(np.argmin(dists))
        return idx, tree[idx][0]

    def _steer(self, q_from, q_to, step):
        delta = q_to - q_from
        dist = float(np.linalg.norm(delta))
        if dist <= step:
            return q_to.copy()
        return q_from + delta * (step / dist)

    def _extend(self, tree, q_target, step, check_step) -> bool:
        idx, q_near = self._nearest(tree, q_target)
        q_new = self._steer(q_near, q_target, step)
        if not self.checker.segment_free(q_near, q_new, check_step):
            return False
        tree.append((q_new, idx))
        return True

    def _connect(self, tree, q_target, step, check_step):
        while True:
            idx, q_near = self._nearest(tree, q_target)
            if float(np.linalg.norm(q_target - q_near)) < 1e-6:
                return idx
            q_new = self._steer(q_near, q_target, step)
            if not self.checker.segment_free(q_near, q_new, check_step):
                return None
            tree.append((q_new, idx))
            if float(np.linalg.norm(q_target - q_new)) < 1e-6:
                return len(tree) - 1

    def _join(self, tree_a, idx_a, tree_b, idx_b):
        path_a = []
        idx = idx_a
        while idx >= 0:
            path_a.append(tree_a[idx][0])
            idx = tree_a[idx][1]
        path_a.reverse()
        path_b = []
        idx = idx_b
        while idx >= 0:
            path_b.append(tree_b[idx][0])
            idx = tree_b[idx][1]
        return path_a + path_b

    def _shortcut(self, path, check_step, rounds: int = 60):
        path = list(path)
        for _ in range(rounds):
            if len(path) < 3:
                break
            i = int(self.rng.integers(0, len(path) - 2))
            j = int(self.rng.integers(i + 2, len(path)))
            if self.checker.segment_free(path[i], path[j], check_step):
                path = path[: i + 1] + path[j:]
        return path


def time_parameterise(path, dt: float = 0.05, qd_max: float = 1.2):
    """Sample a joint path at a fixed control period with a velocity limit."""
    out = [np.asarray(path[0], dtype=float)]
    for q1, q2 in zip(path[:-1], path[1:]):
        q1 = np.asarray(q1, dtype=float)
        q2 = np.asarray(q2, dtype=float)
        span = float(np.max(np.abs(q2 - q1)))
        duration = max(dt, span / qd_max)
        steps = max(1, int(math.ceil(duration / dt)))
        for i in range(1, steps + 1):
            out.append(q1 + (q2 - q1) * (i / steps))
    return out


def _rotvec(mat: np.ndarray) -> np.ndarray:
    """Axis-angle vector of a rotation matrix."""
    cos = (np.trace(mat) - 1.0) / 2.0
    cos = float(np.clip(cos, -1.0, 1.0))
    angle = math.acos(cos)
    if angle < 1e-8:
        return np.zeros(3)
    axis = np.array([mat[2, 1] - mat[1, 2], mat[0, 2] - mat[2, 0], mat[1, 0] - mat[0, 1]])
    norm = float(np.linalg.norm(axis))
    if norm < 1e-8:
        return np.zeros(3)
    return axis / norm * angle
