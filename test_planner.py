"""Offline check for the planning primitives: no plugin, no worker.

Opens a scene, plans from the current configuration to a target pose and
reports timing, collision counts and the achieved end-effector error.
"""

from __future__ import annotations

import time

import numpy as np

from robocasa.utils.env_utils import create_env

from planner import CollisionChecker, Planner, RobotModel, time_parameterise


def main() -> None:
    env = create_env(
        "PickPlaceCounterToCabinet",
        camera_names=["robot0_agentview_left"],
        camera_widths=320,
        camera_heights=320,
        seed=5,
        render_onscreen=False,
    )
    env.reset()

    robot = RobotModel(env)
    checker = CollisionChecker(robot)
    planner = Planner(robot, checker, seed=7)

    q_start = robot.get_q()
    pos0, rot0 = robot.fk()
    print(f"start q      : {np.round(q_start, 3)}")
    print(f"start eef    : {np.round(pos0, 4)}")

    t0 = time.time()
    n = 200
    for _ in range(n):
        checker.in_collision(q_start)
    per_check = (time.time() - t0) / n
    print(f"collision check: {per_check * 1000:.2f} ms each")

    targets = [
        ("above object", np.array([5.0861, -2.9244, 1.10])),
        ("cabinet shelf", np.array([5.21, -2.60, 1.45])),
        ("far left", np.array([4.70, -3.10, 1.05])),
        ("low front", np.array([5.00, -2.60, 0.85])),
        ("behind toaster", np.array([5.47, -2.95, 1.02])),
        ("behind toaster high", np.array([5.47, -2.95, 1.15])),
        ("right of toaster", np.array([5.42, -3.20, 1.05])),
    ]

    for label, target in targets:
        print(f"\n--- {label}: {np.round(target, 3)} ---")
        t1 = time.time()
        q_goal, ik_info = robot.ik(target, q_init=q_start)
        ik_time = time.time() - t1
        if q_goal is None:
            print(f"  IK failed: {ik_info} ({ik_time:.2f}s)")
            continue
        pos1, _ = robot.fk_of(q_goal)
        err_cm = float(np.linalg.norm(pos1 - target)) * 100
        print(f"  IK: {ik_info} err {err_cm:.2f} cm ({ik_time:.2f}s)")

        before = checker.checks
        t2 = time.time()
        path, info = planner.plan(q_start, q_goal)
        elapsed = time.time() - t2
        used = checker.checks - before
        if path is None:
            print(f"  plan: FAILED {info.get('error')} ({elapsed:.2f}s, {used} checks)")
            if info.get("error") == "goal in collision":
                for pair in checker.contacts(q_goal)[:5]:
                    print(f"     contact {pair[0]} <-> {pair[1]} dist={pair[2]:.4f}")
            continue
        dense = time_parameterise(path, dt=0.05, qd_max=1.2)
        bad = sum(1 for q in dense if checker.in_collision(q))
        kind = "direct" if info.get("direct") else "RRT"
        print(f"  plan: {kind}, {len(path)} nodes, {len(dense)} dense samples, "
              f"{used} checks, {elapsed:.2f}s, colliding {bad}")
        robot.set_q(q_goal)

    robot.set_q(q_start)
if __name__ == "__main__":
    main()
