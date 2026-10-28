#!/usr/bin/env python3
"""Headless smoke test for the simulation worker.

Spawns worker.py exactly as the plugin does and drives it over the same
JSON-RPC protocol, so it exercises the real code path rather than a copy:

    python self_check.py [--seed 4] [--task PickPlaceCounterToCabinet]

Each step reports PASS/FAIL with timing; the exit code is non-zero when any
step fails. Use it after changing the worker, the controller or the planner.
"""

from __future__ import annotations

import argparse
import sys
import time

from rpc_client import Worker


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=4)
    parser.add_argument("--task", default="PickPlaceCounterToCabinet")
    args = parser.parse_args()

    results: list[tuple[str, bool, float, str]] = []

    def step(label, fn):
        start = time.time()
        try:
            detail = fn() or ""
            results.append((label, True, time.time() - start, str(detail)))
            return True
        except Exception as exc:  # noqa: BLE001
            results.append((label, False, time.time() - start, f"{type(exc).__name__}: {exc}"))
            return False

    worker = Worker()
    try:
        state: dict = {}

        def do_open():
            value = worker.call("open", {"task": args.task, "seed": args.seed})
            state["eef"] = value.get("eef_pos")
            return f"eef {_fmt(value.get('eef_pos'))}"

        if not step("open", do_open):
            return _report(results)

        def do_status():
            value = worker.call("status")
            state["objects"] = value.get("objects") or {}
            return f"step {value.get('steps')}"

        step("status", do_status)

        # A short lift from the home pose: always reachable and free of scenery,
        # so a failure here means the motion stack broke rather than the scene.
        eef = state.get("eef")
        target = [float(eef[0]), float(eef[1]), float(eef[2]) + 0.08] if eef else None

        if target:
            def do_plan():
                value = worker.call("plan", {"x": target[0], "y": target[1], "z": target[2]})
                if not value.get("ok"):
                    raise RuntimeError(value.get("failure"))
                return f"{value['nodes']} nodes, {value['checks']} checks, {value['seconds']}s"

            step("plan", do_plan)

            def do_move():
                value = worker.call("move", {"x": target[0], "y": target[1], "z": target[2],
                                             "gripper": "open", "max_steps": 80, "tol": 0.02})
                if value.get("failure") not in (None, "ok"):
                    raise RuntimeError(f"{value['failure']} residual {value['residual_m']}")
                return f"residual {value['residual_m']} m"

            step("move", do_move)

        def do_look():
            value = worker.call("look", {})
            if not value.get("image_png_base64"):
                raise RuntimeError("no frame")
            return f"{len(value['image_png_base64'])} bytes"

        step("look", do_look)

        def do_drop():
            value = worker.call("drop", {"x": target[0], "y": target[1], "z": target[2]})
            if value.get("failure") not in (None, "ok"):
                raise RuntimeError(f"{value['failure']} residual {value['residual_m']}")
            return f"residual {value.get('residual_m')} m"

        step("drop", do_drop)

        def do_restart():
            worker.call("restart")
            steps = worker.call("status").get("steps")
            if steps != 0:
                raise RuntimeError(f"steps not reset: {steps}")
            return "state restored"

        step("restart", do_restart)
        step("close", lambda: worker.call("close") and "closed")
    finally:
        worker.close()

    return _report(results)


def _fmt(pos) -> str:
    if not pos:
        return "n/a"
    return "(" + ", ".join(f"{float(v):.3f}" for v in pos) + ")"


def _report(results) -> int:
    print("\n=== robocasa self check ===")
    failed = 0
    for label, ok, elapsed, detail in results:
        status = "PASS" if ok else "FAIL"
        if not ok:
            failed += 1
        print(f"  {status}  {label:<8} {elapsed:6.2f}s  {detail}")
    print(f"\n{len(results) - failed}/{len(results)} passed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
