#!/usr/bin/env python3
"""Batch evaluation for the RoboCasa plugin.

Runs a policy across tasks, seeds and episodes headlessly, in one worker
process, and writes a CSV plus optional per-episode HDF5 recordings. This is
the entry point for comparing conditions, for example oracle versus sensor
perception:

    python evaluate.py --seeds 0-4 --policy builtin:grasp_place --perception oracle
    python evaluate.py --seeds 0-4 --policy builtin:grasp_place --perception sensors

Reported per episode: success (from the simulator, evaluation only), wall time,
tool calls issued and the failure signal of the last call.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

from policies import load as load_policy
from rpc_client import Worker

FIELDS = [
    "task", "seed", "episode", "policy", "perception",
    "success", "steps", "calls", "seconds", "last_action", "error",
]


def parse_seeds(text: str) -> list[int]:
    out: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = part.split("-", 1)
            out.extend(range(int(lo), int(hi) + 1))
        else:
            out.append(int(part))
    return out


def record_episode(path: str, trace: list[dict], meta: dict) -> None:
    """Write one episode as HDF5 when h5py exists, JSONL otherwise."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        import h5py  # noqa: PLC0415
        import numpy as np  # noqa: PLC0415
    except Exception:
        with open(path.replace(".hdf5", ".jsonl"), "w", encoding="utf-8") as fh:
            for row in trace:
                fh.write(json.dumps(row) + "\n")
        return
    with h5py.File(path, "w") as fh:
        for key, value in meta.items():
            fh.attrs[key] = value
        fh.create_dataset("eef_pos", data=np.array([r["eef_pos"] for r in trace], dtype=np.float32))
        fh.create_dataset("eef_quat", data=np.array([r["eef_quat"] for r in trace], dtype=np.float32))
        fh.create_dataset("success", data=np.array([r["success"] for r in trace], dtype=bool))
        fh.create_dataset("action", data=[r["action"] for r in trace])


def run_episode(worker: Worker, policy_cls, context: dict, max_calls: int, record: str | None, verbose: bool = False):
    policy = policy_cls(worker, context)
    trace: list[dict] = []
    calls = 0
    last_action = ""
    error = None
    started = time.time()
    try:
        while calls < max_calls:
            name = policy.act()
            if name is None:
                break
            calls += 1
            last_action = name
            if verbose:
                result = getattr(policy, "last_result", None) or {}
                brief = {k: result.get(k) for k in ("success", "failure", "residual_m", "released") if k in result}
                print(f"      {name}: {brief}")
            state = worker.call("status", {}, timeout=60)
            trace.append({
                "action": name,
                "eef_pos": state.get("eef_pos") or [0, 0, 0],
                "eef_quat": state.get("eef_quat") or [0, 0, 0, 1],
                "success": bool(state.get("success")),
            })
            if state.get("success"):
                break
    except Exception as exc:  # noqa: BLE001
        error = f"{type(exc).__name__}: {exc}"
    elapsed = time.time() - started
    state = {}
    try:
        state = worker.call("status", {}, timeout=60)
    except Exception:
        pass
    success = bool(state.get("success")) or (bool(trace) and trace[-1]["success"])
    if verbose:
        objects = state.get("objects") or {}
        if objects:
            brief = {k: [round(v, 3) for v in pos] for k, pos in objects.items()}
            print(f"      final objects: {brief}")
    if record:
        name = f"{context['task']}-s{context['seed']}-e{context['episode']}.hdf5"
        try:
            record_episode(os.path.join(record, name), trace, {**context, "success": success})
        except Exception as exc:  # noqa: BLE001
            error = error or f"record failed: {exc}"
    return {
        "task": context["task"],
        "seed": context["seed"],
        "episode": context["episode"],
        "policy": context["policy"],
        "perception": context["perception"],
        "success": success,
        "steps": state.get("steps", 0),
        "calls": calls,
        "seconds": round(elapsed, 2),
        "last_action": last_action,
        "error": error or "",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", default="PickPlaceCounterToCabinet")
    parser.add_argument("--seeds", default="0-4", help="0-4 or 0,2,5 (default 0-4)")
    parser.add_argument("--episodes", type=int, default=1, help="episodes per seed (default 1)")
    parser.add_argument("--policy", default="builtin:grasp_place", help="builtin:grasp_place or module:Class")
    parser.add_argument("--perception", choices=["oracle", "sensors"], default="oracle")
    parser.add_argument("--max-calls", type=int, default=12, help="tool calls allowed per episode")
    parser.add_argument("--record", default=None, help="directory for per-episode recordings")
    parser.add_argument("--camera", default="robot0_agentview_right",
                        help="observation camera (default robot0_agentview_right, which sees the counter)")
    parser.add_argument("--vision-query", default="object",
                        help="detector label used in sensor mode, e.g. carrot (default object)")
    parser.add_argument("--out", default="evaluation.csv", help="CSV path")
    parser.add_argument("--verbose", action="store_true", help="print each tool call's outcome")
    args = parser.parse_args()

    policy_cls = load_policy(args.policy)
    seeds = parse_seeds(args.seeds)
    rows: list[dict] = []
    worker = Worker()
    try:
        for seed in seeds:
            worker.call("open", {"task": args.task, "seed": seed, "perception": args.perception,
                                 "camera": args.camera}, timeout=600)
            for episode in range(args.episodes):
                if episode:
                    worker.call("reset", {}, timeout=600)
                start = worker.call("status", {}, timeout=60)
                context = {"start_eef": start.get("eef_pos")}
                row = run_episode(
                    worker, policy_cls,
                    {"task": args.task, "seed": seed, "episode": episode,
                     "policy": args.policy, "perception": args.perception,
                     "start_eef": context["start_eef"],
                     "vision_query": args.vision_query},
                    args.max_calls, args.record, args.verbose,
                )
                rows.append(row)
                print(f"  seed {seed} ep {episode}: success={row['success']} "
                      f"calls={row['calls']} {row['seconds']}s {row['error']}")
    finally:
        worker.close()

    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    total = len(rows)
    wins = sum(1 for r in rows if r["success"])
    print(f"\n{args.policy} on {args.task} [{args.perception}]: "
          f"{wins}/{total} = {wins / total:.0%}" if total else "no episodes")
    print(f"CSV: {os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
