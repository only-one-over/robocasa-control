#!/usr/bin/env python3
"""Environment health check for the RoboCasa plugin.

Standard library only, so it still runs when the simulation environment is
broken. Prints one JSON document to stdout:

    {"ok": bool, "checks": [{id, status, detail, hint}], "summary": {...}}

status is "pass", "warn" or "fail"; only "fail" makes ok false.
"""

from __future__ import annotations

import argparse
import contextlib
import ctypes.util
import importlib
import importlib.metadata as metadata
import io
import json
import shutil
import sys
import time
import traceback
from pathlib import Path

EXPECTED = {"numpy": "2.2.5", "mujoco": "3.3.1", "robosuite": "1.5.2"}
ASSET_SUBDIRS = ("objects", "fixtures", "textures", "scenes")
MIN_FREE_GB = 10.0

HINTS = {
    "numpy": "pip install numpy==2.2.5",
    "mujoco": "pip install mujoco==3.3.1",
    "robosuite": "pip install robosuite==1.5.2",
    "robocasa": "git clone https://github.com/robocasa/robocasa && cd robocasa && pip install -e .",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--deep", action="store_true", help="also build a tiny scene and render one frame")
    args = parser.parse_args()
    deep = args.deep
    checks: list[dict] = []

    def add(check_id: str, status: str, detail: str, hint: str | None = None) -> None:
        entry = {"id": check_id, "status": status, "detail": detail}
        if hint:
            entry["hint"] = hint
        checks.append(entry)

    # 1. interpreter
    v = sys.version_info
    if v >= (3, 10):
        add("python", "pass", f"Python {v.major}.{v.minor}.{v.micro}")
    else:
        add("python", "fail", f"Python {v.major}.{v.minor}.{v.micro} is too old", "robocasa needs Python >= 3.10")

    # 2. imports
    versions: dict[str, str] = {}
    modules: dict[str, object] = {}
    for mod in ("numpy", "mujoco", "robosuite", "robocasa"):
        try:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                modules[mod] = importlib.import_module(mod)
            try:
                version = metadata.version(mod)
            except Exception:
                version = getattr(modules[mod], "__version__", "unknown")
            versions[mod] = str(version)
            add(f"import:{mod}", "pass", f"{mod} {version}")
        except Exception as exc:  # noqa: BLE001 - report anything the import raises
            add(f"import:{mod}", "fail", f"{type(exc).__name__}: {exc}", HINTS.get(mod))

    # 3. pinned versions
    for mod, expected in EXPECTED.items():
        actual = versions.get(mod)
        if actual is None:
            continue
        if actual == expected:
            add(f"version:{mod}", "pass", f"{mod} {actual} matches the pinned version")
        else:
            add(f"version:{mod}", "warn", f"{mod} {actual} != expected {expected}", HINTS.get(mod))

    # 4. assets
    assets = None
    robocasa = modules.get("robocasa")
    if robocasa is not None:
        root = Path(robocasa.__file__).resolve().parent / "models" / "assets"
        assets = root
        if not root.is_dir():
            add("assets", "fail", f"asset directory missing: {root}",
                "python -m robocasa.scripts.download_kitchen_assets")
        else:
            missing = [name for name in ASSET_SUBDIRS if not (root / name).is_dir()]
            if missing:
                add("assets", "warn", f"{root} is missing: {', '.join(missing)}",
                    "python -m robocasa.scripts.download_kitchen_assets")
            else:
                add("assets", "pass", f"assets present at {root} ({', '.join(ASSET_SUBDIRS)})")

    # 5. free disk space
    probe = assets if assets is not None and assets.exists() else Path.home()
    try:
        free_gb = shutil.disk_usage(str(probe)).free / 1024 ** 3
        if free_gb >= MIN_FREE_GB:
            add("disk", "pass", f"{free_gb:.1f} GB free on {probe}")
        else:
            add("disk", "warn", f"only {free_gb:.1f} GB free on {probe}",
                "RoboCasa assets and datasets can take tens of GB")
    except Exception as exc:  # noqa: BLE001
        add("disk", "warn", f"could not measure free space: {exc}")

    # 6. headless rendering backend
    import os

    gl = os.environ.get("MUJOCO_GL")
    egl = ctypes.util.find_library("EGL")
    if gl:
        add("render", "pass", f"MUJOCO_GL={gl}")
    elif egl:
        add("render", "pass", f"libEGL found ({egl}); headless rendering should work")
    else:
        add("render", "warn", "no MUJOCO_GL and no libEGL found",
            "install libegl1 or set MUJOCO_GL=osmesa")

    # 7. optional deep check
    if deep:
        try:
            t0 = time.time()
            from robocasa.utils.env_utils import create_env  # noqa: PLC0415

            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                env = create_env(
                    "PickPlaceCounterToCabinet",
                    camera_names=["robot0_agentview_left"],
                    camera_widths=64,
                    camera_heights=64,
                    seed=0,
                    render_onscreen=False,
                )
                env.reset()
                img = env.sim.render(width=64, height=64, camera_name="robot0_agentview_left", depth=False)
                env.close()
            import numpy as np

            arr = np.asarray(img)
            if arr.size and int(arr.max()) > 0:
                add("deep:render", "pass", f"rendered a 64x64 frame in {time.time() - t0:.1f}s")
            else:
                add("deep:render", "warn", "render returned an empty image")
        except Exception as exc:  # noqa: BLE001
            add("deep:render", "fail", f"{type(exc).__name__}: {exc}", traceback.format_exc(limit=2)[-400:])

    summary = {status: sum(1 for c in checks if c["status"] == status) for status in ("pass", "warn", "fail")}
    result = {
        "ok": summary["fail"] == 0,
        "python": sys.executable,
        "checks": checks,
        "summary": summary,
    }
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
