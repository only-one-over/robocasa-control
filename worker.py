#!/usr/bin/env python3
"""RoboCasa simulation worker.

Speaks newline-delimited JSON-RPC over stdin/stdout:

    request : {"id": 1, "method": "open", "params": {...}}
    response: {"id": 1, "ok": true,  "result": {...}}
              {"id": 1, "ok": false, "error": "..."}

One process owns one simulator session. The DSH plugin keeps it alive across
tool calls so the physics state survives between them.
"""

from __future__ import annotations

import base64
import io
import json
import os
import sys
import time
import traceback

import numpy as np

# RoboCasa/robosuite print banners to stdout at import time, which would corrupt
# the JSON-RPC channel. Keep a private handle on the real stdout for protocol
# frames, then point fd 1 at stderr so every library print becomes diagnostics.

PREVIEW_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "preview.png")


_PROTO_FD = os.dup(1)
os.dup2(2, 1)
PROTO = os.fdopen(_PROTO_FD, "w", buffering=1, encoding="utf-8")


def log(msg: str) -> None:
    print(f"[worker] {msg}", file=sys.stderr, flush=True)


def reply(payload: dict) -> None:
    PROTO.write(json.dumps(payload) + "\n")
    PROTO.flush()


class Sim:
    def __init__(self) -> None:
        self.env = None
        self.task = None
        self.seed = None
        self.camera = "robot0_agentview_left"
        self.width = 320
        self.height = 320
        self.shots_dir = "/tmp/robocasa-shots"
        self.steps = 0
        self.obs = None
        self.shot_index = 0
        self._snapshot = None
        self.torso_cmd = 0.0
        self.preview_mode = "off"
        self.stream_on = False
        self.preview_size = (320, 240)
        self.preview_params = dict(Sim.FREE_DEFAULTS)
        self.preview_every = 2
        self._preview_tick = 0
        self._detector = None
        self._device = "cpu"
        self._half = False
        self.detector_id = "IDEA-Research/grounding-dino-tiny"
        self.detector_threshold = 0.15

    # ---------------------------------------------------------------- helpers
    def _require_env(self):
        if self.env is None:
            raise RuntimeError("no simulation open: call robocasa_open first")
        return self.env

    def _eef_pos(self):
        if self.obs is None:
            return None
        try:
            return [float(x) for x in self.obs["robot0_eef_pos"]]
        except Exception:
            return None

    def _eef_quat(self):
        if self.obs is None:
            return None
        try:
            return [float(x) for x in self.obs["robot0_eef_quat"]]
        except Exception:
            return None

    def _success(self) -> bool:
        env = self._require_env()
        for attr in ("_check_success", "is_success"):
            fn = getattr(env, attr, None)
            if callable(fn):
                try:
                    val = fn()
                    if isinstance(val, dict):
                        return bool(val.get("task", all(val.values())))
                    return bool(val)
                except Exception:
                    continue
        return False

    def _base_action(self) -> np.ndarray:
        env = self._require_env()
        return np.zeros(env.action_dim, dtype=np.float64)

    def _capture_snapshot(self) -> None:
        self._snapshot = np.array(self.env.sim.get_state().flatten(), copy=True)

    def _write_preview(self) -> None:
        """Publish a small frame for the external viewer window to display."""
        try:
            width, height = self.preview_size
            if self.preview_mode == "third_person":
                png = self._encode_png(self._render_free(self.preview_params, width, height))
            else:
                png = self._render(width=width, height=height)
            tmp = PREVIEW_PATH + ".tmp"
            with open(tmp, "wb") as fh:
                fh.write(png)
            os.replace(tmp, PREVIEW_PATH)
        except Exception as exc:
            log(f"preview write failed: {type(exc).__name__}: {exc}")

    def _after_step(self) -> None:
        if self.preview_mode == "off" and not self.stream_on:
            return
        self._preview_tick += 1
        if self._preview_tick % self.preview_every == 0:
            self._write_preview()

    # Default third-person framing: look at the robot from front-left, pulled back.
    FREE_DEFAULTS = {"dist": 2.8, "az": 200.0, "el": -32.0}

    def _normalize_free(self, params) -> dict:
        """Accept tool-style keys (distance/azimuth/elevation/lookat) and map them."""
        out = dict(self.FREE_DEFAULTS)
        if not params:
            return out
        lookat = params.get("lookat")
        if lookat is not None and len(lookat) == 3:
            out["lx"], out["ly"], out["lz"] = (float(v) for v in lookat)
        for src, dst in (("distance", "dist"), ("azimuth", "az"), ("elevation", "el")):
            if params.get(src) is not None:
                out[dst] = float(params[src])
        return out

    def _free_lookat(self, params: dict) -> np.ndarray:
        if all(k in params for k in ("lx", "ly", "lz")):
            return np.array([params["lx"], params["ly"], params["lz"]], dtype=np.float64)
        eef = self._eef_pos()
        return np.asarray(eef if eef else [0.0, 0.0, 1.0], dtype=np.float64)

    def _render_free(self, params: dict, width: int, height: int) -> np.ndarray:
        env = self._require_env()
        ctx = env.sim._render_context_offscreen
        if ctx is None:
            raise RuntimeError("no offscreen render context; free camera needs offscreen rendering")
        import mujoco  # noqa: PLC0415

        # render() only switches cam.type when a camera_id is passed, so a None
        # camera keeps the previous fixed camera. Force the free camera here.
        ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        ctx.cam.fixedcamid = -1
        ctx.cam.lookat[:] = self._free_lookat(params)
        ctx.cam.distance = float(params["dist"])
        ctx.cam.azimuth = float(params["az"])
        ctx.cam.elevation = float(params["el"])
        img = env.sim.render(width=width, height=height, camera_name=None, depth=False)
        return np.asarray(img)[::-1]

    @staticmethod
    def _encode_png(img) -> bytes:
        from PIL import Image  # noqa: PLC0415

        if img.dtype != np.uint8:
            img = img.astype(np.uint8)
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG")
        return buf.getvalue()

    def _render(self, camera=None, width=None, height=None, free=None) -> bytes:
        env = self._require_env()
        camera = camera or self.camera
        width = int(width or self.width)
        height = int(height or self.height)
        if free is not None:
            img = self._render_free(free, width, height)
        else:
            img = env.sim.render(width=width, height=height, camera_name=camera, depth=False)
            img = np.asarray(img)[::-1]
        return self._encode_png(img)

    def _save_shot(self, png: bytes) -> str:
        os.makedirs(self.shots_dir, exist_ok=True)
        self.shot_index += 1
        path = os.path.join(self.shots_dir, f"{self.task or 'scene'}-{self.seed}-{self.shot_index:04d}.png")
        with open(path, "wb") as fh:
            fh.write(png)
        return path

    # ----------------------------------------------------------------- methods
    def open(self, p):
        from robocasa.utils.env_utils import create_env

        if self.env is not None:
            self.close({})

        self.task = p.get("task", "PickPlaceCounterToCabinet")
        self.seed = int(p.get("seed", 0))
        self.camera = p.get("camera", "robot0_agentview_left")
        self.width = int(p.get("width", 320))
        self.height = int(p.get("height", 320))
        self.shots_dir = p.get("shots_dir") or self.shots_dir
        layout = p.get("layout")
        style = p.get("style")
        ls = [[int(layout), int(style)]] if layout is not None and style is not None else None

        t0 = time.time()
        self.preview_mode = str(p.get("preview") or "off")
        if self.preview_mode not in ("off", "shoulder", "third_person"):
            self.preview_mode = "off"
        self.env = create_env(
            self.task,
            camera_names=[self.camera],
            camera_widths=self.width,
            camera_heights=self.height,
            seed=self.seed,
            render_onscreen=False,
            layout_and_style_ids=ls,
        )
        self.obs = self.env.reset()
        self.steps = 0
        self.shot_index = 0
        self.torso_cmd = 0.0
        self._capture_snapshot()
        self._write_preview()
        log(f"opened {self.task} seed={self.seed} preview={self.preview_mode} in {time.time() - t0:.1f}s")
        return {
            "task": self.task,
            "seed": self.seed,
            "camera": self.camera,
            "action_dim": int(self.env.action_dim),
            "eef_pos": self._eef_pos(),
            "versions": _versions(),
            "preview": self.preview_mode,
            "open_seconds": round(time.time() - t0, 2),
        }

    def reset(self, p):
        env = self._require_env()
        self.obs = env.reset()
        self.steps = 0
        self.torso_cmd = 0.0
        self._capture_snapshot()
        if self.preview_mode != "off":
            self._write_preview()
        return {"eef_pos": self._eef_pos(), "eef_quat": self._eef_quat(), "success": self._success()}

    def restart(self, p):
        """Restore the exact state captured at open or at the last reset."""
        env = self._require_env()
        if self._snapshot is None:
            raise RuntimeError("no snapshot available: call robocasa_open first")
        env.sim.set_state_from_flattened(self._snapshot)
        env.sim.forward()
        try:
            env.robots[0].composite_controller.reset()
        except Exception:
            pass
        for robot in getattr(env, "robots", []):
            try:
                robot.gripper.current_action = np.zeros(robot.gripper.dof)
            except Exception:
                pass
        self.obs = env._get_observations(force_update=True)
        self.steps = 0
        return {
            "eef_pos": self._eef_pos(),
            "eef_quat": self._eef_quat(),
            "success": self._success(),
            "steps": 0,
        }

    def _load_detector(self):
        if self._detector is not None:
            return self._detector
        # The host process may carry a SOCKS proxy that huggingface_hub cannot use,
        # and a NO_PROXY entry such as [::1] that httpx rejects. Normalise both
        # before transformers is imported, and prefer the local weights cache.
        for key in ("ALL_PROXY", "all_proxy"):
            os.environ.pop(key, None)
        os.environ["NO_PROXY"] = "localhost,127.0.0.1"
        os.environ["no_proxy"] = "localhost,127.0.0.1"
        os.environ.setdefault("HF_HUB_OFFLINE", "1")
        import torch  # noqa: PLC0415
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection  # noqa: PLC0415

        device = "cuda" if torch.cuda.is_available() else "cpu"
        log(f"loading detector {self.detector_id} on {device}")
        proc = AutoProcessor.from_pretrained(self.detector_id)
        model = AutoModelForZeroShotObjectDetection.from_pretrained(self.detector_id).eval().to(device)
        self._device = device
        self._half = device == "cuda"
        if self._half:
            model = model.half()
        self._detector = (proc, model)
        return self._detector

    def _vision_locate(self, query):
        """Open-vocabulary detection, one candidate per box, projected to world space."""
        env = self._require_env()
        from PIL import Image  # noqa: PLC0415
        from robosuite.utils.camera_utils import (  # noqa: PLC0415
            get_camera_intrinsic_matrix, get_camera_extrinsic_matrix, get_real_depth_map,
        )
        import torch  # noqa: PLC0415

        cam, H, W = self.camera, self.height, self.width
        rgb = np.asarray(env.sim.render(width=W, height=H, camera_name=cam, depth=False))[::-1].astype(np.uint8)
        _, dep = env.sim.render(width=W, height=H, camera_name=cam, depth=True)
        depth = get_real_depth_map(env.sim, np.asarray(dep)[::-1].copy())
        K = get_camera_intrinsic_matrix(env.sim, cam, H, W)
        E = get_camera_extrinsic_matrix(env.sim, cam)
        K_inv = np.linalg.inv(K)

        proc, model = self._load_detector()
        pil = Image.fromarray(rgb)
        inputs = proc(images=pil, text=[[query]], return_tensors="pt").to(self._device)
        if self._half:
            inputs = {k: (v.half() if getattr(v, "dtype", None) == torch.float32 else v) for k, v in inputs.items()}
        with torch.no_grad():
            out = model(**inputs)
        if self._half:
            for field in ("logits", "pred_boxes"):
                value = getattr(out, field, None)
                if value is not None and hasattr(value, "float"):
                    setattr(out, field, value.float())
        res = proc.post_process_grounded_object_detection(
            out, inputs["input_ids"], threshold=self.detector_threshold, text_threshold=0.15,
            target_sizes=[pil.size[::-1]],
        )[0]
        boxes = res["boxes"].detach().cpu().numpy().tolist()
        scores = res["scores"].detach().cpu().numpy().tolist()
        labels = res.get("text_labels", res.get("labels", [query] * len(boxes)))

        candidates = []
        for box, score, label in zip(boxes, scores, labels):
            u = float(np.clip((box[0] + box[2]) / 2.0, 0, W - 1))
            v = float(np.clip((box[1] + box[3]) / 2.0, 0, H - 1))
            z = float(depth[int(v), int(u)])
            cam_pt = z * (K_inv @ np.array([u, v, 1.0]))
            world = (E @ np.append(cam_pt, 1.0))[:3]
            # Box size in pixels times depth over focal length estimates the
            # visible extent in metres, so the grasp point can sit at the centre
            # instead of on the visible surface.
            height_m = max(0.0, float(box[3] - box[1])) * z / float(K[1, 1])
            width_m = max(0.0, float(box[2] - box[0])) * z / float(K[0, 0])
            # A top-down view projects the top face, so box height is not object
            # height. Aim a fixed depth below the visible surface instead: deep
            # enough for the fingers to straddle, shallow enough not to sink.
            grasp = [float(world[0]), float(world[1]), float(world[2] - 0.02)]

            # Orientation: back-project the pixels inside the box, keep the nearer
            # half (the object rather than the counter behind it) and take the
            # principal axis of the resulting points in the world XY plane.
            yaw_deg = None
            u1, v1 = int(max(0, box[0])), int(max(0, box[1]))
            u2, v2 = int(min(W - 1, box[2])), int(min(H - 1, box[3]))
            if u2 - u1 >= 4 and v2 - v1 >= 4:
                patch = depth[v1:v2, u1:u2].reshape(-1)
                vv, uu = np.mgrid[v1:v2, u1:u2]
                ok = patch > 0.05
                if ok.sum() > 12:
                    cut = np.percentile(patch[ok], float(os.environ.get("VISION_DEPTH_PERCENTILE", "60")))
                    sel = ok & (patch <= cut)
                    zz = patch[sel]
                    if zz.size > 12:
                        pxy = np.stack([uu.reshape(-1)[sel], vv.reshape(-1)[sel], np.ones_like(zz)]).astype(np.float64)
                        pts_cam = (K_inv @ pxy) * zz
                        pts_w = (E @ np.vstack([pts_cam, np.ones((1, zz.size))]))[:3].T
                        # Fit the visible surface plane, then take the principal
                        # direction inside that plane. Working in world XY directly
                        # would be skewed by the oblique camera view.
                        centre3 = pts_w.mean(0)
                        centred = pts_w - centre3
                        _, _, vt = np.linalg.svd(centred, full_matrices=False)
                        basis1, basis2 = vt[0], vt[1]
                        proj = np.stack([centred @ basis1, centred @ basis2], axis=1)
                        _, vecs2 = np.linalg.eigh(proj.T @ proj)
                        axis3 = vecs2[0, -1] * basis1 + vecs2[1, -1] * basis2
                        if axis3[0] < 0:
                            axis3 = -axis3
                        yaw_deg = round(float(np.degrees(np.arctan2(axis3[1], axis3[0]))), 1)
            candidates.append({
                "label": str(label),
                "score": round(float(score), 3),
                "box": [round(float(b), 1) for b in box],
                "pixel": [round(u, 1), round(v, 1)],
                "depth": round(z, 3),
                "world": [round(float(x), 4) for x in world],
                "visible_extent": [round(width_m, 4), round(height_m, 4)],
                "grasp_world": [round(float(x), 4) for x in grasp],
                "yaw_deg": yaw_deg,
                "grasp_yaw_deg": yaw_deg,
            })
        candidates.sort(key=lambda c: c["score"], reverse=True)
        return {
            "query": query,
            "found": bool(candidates),
            "count": len(candidates),
            "source": "vision",
            "model": self.detector_id,
            "candidates": candidates,
        }

    def locate(self, p):
        """Structured lookup of task objects and fixtures.

        The current backend reads simulator ground truth and labels every match
        with source="ground_truth"; a vision backend can replace it behind the
        same response shape.
        """
        env = self._require_env()
        if str(p.get("backend", "ground_truth")) == "vision":
            return self._vision_locate(str(p.get("query", "")).strip())
        query = str(p.get("query", "")).strip().lower()
        matches = []

        objects = getattr(env, "objects", None) or {}
        obj_items = list(objects.items()) if isinstance(objects, dict) else []
        for name, obj in obj_items:
            label = str(name)
            if query not in ("", "all") and query not in label.lower():
                continue
            try:
                pos = np.asarray(env.sim.data.body_xpos[env.obj_body_id[name]], dtype=np.float64)
                pts = np.asarray(obj.get_bbox_points(), dtype=np.float64)
                size = pts.max(0) - pts.min(0)
            except Exception:
                continue
            entry = {
                "kind": "object",
                "label": label,
                "source": "ground_truth",
                "world": [round(float(v), 4) for v in pos],
                "size": [round(float(v), 4) for v in size],
                "top_z": round(float(pos[2] + size[2] / 2.0), 4),
                "pre_grasp": [round(float(v), 4) for v in (pos + np.array([0.0, 0.0, size[2] / 2.0 + 0.10]))],
                "grasp": [round(float(v), 4) for v in pos],
            }
            try:
                frame = _horizontal_frame(env, name, obj)
                entry["yaw_deg"] = frame["yaw_deg"]
                entry["grasp_yaw_deg"] = frame["yaw_deg"]
                entry["world_extent"] = [frame["extent_long"], frame["extent_short"], frame["height"]]
                entry["top_z"] = frame["top_z"]
                entry["pre_grasp"] = [frame["centre"][0], frame["centre"][1], round(frame["top_z"] + 0.10, 4)]
                entry["grasp"] = [frame["centre"][0], frame["centre"][1],
                                  round((frame["top_z"] + frame["top_z"] - frame["height"]) / 2.0, 4)]
            except Exception:
                pass
            matches.append(entry)

        fixtures = getattr(env, "fixtures", None) or {}
        fx_items = list(fixtures.items()) if isinstance(fixtures, dict) else []
        for key, fixture in fx_items:
            inst = fixture if hasattr(fixture, "pos") else (fixture[0] if isinstance(fixture, (list, tuple)) and fixture else None)
            if inst is None:
                continue
            name = str(getattr(inst, "name", key))
            if query not in ("", "all") and query not in name.lower() and query not in str(key).lower():
                continue
            entry = {"kind": "fixture", "label": name, "source": "ground_truth"}
            try:
                entry["world"] = [round(float(v), 4) for v in np.asarray(inst.pos, dtype=np.float64).reshape(-1)[:3]]
                entry["size"] = [round(float(v), 4) for v in np.asarray(inst.size, dtype=np.float64).reshape(-1)[:3]]
            except Exception:
                pass
            try:
                regions = inst.get_int_sites(relative=False)
                entry["interior"] = [
                    {"region": rname, "center": [round(float(v), 4) for v in
                     (np.asarray(p0) + np.asarray(px) + np.asarray(py) + np.asarray(pz)) / 4.0]}
                    for rname, (p0, px, py, pz) in regions.items()
                ]
            except Exception:
                pass
            matches.append(entry)

        return {"query": query, "found": bool(matches), "count": len(matches), "matches": matches}

    def set_preview(self, p):
        """Switch the preview window mode and framing at runtime."""
        mode = str(p.get("mode", self.preview_mode))
        if mode not in ("off", "shoulder", "third_person"):
            raise ValueError(f"unknown preview mode: {mode!r}")
        params = dict(self.preview_params)
        if p.get("lookat") is not None:
            lookat = [float(v) for v in p["lookat"]]
            if len(lookat) != 3:
                raise ValueError("lookat needs three numbers")
            params["lx"], params["ly"], params["lz"] = lookat
        for key in ("distance", "azimuth", "elevation"):
            if p.get(key) is not None:
                params[{"distance": "dist", "azimuth": "az", "elevation": "el"}[key]] = float(p[key])
        self.preview_params = params
        self.preview_mode = mode
        if mode != "off":
            self._write_preview()
        return {"mode": self.preview_mode, "params": self.preview_params}

    def look(self, p):
        free = p.get("free")
        png = self._render(
            p.get("camera"), p.get("width"), p.get("height"),
            free=self._normalize_free(free) if free is not None else None,
        )
        path = self._save_shot(png)
        return {
            "path": path,
            "camera": "free" if free is not None else (p.get("camera") or self.camera),
            "image_png_base64": base64.b64encode(png).decode("ascii"),
            "eef_pos": self._eef_pos(),
            "steps": self.steps,
            "success": self._success(),
        }

    def act(self, p):
        env = self._require_env()
        steps = max(1, min(int(p.get("steps", 1)), 200))
        if p.get("torso") is not None:
            self.torso_cmd = float(p["torso"])
        action = _fill_action(env, {**p, "torso": self.torso_cmd})
        for _ in range(steps):
            self.obs, _, _, _ = env.step(action)
            self.steps += 1
            self._after_step()
        return {
            "steps": self.steps,
            "eef_pos": self._eef_pos(),
            "eef_quat": self._eef_quat(),
            "success": self._success(),
        }

    def move(self, p):
        """Proportional move of the end effector toward a world-space target."""
        env = self._require_env()
        target = np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=np.float64)
        gripper = p.get("gripper")
        max_steps = max(1, min(int(p.get("max_steps", 80)), 400))
        tol = float(p.get("tol", 0.015))
        scale = float(p.get("scale", 0.05))
        action = _fill_action(
            env,
            {"torso": self.torso_cmd, "gripper": gripper} if gripper is not None else {"torso": self.torso_cmd},
        )

        gain = float(p.get("gain", 0.6))
        gain_rot = float(p.get("gain_rot", 0.8))
        rot_tol = float(p.get("rot_tol", 0.03))

        # Absolute orientation target: full quaternion, or a yaw about world z that
        # keeps the current pitch/roll.
        target_rot = None
        if p.get("quat") or p.get("yaw_deg") is not None:
            R_now = _quat_to_mat(self.obs["robot0_eef_quat"])
            if p.get("quat"):
                target_rot = _quat_to_mat(p["quat"])
            else:
                delta = np.radians(float(p["yaw_deg"])) - _yaw_of_mat(R_now)
                # A parallel-jaw gripper is symmetric under 180 degrees, so take the
                # equivalent yaw that needs the smaller wrist rotation.
                delta = (delta + np.pi / 2) % np.pi - np.pi / 2
                c, s = np.cos(delta), np.sin(delta)
                target_rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ R_now

        start = self._eef_pos()
        start_quat = self._eef_quat()
        dist = float("inf")
        rot_err = 0.0
        used = 0
        for _ in range(max_steps):
            cur = np.array(self.obs["robot0_eef_pos"], dtype=np.float64)
            error = target - cur
            dist = float(np.linalg.norm(error))
            if target_rot is not None:
                R_now = _quat_to_mat(self.obs["robot0_eef_quat"])
                err_w = _rotvec_from_mat(target_rot @ R_now.T)
                rot_err = float(np.linalg.norm(err_w))
                action[3:6] = np.clip(_world_to_base(env, err_w) / 0.5 * gain_rot, -1.0, 1.0)
            if dist <= tol and rot_err <= rot_tol:
                break
            action[0:3] = np.clip(_world_to_base(self.env, error) / scale * gain, -1.0, 1.0)
            self.obs, _, _, _ = env.step(action)
            self.steps += 1
            used += 1
            self._after_step()
        end = self._eef_pos()
        # Recompute from the final pose: the loop may exit before ever updating
        # these, which previously leaked Infinity into the JSON reply.
        end_arr = np.asarray(end, dtype=np.float64)
        dist = float(np.linalg.norm(target - end_arr))
        if target_rot is not None:
            R_end = _quat_to_mat(self.obs["robot0_eef_quat"])
            rot_err = float(np.linalg.norm(_rotvec_from_mat(target_rot @ R_end.T)))
        return {
            "start": start,
            "target": [float(v) for v in target],
            "end": end,
            "residual_m": _num(dist),
            "rot_residual_rad": _num(rot_err),
            "start_quat": start_quat,
            "end_quat": self._eef_quat(),
            "steps_used": used,
            "total_steps": self.steps,
            "success": self._success(),
        }

    def rotate(self, p):
        """Closed-loop rotation by a relative amount, in degrees, about world axes."""
        env = self._require_env()
        max_steps = max(1, min(int(p.get("max_steps", 60)), 400))
        tol = float(p.get("tol", 0.03))
        gain = float(p.get("gain_rot", 0.8))
        delta = np.radians([
            float(p.get("droll", 0.0)),
            float(p.get("dpitch", 0.0)),
            float(p.get("dyaw", 0.0)),
        ])
        start_quat = self._eef_quat()
        if float(np.linalg.norm(delta)) < 1e-9:
            return {
                "start_quat": start_quat,
                "end_quat": start_quat,
                "residual_rad": 0.0,
                "steps_used": 0,
                "total_steps": self.steps,
            }

        R_target = _mat_from_rotvec(delta) @ _quat_to_mat(start_quat)
        action = _fill_action(
            env,
            {"torso": self.torso_cmd, "gripper": p.get("gripper")} if p.get("gripper")
            else {"torso": self.torso_cmd},
        )
        err = float("inf")
        used = 0
        for _ in range(max_steps):
            R_now = _quat_to_mat(self.obs["robot0_eef_quat"])
            err_w = _rotvec_from_mat(R_target @ R_now.T)
            err = float(np.linalg.norm(err_w))
            if err <= tol:
                break
            action[3:6] = np.clip(_world_to_base(env, err_w) / 0.5 * gain, -1.0, 1.0)
            self.obs, _, _, _ = env.step(action)
            self.steps += 1
            used += 1
            self._after_step()
        end_quat = self._eef_quat()
        err = float(np.linalg.norm(_rotvec_from_mat(R_target @ _quat_to_mat(end_quat).T)))
        return {
            "start_quat": start_quat,
            "end_quat": end_quat,
            "residual_rad": _num(err),
            "steps_used": used,
            "total_steps": self.steps,
        }

    def grasp(self, p):
        """Multi-hypothesis grasp: try several yaw offsets, keep the one that lifts.

        Vision-based orientation is only accurate to roughly 30 degrees, so the
        approach is to search a small set of candidates and verify each attempt by
        watching whether the verified object starts to follow the gripper.
        """
        env = self._require_env()
        target = np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=np.float64)
        base_yaw = float(p.get("yaw_deg", 0.0))
        offsets = p.get("offsets") or [0.0, -30.0, 30.0]
        verify = str(p.get("verify", "obj"))
        lift = float(p.get("lift", 0.10))
        body = env.obj_body_id.get(verify)
        if body is None:
            raise RuntimeError(f"unknown verify object: {verify!r}")

        start_z = float(env.sim.data.body_xpos[body][2])
        z_offset = float(target[2]) - start_z
        refresh = bool(p.get("refresh", True))
        retry_stalls = bool(p.get("retry_stalls", True))

        attempts = []
        for offset in offsets:
            yaw = base_yaw + float(offset)

            # Re-read the target from the simulator before each attempt: a failed
            # grasp usually pushes the object, which would make the previous
            # coordinates stale.
            if refresh:
                now = np.asarray(env.sim.data.body_xpos[body], dtype=np.float64)
                target[0], target[1] = float(now[0]), float(now[1])
                target[2] = float(now[2]) + z_offset

            z0 = float(env.sim.data.body_xpos[body][2])
            # Approach from high up first: a straight reach from the home pose
            # often jams against the counter edge and never descends.
            hover = max(1.30, float(target[2]) + 0.25)
            self.move({"x": float(target[0]), "y": float(target[1]), "z": hover,
                       "yaw_deg": yaw, "gripper": "open", "max_steps": 70})
            pre = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2] + 0.10),
                             "yaw_deg": yaw, "gripper": "open", "max_steps": 90, "tol": 0.015})
            down = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2]),
                              "yaw_deg": yaw, "gripper": "open", "max_steps": 60, "tol": 0.012})

            # A straight descent sometimes jams against a fixture; approach from
            # the side instead of pushing harder.
            if retry_stalls and (down.get("residual_m") or 0) > 0.04:  # noqa: E501
                self.move({"x": float(target[0] - 0.09), "y": float(target[1]), "z": float(target[2] + 0.14),
                           "yaw_deg": yaw, "gripper": "open", "max_steps": 60})
                down = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2]),
                                  "yaw_deg": yaw, "gripper": "open", "max_steps": 60, "tol": 0.012})
            self.act({"gripper": "close", "steps": 15})
            up = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2] + lift),
                            "yaw_deg": yaw, "gripper": "close", "max_steps": 50})
            z1 = float(env.sim.data.body_xpos[body][2])
            delta = _num(z1 - z0)
            attempts.append({
                "yaw_deg": round(yaw, 1),
                "target_used": [round(float(v), 4) for v in target],
                "offset": float(offset),
                "descend_residual_m": _num(down.get("residual_m") or 0.0),
                "lift_residual_m": _num(up.get("residual_m") or 0.0),
                "object_z_delta": delta,
                "success": bool(delta > 0.02),
            })
            if delta > 0.02:
                return {"success": True, "attempts": attempts, "grasp": [float(v) for v in target],
                        "yaw_deg": round(yaw, 1), "object": verify}
            self.act({"gripper": "open", "steps": 10})
            self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2] + 0.16),
                       "gripper": "open", "max_steps": 50})
        return {"success": False, "attempts": attempts, "grasp": [float(v) for v in target], "object": verify}

    def frame(self, p):
        """One small frame for the UI stream, never written to disk."""
        self._require_env()
        free = self.preview_params if self.preview_mode == "third_person" else None
        png = self._render(None, int(p.get("width", 320)), int(p.get("height", 240)), free=free)
        return {
            "image_png_base64": base64.b64encode(png).decode("ascii"),
            "mode": self.preview_mode,
        }

    def set_streaming(self, p):
        """Enable frame writing for web clients without touching the preview window."""
        self.stream_on = bool(p.get("on"))
        if self.stream_on:
            self._write_preview()
        return {"streaming": self.stream_on}

    def jog(self, p):
        """Move the end effector by a relative world delta (metres)."""
        self._require_env()
        cur = np.asarray(self.obs["robot0_eef_pos"], dtype=np.float64)
        delta = np.array([float(p.get("dx", 0.0)), float(p.get("dy", 0.0)), float(p.get("dz", 0.0))])
        return self.move({
            "x": float(cur[0] + delta[0]),
            "y": float(cur[1] + delta[1]),
            "z": float(cur[2] + delta[2]),
            "gripper": p.get("gripper"),
            "max_steps": int(p.get("max_steps", 40)),
        })

    def grasp_object(self, p):
        """Locate a simulator object by name and grasp it with the usual search."""
        name = str(p.get("name", "obj"))
        info = self.locate({"query": name})
        match = next(
            (m for m in info["matches"] if m.get("kind") == "object" and m.get("label") == name),
            None,
        )
        if match is None:
            raise RuntimeError(f"object not found: {name}")
        offsets = p.get("offsets") or [0.0, -30.0, 30.0]
        return self.grasp({
            "x": match["grasp"][0],
            "y": match["grasp"][1],
            "z": match["grasp"][2],
            "yaw_deg": match.get("grasp_yaw_deg") or 0.0,
            "offsets": offsets,
            "verify": name,
        })

    def configure(self, p):
        """Apply runtime settings; unknown keys are ignored."""
        changed = {}
        if p.get("detectorThreshold") is not None:
            self.detector_threshold = float(p["detectorThreshold"])
            changed["detectorThreshold"] = self.detector_threshold
        if p.get("previewEvery") is not None:
            self.preview_every = max(1, int(p["previewEvery"]))
            changed["previewEvery"] = self.preview_every
        if p.get("camera"):
            self.camera = str(p["camera"])
            changed["camera"] = self.camera
        if p.get("streamWidth") and p.get("streamHeight"):
            self.preview_size = (int(p["streamWidth"]), int(p["streamHeight"]))
            changed["preview_size"] = list(self.preview_size)
        if p.get("detectorModel"):
            self.detector_id = str(p["detectorModel"])
            self._detector = None
            changed["detectorModel"] = self.detector_id
        return {"applied": changed}

    def settings(self, p):
        """Current runtime settings, for the UI to render."""
        return {
            "camera": self.camera,
            "detectorThreshold": self.detector_threshold,
            "previewEvery": self.preview_every,
            "detectorModel": self.detector_id,
        }

    def state(self, p):
        """Compact live state for the UI; safe to call when nothing is open."""
        if self.env is None:
            return {"open": False, "preview": self.preview_mode}
        value = self.status(p)
        value["open"] = True
        value["preview"] = self.preview_mode
        return value

    def status(self, p):
        env = self._require_env()
        return {
            "task": self.task,
            "seed": self.seed,
            "camera": self.camera,
            "steps": self.steps,
            "eef_pos": self._eef_pos(),
            "eef_quat": self._eef_quat(),
            "success": self._success(),
            "objects": _object_positions(env),
            "dimensions": _object_dimensions(env),
        }

    def close(self, p):
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass
            self.env = None
        self.obs = None
        return {"closed": True}


def _base_rotation(env) -> np.ndarray:
    """Rotation of the OSC reference frame (world = R @ base), or identity."""
    try:
        cc = env.robots[0].composite_controller
        ori = cc.part_controllers["right"].origin_ori
        if ori is not None:
            return np.array(ori, dtype=np.float64).reshape(3, 3)
    except Exception:
        pass
    try:
        bid = env.sim.model.body_name2id("robot0_base")
        return np.array(env.sim.data.body_xmat[bid], dtype=np.float64).reshape(3, 3)
    except Exception:
        return np.eye(3)


def _world_to_base(env, delta) -> np.ndarray:
    """OSC_POSE takes deltas in the robot base frame; callers think in world axes."""
    rot = _base_rotation(env)
    return rot.T @ np.asarray(delta, dtype=np.float64)


def _num(value, digits=4) -> float:
    """Round for transport and normalise -0.0, which JSON loses on round-trip."""
    out = round(float(value), digits)
    return 0.0 if out == 0 else out


def _quat_to_mat(q) -> np.ndarray:
    """(x, y, z, w) quaternion to a 3x3 rotation matrix."""
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _rotvec_from_mat(R) -> np.ndarray:
    """Rotation matrix to an axis-angle vector (log map)."""
    cos = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cos))
    if angle < 1e-6:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(angle))
    return axis * angle


def _mat_from_rotvec(w) -> np.ndarray:
    """Axis-angle vector to a rotation matrix (exp map)."""
    w = np.asarray(w, dtype=np.float64)
    theta = float(np.linalg.norm(w))
    if theta < 1e-9:
        return np.eye(3)
    k = w / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def _yaw_of_mat(R) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def _split(env, part: str):
    """Return (start, end) of a composite-controller action slice."""
    try:
        cc = env.robots[0].composite_controller
        return cc._action_split_indexes[part]
    except Exception:
        return None


def _fill_action(env, p: dict) -> np.ndarray:
    """Build a 12-dim composite action: arm delta, gripper, mobile base, torso."""
    action = np.zeros(env.action_dim, dtype=np.float64)
    action[0] = float(p.get("dx", 0.0))
    action[1] = float(p.get("dy", 0.0))
    action[2] = float(p.get("dz", 0.0))
    gripper = p.get("gripper")
    if gripper is not None:
        slot = _split(env, "right_gripper")
        idx = slot[0] if slot else len(action) - 1
        action[idx] = _gripper_value(gripper)
    torso = _split(env, "torso")
    if torso and p.get("torso") is not None:
        action[torso[0]] = float(p["torso"])
    base = _split(env, "base")
    if base and any(k in p for k in ("forward", "side", "yaw")):
        action[base[0]] = float(p.get("forward", 0.0))
        action[base[0] + 1] = float(p.get("side", 0.0))
        action[base[0] + 2] = float(p.get("yaw", 0.0))
    return action


def _gripper_value(gripper) -> float:
    """robosuite convention: +1 closes, -1 opens."""
    g = str(gripper).lower()
    if g in ("close", "closed", "grasp", "1"):
        return 1.0
    if g in ("open", "opened", "release", "-1"):
        return -1.0
    if g in ("hold", "none", "0"):
        return 0.0
    raise ValueError(f"unknown gripper value: {gripper!r} (use open/close/hold)")


def _versions() -> dict:
    """Runtime versions recorded in each run manifest."""
    import importlib.metadata as md

    out = {"python": f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"}
    for mod in ("numpy", "mujoco", "robosuite", "robocasa"):
        try:
            out[mod] = md.version(mod)
        except Exception:
            out[mod] = "unknown"
    return out


def _horizontal_frame(env, name, obj):
    """World-space horizontal principal axis and extents of an object's bbox."""
    from robosuite.utils.transform_utils import convert_quat  # noqa: PLC0415

    body = env.obj_body_id[name]
    pos = np.asarray(env.sim.data.body_xpos[body], dtype=np.float64)
    quat = convert_quat(env.sim.data.body_xquat[body], to="xyzw")
    pts = np.asarray(obj.get_bbox_points(trans=pos, rot=quat), dtype=np.float64)
    centre = pts.mean(0)
    xy = pts[:, :2] - centre[:2]
    cov = xy.T @ xy
    _, vecs = np.linalg.eigh(cov)
    axis = vecs[:, -1]
    if axis[0] < 0:
        axis = -axis
    yaw = float(np.degrees(np.arctan2(axis[1], axis[0])))
    proj_along = xy @ axis
    perp = np.array([-axis[1], axis[0]])
    proj_perp = xy @ perp
    extent_long = float(proj_along.max() - proj_along.min())
    extent_short = float(proj_perp.max() - proj_perp.min())
    height = float(pts[:, 2].max() - pts[:, 2].min())
    return {
        "yaw_deg": round(yaw, 1),
        "extent_long": round(extent_long, 4),
        "extent_short": round(extent_short, 4),
        "height": round(height, 4),
        "top_z": round(float(pts[:, 2].max()), 4),
        "centre": [round(float(v), 4) for v in centre],
    }


def _object_dimensions(env) -> dict:
    """Bounding-box extent of each task object, in metres."""
    out = {}
    objs = getattr(env, "objects", None) or {}
    items = list(objs.items()) if isinstance(objs, dict) else [(getattr(o, "name", str(i)), o) for i, o in enumerate(objs)]
    for name, obj in items:
        try:
            pts = np.asarray(obj.get_bbox_points(), dtype=np.float64)
            out[str(name)] = [round(float(v), 4) for v in (pts.max(0) - pts.min(0))]
        except Exception:
            continue
    return out


def _object_positions(env) -> dict:
    """Best-effort mapping of task object names to world positions."""
    out = {}
    objs = getattr(env, "objects", None) or {}
    items = list(objs.items()) if isinstance(objs, dict) else [(getattr(o, "name", str(i)), o) for i, o in enumerate(objs)]
    sim = env.sim
    for name, obj in items:
        try:
            root = getattr(obj, "root_body", None)
            if not root:
                continue
            bid = sim.model.body_name2id(root)
            out[str(name)] = [round(float(v), 4) for v in sim.data.body_xpos[bid][:3]]
        except Exception:
            continue
    return out


METHODS = {
    "open": Sim.open,
    "reset": Sim.reset,
    "restart": Sim.restart,
    "look": Sim.look,
    "set_preview": Sim.set_preview,
    "act": Sim.act,
    "move": Sim.move,
    "locate": Sim.locate,
    "rotate": Sim.rotate,
    "status": Sim.status,
    "frame": Sim.frame,
    "state": Sim.state,
    "set_streaming": Sim.set_streaming,
    "jog": Sim.jog,
    "grasp_object": Sim.grasp_object,
    "configure": Sim.configure,
    "settings": Sim.settings,
    "grasp": Sim.grasp,
    "close": Sim.close,
}


def main() -> None:
    sim = Sim()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception as exc:
            reply({"id": None, "ok": False, "error": f"bad json: {exc}"})
            continue
        rid = req.get("id")
        method = req.get("method")
        params = req.get("params") or {}
        fn = METHODS.get(method)
        if fn is None:
            reply({"id": rid, "ok": False, "error": f"unknown method: {method}"})
            continue
        try:
            result = fn(sim, params)
            reply({"id": rid, "ok": True, "result": result})
        except Exception as exc:
            tb = traceback.format_exc()
            log(tb)
            reply({"id": rid, "ok": False, "error": f"{type(exc).__name__}: {exc}", "traceback": tb[-1500:]})
    sim.close({})


if __name__ == "__main__":
    main()
