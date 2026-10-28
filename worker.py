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

import mujoco
import numpy as np

from planner import (
    COLLISION_TOLERANCE, IK_DAMPING, RRT_EDGE_STEP, RRT_MAX_ITERS, RRT_STEP,
    CollisionChecker, Planner, RobotModel, time_parameterise,
)

# RoboCasa/robosuite print banners to stdout at import time, which would corrupt
# the JSON-RPC channel. Keep a private handle on the real stdout for protocol
# frames, then point fd 1 at stderr so every library print becomes diagnostics.

PREVIEW_PATH = os.environ.get("ROBOCASA_PREVIEW_PATH") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "preview.png"
)


_PROTO_FD = os.dup(1)
os.dup2(2, 1)
PROTO = os.fdopen(_PROTO_FD, "w", buffering=1, encoding="utf-8")


ARM_KP = 400.0


def _joint_controller_config():
    """Composite config whose right arm takes absolute joint positions."""
    from robosuite.controllers import load_composite_controller_config, load_part_controller_config

    cfg = load_composite_controller_config(controller=None, robot="PandaOmron")
    part = load_part_controller_config(default_controller="JOINT_POSITION")
    part.update({"gripper": {"type": "GRIP"}, "input_type": "absolute", "kp": ARM_KP})
    cfg["body_parts"]["right"] = part
    return cfg


def make_env(task, camera, width, height, seed, layout_and_style_ids=None):
    """create_env with the arm switched to absolute joint position control."""
    import robocasa.utils.env_utils as env_utils

    cfg = _joint_controller_config()
    original = env_utils.load_composite_controller_config
    env_utils.load_composite_controller_config = lambda controller=None, robot=None: cfg
    try:
        env = env_utils.create_env(
            task,
            camera_names=[camera],
            camera_widths=width,
            camera_heights=height,
            seed=seed,
            render_onscreen=False,
            layout_and_style_ids=layout_and_style_ids,
        )
    finally:
        env_utils.load_composite_controller_config = original
    return env


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
        self.held_object = None
        # "oracle" reads simulator ground truth; "sensors" only uses what a real
        # robot has: depth, contact and proprioception.
        self.perception = "oracle"
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

    def _sync_controller(self) -> None:
        """Point the arm controller at the live pose after a state restore."""
        if getattr(self, "robot_model", None) is None:
            return
        action = self._action(self._q(), torso=0.0)
        self.env.step(action)

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
        if self.env is not None:
            self.close({})

        self.task = p.get("task", "PickPlaceCounterToCabinet")
        self.seed = int(p.get("seed", 0))
        self.camera = p.get("camera", "robot0_agentview_left")
        self.perception = str(p.get("perception") or "oracle")
        if self.perception not in ("oracle", "sensors"):
            self.perception = "oracle"
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
        self.env = make_env(
            self.task,
            self.camera,
            self.width,
            self.height,
            self.seed,
            ls,
        )
        self.obs = self.env.reset()
        self._bind_robot()
        self.held_object = None
        self._settle_home()
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

    def _bind_robot(self) -> None:
        """Cache action slices and build the planning stack for this env."""
        env = self.env
        cc = env.robots[0].composite_controller
        self.arm_slice = cc._action_split_indexes["right"]
        self.grip_slice = cc._action_split_indexes.get("right_gripper")
        self.base_slice = cc._action_split_indexes.get("base")
        self.torso_slice = cc._action_split_indexes.get("torso")
        self.home_q = np.asarray(env.robots[0].init_qpos, dtype=np.float64).copy()
        self.robot_model = RobotModel(env)
        try:
            self.robot_model.calibrate(_quat_to_mat(env._get_observations(force_update=True)["robot0_eef_quat"]))
        except Exception:
            pass
        self.checker = CollisionChecker(self.robot_model)
        self.planner = Planner(self.robot_model, self.checker, seed=int(time.time()) % 9973)

    def _q(self) -> np.ndarray:
        return self.robot_model.get_q()

    def _action(self, q=None, gripper=None, torso=None, base=None) -> np.ndarray:
        """Absolute joint targets for the arm; everything else as before.

        The arm slice must always carry a target: a zero vector would drive the
        arm to the zero pose, which is outside several joint limits.
        """
        env = self.env
        action = np.zeros(env.action_dim, dtype=np.float64)
        a, b = self.arm_slice
        action[a:b] = self._q() if q is None else np.asarray(q, dtype=np.float64)[: b - a]
        if gripper is not None and self.grip_slice:
            action[self.grip_slice[0]] = _gripper_value(gripper)
        if torso is not None and self.torso_slice:
            action[self.torso_slice[0]] = float(torso)
        if base is not None and self.base_slice:
            action[self.base_slice[0]] = float(base[0])
            action[self.base_slice[0] + 1] = float(base[1])
            action[self.base_slice[0] + 2] = float(base[2])
        return action

    def _capture_home_orientation(self) -> None:
        """Remember the downward grasp orientation of the home pose."""
        rot = _quat_to_mat(self.obs["robot0_eef_quat"])
        self.home_rot = rot
        self.home_yaw = _yaw_of_mat(rot)

    def _settle_home(self, steps: int = 18) -> None:
        """Return to the robot's initial pose: absolute joint control starts at zero."""
        action = self._action(self.home_q, torso=0.0)
        for _ in range(steps):
            self.obs, _, _, _ = self.env.step(action)
        self.torso_cmd = 0.0
        self._capture_home_orientation()

    def reset(self, p):
        env = self._require_env()
        self.obs = env.reset()
        self._settle_home()
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
        self._sync_controller()
        self.steps = 0
        return {
            "eef_pos": self._eef_pos(),
            "eef_quat": self._eef_quat(),
            "success": self._success(),
            "steps": 0,
        }

    def _depth_points(self, step: int = 3):
        """World-frame points from the current camera depth image."""
        env = self._require_env()
        from robosuite.utils.camera_utils import (  # noqa: PLC0415
            get_camera_intrinsic_matrix, get_camera_extrinsic_matrix, get_real_depth_map,
        )

        cam, H, W = self.camera, self.height, self.width
        _, dep = env.sim.render(width=W, height=H, camera_name=cam, depth=True)
        depth = get_real_depth_map(env.sim, np.asarray(dep)[::-1].copy())
        K = get_camera_intrinsic_matrix(env.sim, cam, H, W)
        E = get_camera_extrinsic_matrix(env.sim, cam)
        K_inv = np.linalg.inv(K)

        vs, us = np.mgrid[0:H:step, 0:W:step]
        z = depth[vs, us]
        keep = (z > 0.02) & (z < 8.0)
        # Drop the robot's own body: a real robot knows which points are its own
        # links, and without this every clearance reading is the gripper itself.
        try:
            model = self.robot_model.model
            data = env.sim.data
            own = [
                g for g in range(model.ngeom)
                if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(
                    self.robot_model.prefix
                )
            ]
            if own:
                centres = np.asarray(data.geom_xpos[own], dtype=np.float64)
                radii = np.asarray(model.geom_rbound[own], dtype=np.float64) * 1.15
                # points are built below; filter after back-projection
                self._self_geoms = (centres, radii)
        except Exception:
            self._self_geoms = None
        us, vs, z = us[keep], vs[keep], z[keep]
        if z.size == 0:
            return np.zeros((0, 3))
        rays = np.stack([us, vs, np.ones_like(us)], axis=1) @ K_inv.T
        cam_pts = rays * z[:, None]
        world = (E @ np.hstack([cam_pts, np.ones((z.size, 1))]).T)[:3].T
        self_geoms = getattr(self, "_self_geoms", None)
        if self_geoms is not None:
            centres, radii = self_geoms
            dist = np.linalg.norm(world[:, None, :] - centres[None, :, :], axis=2)
            outside = np.all(dist > radii[None, :], axis=1)
            world = world[outside]
        return world

    @staticmethod
    def _clearance(points: np.ndarray, origin, radius: float = 0.14) -> dict:
        """Nearest surface distance along six directions, measured from points."""
        if points.size == 0:
            return {name: None for name in ("up", "down", "left", "right", "front", "back")}
        delta = points - np.asarray(origin, dtype=np.float64)
        out = {}
        for name, axis in (
            ("up", (0, 0, 1)), ("down", (0, 0, -1)),
            ("left", (-1, 0, 0)), ("right", (1, 0, 0)),
            ("front", (0, -1, 0)), ("back", (0, 1, 0)),
        ):
            unit = np.asarray(axis, dtype=np.float64)
            along = delta @ unit
            lateral = np.linalg.norm(delta - along[:, None] * unit, axis=1)
            mask = (along > 0.03) & (lateral < radius)
            out[name] = _num(float(along[mask].min())) if bool(np.any(mask)) else None
        return out

    def _holding(self) -> dict:
        """Whether the fingers currently touch something that is not the robot."""
        env = self._require_env()
        data = env.sim.data
        touching = False
        for i in range(data.ncon):
            contact = data.contact[i]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            r1 = self._is_robot_geom(g1)
            r2 = self._is_robot_geom(g2)
            if r1 != r2:
                touching = True
                break
        return {"touching": touching, "object": self.held_object}

    def _is_robot_geom(self, geom_id: int) -> bool:
        name = mujoco.mj_id2name(self.robot_model.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
        return name.startswith(self.robot_model.prefix)

    def scan(self, p):
        """Summarise nearby geometry from depth: what a depth camera can see."""
        self._require_env()
        origin = p.get("lookat") or self._eef_pos()
        pts = self._depth_points()
        clearance = self._clearance(pts, origin)
        return {
            "camera": self.camera,
            "points": int(pts.shape[0]),
            "lookat": [float(v) for v in origin],
            "clearance_m": clearance,
            "holding": self._holding(),
            "source": "depth",
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
        default_backend = "vision" if self.perception == "sensors" else "ground_truth"
        backend = str(p.get("backend") or default_backend)
        if backend == "vision":
            return self._vision_locate(str(p.get("query", "")).strip())
        if self.perception == "sensors":
            raise RuntimeError(
                "ground truth is disabled in sensor mode: use backend='vision', or "
                "robocasa_scan for geometry from depth"
            )
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
        """Hold the current joint targets and drive the gripper (and base/torso)."""
        env = self._require_env()
        steps = max(1, min(int(p.get("steps", 1)), 200))
        if p.get("torso") is not None:
            self.torso_cmd = float(p["torso"])
        if any(k in p for k in ("dx", "dy", "dz")):
            cur = self._eef_pos()
            target = np.array([cur[0] + float(p.get("dx", 0.0)),
                               cur[1] + float(p.get("dy", 0.0)),
                               cur[2] + float(p.get("dz", 0.0))])
            moved = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2]),
                               "gripper": p.get("gripper"), "max_steps": steps})
            return {
                "steps": self.steps,
                "eef_pos": self._eef_pos(),
                "eef_quat": self._eef_quat(),
                "move_residual_m": moved.get("residual_m"),
                "success": self._success(),
            }
        base = None
        if any(k in p for k in ("forward", "side", "yaw")):
            base = (p.get("forward", 0.0), p.get("side", 0.0), p.get("yaw", 0.0))
        if str(p.get("gripper", "")).lower() in ("open", "opened", "-1"):
            self.held_object = None
        action = self._action(gripper=p.get("gripper"), torso=self.torso_cmd, base=base)
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

    def _target_rot(self, p):
        """Resolve an absolute orientation target from quat or yaw_deg.

        With top_down the orientation is the home pose's downward grasp pose
        rotated to the requested yaw, so it does not depend on where the arm
        happens to point now: deriving it from the current orientation makes a
        failed attempt leave the next target unreachable.
        """
        if p.get("quat"):
            return _quat_to_mat(p["quat"])
        if p.get("yaw_deg") is not None and p.get("top_down") and getattr(self, "home_rot", None) is not None:
            delta = np.radians(float(p["yaw_deg"])) - self.home_yaw
            delta = (delta + np.pi / 2) % np.pi - np.pi / 2
            c, s = np.cos(delta), np.sin(delta)
            return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ self.home_rot
        if p.get("yaw_deg") is not None:
            R_now = _quat_to_mat(self.obs["robot0_eef_quat"])
            delta = np.radians(float(p["yaw_deg"])) - _yaw_of_mat(R_now)
            # A parallel-jaw gripper is symmetric under 180 degrees, so take the
            # equivalent yaw that needs the smaller wrist rotation.
            delta = (delta + np.pi / 2) % np.pi - np.pi / 2
            c, s = np.cos(delta), np.sin(delta)
            return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ R_now
        return None

    def _held_geom_ids(self):
        """Geoms of the object the gripper is actually holding.

        Contacts are not a reliable signal: the fingers touch the object during
        every approach, and an object resting on the counter then looks attached
        to the robot, which makes planning reject the grasp pose as colliding
        with the counter. The worker records what it grasped instead, and clears
        it when the gripper opens or a drop releases.
        """
        env = self.env
        name = self.held_object
        if not name:
            return []
        body_id = getattr(env, "obj_body_id", {}).get(name)
        if body_id is None:
            return []
        model = self.robot_model.model
        start = int(model.body_geomadr[body_id])
        count = int(model.body_geomnum[body_id])
        return list(range(start, start + count))

    def _follow(self, path, gripper, qd_max: float = 1.0) -> int:
        """Drive the arm along a joint-space path, one control step per waypoint."""
        traj = time_parameterise(path, dt=0.05, qd_max=qd_max)
        # Two control steps per waypoint: a single step leaves the arm trailing the
        # commanded path by ~0.06 rad, which is enough to clip obstacles in tight
        # spaces even though the planned path itself is collision free.
        for q_cmd in traj:
            action = self._action(q_cmd, gripper=gripper, torso=self.torso_cmd)
            for _ in range(2):
                self.obs, _, _, _ = self.env.step(action)
                self.steps += 1
                self._after_step()
        return len(traj) * 2

    def _escape_targets(self, target):
        """Back-off poses to try when the arm is wedged against something."""
        cur = np.asarray(self._eef_pos(), dtype=np.float64)
        delta = cur - np.asarray(target, dtype=np.float64)
        norm = float(np.linalg.norm(delta))
        out = []
        if norm > 1e-6:
            away = delta / norm
            out.append(cur + away * 0.08)
            out.append(cur + away * 0.14)
        out.append(cur + np.array([0.0, 0.0, 0.10]))
        out.append(cur + np.array([-0.08, 0.0, 0.06]))
        return out

    def _plan_to(self, q_goal):
        """Plan from the live configuration to q_goal, ignoring what we carry."""
        self.checker.set_attached_geoms(self._held_geom_ids())
        try:
            return self.planner.plan(self._q(), q_goal)
        finally:
            self.checker.set_attached_geoms([])

    def _servo(self, q_goal, gripper, max_steps, target, target_rot, tol, rot_tol):
        """Hold an absolute joint target until the end effector converges."""
        env = self.env
        action = self._action(q_goal, gripper=gripper, torso=self.torso_cmd)
        used = 0
        start_pos = np.asarray(self._eef_pos(), dtype=np.float64)
        progress = 0.0
        residual = float(np.linalg.norm(np.asarray(target) - self._eef_pos()))
        rot_residual = 0.0
        for _ in range(max_steps):
            self.obs, _, _, _ = env.step(action)
            self.steps += 1
            used += 1
            self._after_step()
            residual = float(np.linalg.norm(np.asarray(target, dtype=np.float64) - self._eef_pos()))
            progress = max(progress, float(np.linalg.norm(
                np.asarray(self._eef_pos(), dtype=np.float64) - start_pos)))
            if target_rot is not None:
                R_now = _quat_to_mat(self.obs["robot0_eef_quat"])
                rot_residual = float(np.linalg.norm(_rotvec_from_mat(target_rot @ R_now.T)))
            if residual <= tol and rot_residual <= rot_tol:
                break
        return used, residual, rot_residual, progress

    def move(self, p):
        """Inverse-kinematics move to a world-space pose under joint control."""
        env = self._require_env()
        target = np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=np.float64)
        gripper = p.get("gripper")
        max_steps = max(1, min(int(p.get("max_steps", 80)), 400))
        tol = float(p.get("tol", 0.015))
        rot_tol = float(p.get("rot_tol", 0.03))
        target_rot = self._target_rot(p)

        start = self._eef_pos()
        start_quat = self._eef_quat()
        q_goal, ik_info = self.robot_model.ik_multi(
            target, target_rot=target_rot, iters=300, tries=24, collect=6, margin=0.10, seed=self.steps
        )
        if q_goal is None:
            return {
                "start": start,
                "target": [float(v) for v in target],
                "end": self._eef_pos(),
                "residual_m": _num(float(np.linalg.norm(target - self._eef_pos()))),
                "rot_residual_rad": 0.0,
                "start_quat": start_quat,
                "end_quat": self._eef_quat(),
                "steps_used": 0,
                "total_steps": self.steps,
                "ik_failed": True,
                "failure": "ik_failed",
                "ik": ik_info,
                "success": self._success(),
            }

        budget = min(max_steps, 25)
        used, residual, rot_residual, progress = self._servo(
            q_goal, gripper, budget, target, target_rot, tol, rot_tol
        )
        plan_info = None
        if residual > max(tol * 2.0, 0.05) and bool(p.get("plan", True)):
            path, plan_info = self._plan_to(q_goal)
            if path is not None:
                # The detour gets its own budget: sharing it with the final servo
                # used to consume every step and leave nothing to converge with.
                used += self._follow(path, gripper)
                u2, residual, rot_residual, _ = self._servo(
                    q_goal, gripper, max(40, max_steps), target, target_rot, tol, rot_tol
                )
                used += u2
        recovered = False
        if residual > tol and bool(p.get("recover", True)):
            for escape in self._escape_targets(target):
                self.move({
                    "x": float(escape[0]), "y": float(escape[1]), "z": float(escape[2]),
                    "gripper": gripper, "max_steps": 40, "recover": False, "plan": False,
                })
                u3, residual, rot_residual, _ = self._servo(
                    q_goal, gripper, max(25, max_steps // 2), target, target_rot, tol, rot_tol
                )
                used += u3
                if residual <= tol:
                    recovered = True
                    break
        failure = "ok"
        if residual > tol:
            error = (plan_info or {}).get("error")
            if error == "goal in collision":
                failure = "goal_in_collision"
            elif error == "no path":
                failure = "no_path"
            else:
                failure = "servo_stalled"
        return {
            "planned": bool(plan_info and plan_info.get("nodes")),
            "plan": plan_info,
            "failure": failure,
            "recovered": recovered,
            "progress_m": _num(progress),
            "start": start,
            "target": [float(v) for v in target],
            "end": self._eef_pos(),
            "residual_m": _num(residual),
            "rot_residual_rad": _num(rot_residual),
            "start_quat": start_quat,
            "end_quat": self._eef_quat(),
            "steps_used": used,
            "total_steps": self.steps,
            "success": self._success(),
        }

    def rotate(self, p):
        """Rotate the gripper by a relative amount about world axes."""
        env = self._require_env()
        max_steps = max(1, min(int(p.get("max_steps", 60)), 400))
        tol = float(p.get("tol", 0.03))
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
        q_goal, ik_info = self.robot_model.ik_multi(
            self._eef_pos(), target_rot=R_target, iters=300, tries=24, collect=6, margin=0.10, seed=self.steps
        )
        if q_goal is None:
            return {
                "start_quat": start_quat,
                "end_quat": start_quat,
                "residual_rad": 0.0,
                "steps_used": 0,
                "total_steps": self.steps,
                "ik_failed": True,
                "ik": ik_info,
            }

        anchor = self._eef_pos()
        used, _, rot_residual, _ = self._servo(
            q_goal, p.get("gripper"), max_steps, anchor, R_target, 0.05, tol
        )
        self._last_rot_debug = {
            "ik": ik_info,
            "q_goal": [round(float(v), 3) for v in q_goal],
            "q_end": [round(float(v), 3) for v in self._q()],
            "rot_err_during": round(float(rot_residual), 4),
        }
        end_quat = self._eef_quat()
        err = float(np.linalg.norm(_rotvec_from_mat(R_target @ _quat_to_mat(end_quat).T)))
        return {
            "start_quat": start_quat,
            "end_quat": end_quat,
            "residual_rad": _num(err),
            "steps_used": used,
            "total_steps": self.steps,
            "debug": getattr(self, "_last_rot_debug", None),
        }

    def plan(self, p):
        """Plan a collision-free joint path to a pose without moving the arm."""
        self._require_env()
        target = np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=np.float64)
        target_rot = self._target_rot(p)
        q_goal, ik_info = self.robot_model.ik_multi(
            target, target_rot=target_rot, iters=300, tries=24, collect=6,
            margin=0.10, seed=self.steps,
        )
        if q_goal is None:
            return {"ok": False, "failure": "ik_failed", "ik": ik_info}
        path, info = self._plan_to(q_goal)
        if path is None:
            error = info.get("error", "no path")
            return {
                "ok": False,
                "failure": "goal_in_collision" if error == "goal in collision" else "no_path",
                "plan": info,
                "ik": ik_info,
            }
        return {
            "ok": True,
            "nodes": len(path),
            "direct": bool(info.get("direct")),
            "seconds": info.get("seconds"),
            "checks": info.get("checks"),
            "travel_m": _num(float(np.sum([np.linalg.norm(b - a_) for a_, b in zip(path[:-1], path[1:])]))),
            "ik": ik_info,
            "target": [float(v) for v in target],
        }

    def drop(self, p):
        """Carry the held object above a point and release it there.

        Some receptacles (a sink, a bin) sit below the lowest pose the wrist can
        reach; releasing from just above is the only way to get the object in.
        """
        self._require_env()
        target = np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=np.float64)
        lift = float(p.get("height", 0.02))
        approach = target.copy()
        approach[2] = max(target[2], target[2] + lift)
        moved = self.move({
            "x": float(approach[0]),
            "y": float(approach[1]),
            "z": float(approach[2]),
            "gripper": "close",
            "max_steps": int(p.get("max_steps", 90)),
            # A drop only needs the object roughly over the receptacle; demanding
            # the usual 1.5 cm would abort on a perfectly good release point.
            "tol": float(p.get("tol", 0.02)),
        })
        failure = moved.get("failure")
        if failure not in (None, "ok"):
            # Never release on a failed approach: the object would be dropped
            # wherever the arm happened to stall, which is usually mid air.
            return {
                "released": False,
                "failure": failure,
                "residual_m": moved.get("residual_m"),
                "approach": [float(v) for v in approach],
                "objects": None if self.perception == "sensors" else _object_positions(self.env),
                "eef_pos": self._eef_pos(),
                "success": self._success(),
            }
        self.held_object = None
        self.act({"gripper": "open", "steps": int(p.get("release_steps", 20))})
        for _ in range(int(p.get("settle_steps", 25))):
            action = self._action(gripper="open", torso=self.torso_cmd)
            self.obs, _, _, _ = self.env.step(action)
            self.steps += 1
            self._after_step()
        return {
            "released": True,
            "approach": [float(v) for v in approach],
            "residual_m": moved.get("residual_m"),
            "failure": moved.get("failure"),
            "objects": None if self.perception == "sensors" else _object_positions(self.env),
            "eef_pos": self._eef_pos(),
            "success": self._success(),
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
            self.move({"x": float(target[0]), "y": float(target[1]), "z": hover, "top_down": True,
                       "yaw_deg": yaw, "gripper": "open", "max_steps": 70})
            pre = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2] + 0.10),
                             "top_down": True, "yaw_deg": yaw, "gripper": "open", "max_steps": 90, "tol": 0.015})
            down = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2]),
                              "top_down": True, "yaw_deg": yaw, "gripper": "open", "max_steps": 60, "tol": 0.012})

            # A straight descent sometimes jams against a fixture; approach from
            # the side instead of pushing harder.
            if retry_stalls and (down.get("residual_m") or 0) > 0.04:  # noqa: E501
                self.move({"x": float(target[0] - 0.09), "y": float(target[1]), "z": float(target[2] + 0.14),
                           "top_down": True, "yaw_deg": yaw, "gripper": "open", "max_steps": 60})
                down = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2]),
                                  "top_down": True, "yaw_deg": yaw, "gripper": "open", "max_steps": 60, "tol": 0.012})
            self.act({"gripper": "close", "steps": 15})
            up = self.move({"x": float(target[0]), "y": float(target[1]), "z": float(target[2] + lift),
                            "top_down": True, "yaw_deg": yaw, "gripper": "close", "max_steps": 50})
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
                self.held_object = verify
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
        # A worker that was just spawned has no scene yet; writing a frame here
        # would log a misleading "no simulation open" on a perfectly normal start.
        if self.stream_on and self.env is not None:
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

    def _stack(self) -> dict:
        """What the plugin is actually using, for the UI to display."""
        return {
            "vision": {
                "model": self.detector_id,
                "threshold": self.detector_threshold,
                "method": "open-vocabulary detection + depth back-projection",
            },
            "kinematics": {
                "forward": "mj_forward (site_xpos / site_xmat)",
                "jacobian": "mj_jacSite",
                "ik": "damped least squares",
                "damping": IK_DAMPING,
                "multi_start": 24,
                "limit_margin_rad": 0.10,
            },
            "control": {"type": "JOINT_POSITION absolute", "kp": ARM_KP},
            "planning": {
                "planner": "RRT-Connect",
                "collision": "contact scan",
                "collision_tolerance_m": COLLISION_TOLERANCE,
                "step_rad": RRT_STEP,
                "edge_step_rad": RRT_EDGE_STEP,
                "max_iters": RRT_MAX_ITERS,
            },
            "cameras": {
                "current": self.camera,
                "available": [
                    "robot0_agentview_left", "robot0_agentview_right", "robot0_agentview_center",
                    "robot0_eye_in_hand", "robot0_frontview", "robot0_robotview",
                ],
            },
        }

    def status(self, p):
        env = self._require_env()
        oracle = self.perception != "sensors"
        value = {
            "task": self.task,
            "seed": self.seed,
            "camera": self.camera,
            "perception": self.perception,
            "steps": self.steps,
            "eef_pos": self._eef_pos(),
            "eef_quat": self._eef_quat(),
            # success is simulator evaluation, never a decision input
            "success": self._success(),
            "evaluation_only": {"success": True},
            "holding": self._holding(),
            "stack": self._stack(),
        }
        if oracle:
            value["objects"] = _object_positions(env)
            value["dimensions"] = _object_dimensions(env)
        else:
            value["note"] = (
                "sensor mode: object positions come from robocasa_locate(backend='vision') "
                "or robocasa_scan, not from the simulator"
            )
        return value

    def close(self, p):
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass
            self.env = None
        self.obs = None
        return {"closed": True}


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
    "scan": Sim.scan,
    "plan": Sim.plan,
    "drop": Sim.drop,
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
