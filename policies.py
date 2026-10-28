"""Policies the evaluator can run.

A policy is a small state machine that issues tool calls through the worker
client. That matches how the plugin drives the simulation, so a scripted
baseline and an LLM agent use exactly the same interface:

    class MyPolicy:
        name = "my_policy"
        def __init__(self, worker, context): ...
        def act(self) -> str | None: ...
            # perform one tool call; return its name, or None when finished

`context` carries the task, seed and perception mode so a policy can adapt,
for example by asking for vision instead of ground truth in sensor mode.
"""

from __future__ import annotations


def _matches(result):
    """Vision returns candidates, ground truth returns matches; accept both."""
    return result.get("matches") or result.get("candidates") or []


def _call(owner, tool, args, timeout=600):
    result = owner.worker.call(tool, args, timeout=timeout)
    owner.last_result = result
    return result


def _locate(w, context, query, label=None):
    """Locate through whichever backend the scene's perception mode allows.

    In sensor mode the query must be a natural-language label the detector can
    recognise: "obj" is a simulator name and means nothing to vision, so the
    caller supplies the visible label through context["vision_query"].
    """
    if context.get("perception") == "sensors":
        # Only the task object needs a natural-language label; "cabinet" is a
        # word the detector already understands.
        return w.call("locate", {"query": label or query, "backend": "vision"}, timeout=180)
    return w.call("locate", {"query": query})


class NoopPolicy:
    """Does nothing: measures the baseline success rate, which should be zero."""

    name = "noop"

    def __init__(self, worker, context):
        self.worker = worker
        self.context = context
        self.finished = False

    def act(self):
        self.finished = True
        return None


class GraspPolicy:
    """Locate the task object and grasp it: a grasp-only baseline."""

    name = "grasp"

    def __init__(self, worker, context):
        self.worker = worker
        self.context = context
        self.stage = "locate"
        self.target = None
        self.yaw = 0.0
        self.last_result = None

    def act(self):
        w = self.worker
        if self.stage == "locate":
            found = _locate(w, self.context, "obj", label=self.context.get("vision_query"))
            matches = _matches(found)
            if not matches:
                self.stage = "done"
                return "locate"
            best = matches[0]
            self.target = best.get("grasp") or best.get("world")
            self.yaw = best.get("grasp_yaw_deg") or best.get("yaw_deg") or 0.0
            self.stage = "grasp"
            return "locate"
        if self.stage == "grasp":
            self.stage = "done"
            _call(self, "grasp",
                  {"x": self.target[0], "y": self.target[1], "z": self.target[2],
                   "yaw_deg": self.yaw, "verify": "obj"})
            return "grasp"
        return None


class GraspPlacePolicy:
    """Grasp the task object, then carry it into the nearest cabinet interior."""

    name = "grasp_place"

    def __init__(self, worker, context):
        self.worker = worker
        self.context = context
        self.stage = "locate"
        self.target = None
        self.yaw = 0.0
        self.last_result = None
        self.place = None

    def act(self):
        w = self.worker
        if self.stage == "locate":
            found = _locate(w, self.context, "obj", label=self.context.get("vision_query"))
            matches = _matches(found)
            if not matches:
                self.stage = "done"
                return "locate"
            best = matches[0]
            self.target = best.get("grasp") or best.get("world")
            self.yaw = best.get("grasp_yaw_deg") or best.get("yaw_deg") or 0.0
            self.stage = "grasp"
            return "locate"

        if self.stage == "grasp":
            self.stage = "find_receptacle"
            _call(self, "grasp",
                  {"x": self.target[0], "y": self.target[1], "z": self.target[2],
                   "yaw_deg": self.yaw, "verify": "obj"})
            return "grasp"

        if self.stage == "find_receptacle":
            anchor = self.target
            if self.context.get("perception") != "sensors":
                # The task marker is a simulator name; vision cannot query it.
                marker = _locate(w, self.context, "distr_cab")
                matches = _matches(marker)
                anchor = (matches[0].get("world") if matches else None) or self.target
            found = _locate(w, self.context, "cabinet")
            interiors = [
                region["center"]
                for match in _matches(found)
                for region in (match.get("interior") or [])
                if region.get("region") == "level0"
            ]
            if not interiors:
                self.stage = "done"
                return "locate"
            # Interiors are the reachable centres the simulator reports; the
            # marker itself sits on a surface and cannot be dropped onto.
            self.place = min(
                interiors,
                key=lambda p: (p[0] - anchor[0]) ** 2 + (p[1] - anchor[1]) ** 2,
            )
            self.stage = "drop"
            return "locate"

        if self.stage == "drop":
            # drop() already carries the object to the point; no separate move.
            self.stage = "retreat"
            _call(self, "drop",
                  {"x": self.place[0], "y": self.place[1], "z": self.place[2] + 0.02})
            return "drop"

        if self.stage == "retreat":
            # The task predicate needs the gripper away from the receptacle, and
            # a fixed back-off direction can point into a cabinet wall: go home.
            self.stage = "done"
            home = self.context.get("start_eef") or [self.place[0], self.place[1] - 0.28, self.place[2]]
            _call(self, "move",
                  {"x": home[0], "y": home[1], "z": home[2], "gripper": "open", "max_steps": 120})
            return "move"

        return None


BUILTIN = {
    "noop": NoopPolicy,
    "grasp": GraspPolicy,
    "grasp_place": GraspPlacePolicy,
}


def load(spec: str):
    """Resolve "builtin:grasp_place" or "module:Class" to a policy class."""
    module_name, _, attr = spec.partition(":")
    if not attr:
        raise ValueError(f"policy spec must be module:Class, got {spec!r}")
    if module_name == "builtin":
        if attr not in BUILTIN:
            raise ValueError(f"unknown builtin policy {attr!r}; choose from {sorted(BUILTIN)}")
        return BUILTIN[attr]
    import importlib

    module = importlib.import_module(module_name)
    return getattr(module, attr)
