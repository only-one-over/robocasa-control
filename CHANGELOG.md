# Changelog

All notable changes to this plugin. Versions follow the package version; the
project is pre-1.0, so behaviour may still change between minor versions.

## 1.2.0 — batch evaluation

The plugin can now be used as an experiment harness, not only through chat.

- `evaluate.py` runs a policy across tasks, seeds and episodes in one worker
  process and writes a CSV (success, steps, tool calls, wall time, last action)
  plus optional per-episode HDF5 recordings (`--record`).
- `policies.py` defines the policy interface: a small state machine that issues
  tool calls, with three builtins (`noop`, `grasp`, `grasp_place`) and
  `module:Class` loading for your own.
- `rpc_client.py` is the shared worker client; `self_check.py` now imports it.

First measurements on seed 4 with `grasp_place`:

| condition | calls | outcome |
|---|---|---|
| oracle | 5 | grasp ok, placement ok, task false — the cabinet door is closed, so the object bounced back to the counter |
| sensors | 3 | **grasp ok**, then stopped: receptacle interiors are oracle-only, and vision returns boxes without an interior |

That asymmetry is the useful part: the perception gap shows up in receptacle
reasoning, not in grasping.

## 1.1.1 — show the stack in the panel

The panel now lists what the plugin is actually using: detector and threshold,
IK method with damping and multi-start count, controller type and gain, planner
with step size and collision tolerance, and the current camera among the six
available. The numbers come from status.stack, which reads the same constants
the algorithms use, so the display cannot drift from the code.

## 1.1.0 — sensor mode

`open(perception="sensors")` removes every ground-truth shortcut, so the agent
only sees what a real robot has:

- `locate` refuses the oracle backend and falls back to the detector;
  `status` reports no object positions at all.
- `robocasa_scan` projects the depth image into a point cloud (robot's own geoms
  filtered out) and reports clearance up/down/left/right/front/back.
- `status` reports `holding` from gripper contact, and the success flag is
  marked evaluation-only so it cannot be used as a decision input.

Measured on seed 4: the detector put the carrot at (1.872, -0.774, 0.953) with
yaw -31.7 while the simulator has (1.852, -0.773, 0.940) at -28 — a 2 cm / 3.7
degree estimation error, which is the realistic cost of dropping the oracle.

## 1.0.2 — the preview no longer vanishes during a run

- While a long call held the worker, `/robocasa/state` answered with a reduced
  payload that had no task name. The floating widget called
  `state.task.replace(...)`, threw, and React unmounted it — so the picture
  disappeared as soon as anything started running. The route now serves the last
  known scene plus a `busy` flag, and the client tolerates missing fields.
- Aborting a tool call no longer kills the worker: the call finishes in the
  background and its reply releases the busy lock.
- `stop()` clears `sceneOpen`, which previously left the plugin believing a
  scene was open after the process had been replaced.

## 1.0.1 — client module fix

- The zh locale table called its own lookup function before it was defined, so
  the client module threw a temporal-dead-zone error and the plugin failed to
  load in the browser. The table now holds literal strings.
- Added `client-check.mjs`: it loads the module through a stub loader and renders
  every registered component, which `node --check` cannot catch.

## 1.0.0 — first stable release

The plugin controls a RoboCasa kitchen simulation from chat: 16 tools, a
configuration panel, a floating live view, and a motion stack that plans around
obstacles. Everything below was exercised against a live simulation.

**Tools**: doctor, open, locate, look, move, rotate, act, grasp, drop, plan,
status, reset, restart, close, preview, frame.

**Motion**: absolute joint position control through damped least squares IK,
with RRT-Connect detours when the direct path stalls, back-off recovery when the
arm wedges, and a `failure` enum that distinguishes unreachable poses from
blocked ones.

**Perception**: Grounding DINO detection with depth back-projection, plus ground
truth lookup by simulator object name.

**UI**: a sidebar panel that is configuration-only (stream resolution, default
camera, detector threshold, preview interval, running simulations with the
ability to close them) and a draggable floating widget that is the only surface
showing live frames.

A recorded demo lives in the repository: `media/demo-pick-place.mp4` (full run) and
`media/demo-pick-place.gif` (21 s highlight), both captured from the plugin's own
preview stream at 20 Hz while the carrot was picked from the counter and dropped
into the sink.

Known limits, stated plainly:

- the arm cannot recover from every jam; `recover` retries back-off poses but
  the successful case has not been reproduced on demand;
- objects longer than the gripper opening are grasped at the edge;
- a receptacle whose bottom sits below ~0.99 m needs `drop` rather than a place;
- one worker per plugin instance: a second concurrent tool call is refused
  rather than queued.

## 0.24.0 — portable configuration

- Python is discovered instead of hardcoded: configured path, `ROBOCASA_PYTHON`,
  `CONDA_PREFIX`, `VIRTUAL_ENV`, then every conda environment (preferring names
  containing "casa"), each verified with `import robocasa`.
- Runtime state moved out of the package into `~/.dsh/robocasa-control`
  (`settings.json`, `preview.png`, run directories), so a read-only install works.
- `cordis.patch.yml` no longer carries absolute paths; every setting has a default.
- Mutating calls take an exclusive lock and return a `busy` error instead of
  queueing behind a long grasp.
- The last opened task/seed and the crash message are exposed on `/robocasa/state`.
- `POST /robocasa/workers` only accepts children of this host process, and
  state-changing routes reject cross-origin requests.
- UI strings moved into a zh/en table picked from the browser language.
- `engines.dsh`, `npm run check`, `npm run test` and `CHANGELOG.md` added.

## 0.23.0 — unsticking

- `move` records how far the arm actually travelled and, on failure, tries
  back-off poses (away from the target, then upward) before servoing again.
- Results report `recovered` and `progress_m`.

## 0.22.0 — plan/drop, honest tools

- New `robocasa_plan` (plan without moving) and `robocasa_drop` (release over a
  receptacle the wrist cannot reach into; refuses to release on a failed approach).
- `move` reports a `failure` enum: `ik_failed`, `goal_in_collision`, `no_path`,
  `servo_stalled`.
- Tool descriptions rewritten for the joint-control implementation.
- The carried object is tracked explicitly; contact inference made a grasp pose
  look like it collided with the counter.
- Grasp orientation comes from the home pose, so a failed attempt cannot make the
  next target unreachable.
- Orphaned workers from earlier plugin generations are reaped on load.
- `self_check.py` drives the worker over the real protocol.

## 0.21.0 — planning inside move

- `move` falls back to RRT-Connect when the direct servo stalls, with separate
  step budgets for the detour and the final servo.

## 0.20.0 — absolute joint position control

- The arm is driven by joint targets resolved through damped least squares IK,
  replacing OSC end-effector deltas. Steady state accuracy improved from ~10 mm
  to 0.6 mm.
- Fixed the 90° offset between the IK site frame and the observed eef frame.
- IK prefers the branch closest to the current configuration and rejects
  solutions inside a joint-limit margin.

## 0.19.0 / 0.18.0 — planning layer

- Added `planner.py`: MuJoCo-native collision checking, DLS IK, RRT-Connect,
  shortcutting and time parameterisation. No new dependencies.
- Fixed three bugs found by execution testing: reversed paths from tree swaps,
  planning mutating the live simulation state, and IK solutions pinned at limits.

## 0.17.0 / 0.16.0 — observation UI

- The panel became configuration-only; the floating widget is the sole display.
- View controls (mode, azimuth, elevation, distance) live on the floating widget.
- Fixed the preview writer passing an array to a bytes write, and a resolution
  setting that never applied because it was ordered after the first frame.

## 0.14.0 / 0.13.0 — first UI

- Sidebar panel with a live MJPEG stream, state polling, settings and a list of
  running simulations with the ability to close them.
- The floating overlay lets the conversation stay usable while watching.

## 0.12.0 — web surface

- Host routes `/robocasa/state`, `/robocasa/frame.mjpg`, `/robocasa/preview` and
  `/robocasa/settings`, plus a client module registering the sidebar icon.

## 0.11.0 and earlier — core

- JSON-RPC worker over stdin/stdout holding one MuJoCo/RoboCasa session.
- Tools: doctor, open, locate, look, move, rotate, act, grasp, status, reset,
  restart, close, preview.
- Free camera, third-person preview, vision detection (Grounding DINO) with
  depth back-projection, and a 12-check environment doctor.
