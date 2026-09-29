# PROGRESS v4

Tracking from 2026-09-27 (after [`PROGRESS_v3.md`](PROGRESS_v3.md), the simulated rig; earlier:
[`PROGRESS_v2.md`](PROGRESS_v2.md), [`PROGRESS.md`](PROGRESS.md)). All progress files live in `docs/`.

## Rules (settled 2026-09-27)

- **The simulator is a project of its own.**
  - `simulator/` has its own `pyproject.toml`, `uv.lock` and `.venv`, and is excluded from the root workspace.
  - It imports nothing from `myaloha`, and carries its own copies of what it needs:
    - the follower URDF and meshes (`src/simulator/assets/`);
    - rigid-transform helpers (`geometry.py`);
    - its own implementation of the wire protocol (`protocol.py`).
  - Kinematics come from MuJoCo, on the model it renders.
  - It prints no backend commands.
  - The only thing it shares with the backend is the network device protocol, implemented once on each side.
- **The backend never reads the simulator's ground truth.** It gets only what the rig would give: arm sessions and camera streams at an address (`panel --simulator HOST:PORT`).
  - The simulator's control channel (truth, spawn, unplug) is not in `myaloha`. The test harness (`tests/simrig.py`) and the simulator's own CLI and page use it.
  - Tests may use the truth only to judge the system, never to feed it.

## Done

- **Audit of the two rules (before the fixes).**
  - The simulator imported `myaloha.netdev`, `myaloha.coords` and `myaloha.vendor.heron_kinematics`, read the backend's URDF and meshes, and lived in the backend's environment (a `sim` dependency group).
  - The system code itself was clean: no truth reads, no simulator imports.
  - The pick_place tests fed truth into the system: the true mounts as the calibration, and a stand-in for locate_cubes built from the true cube poses.
  - `netdev.control()` sat in the backend package, unused by the system.
- **Simulator made independent.**
  - `simulator/` is now a src-layout uv project (`python -m simulator` from inside it, or `uv run --project simulator ...`). Config paths are relative to `rig.yaml`.
  - The root `pyproject.toml` lost the `sim` group, and MuJoCo is no longer in the backend environment.
  - Its own tests: `cd simulator && uv run pytest`.
- **Honest tests.**
  - `tests/simrig.py` starts the simulator as a subprocess in its own environment, on free ports, and reads its address from its output.
  - `tests/test_pick_place.py` runs the real chain at full noise: the backend calibrates itself (`localize`), finds the cube (the `locate_cubes` topic), then picks. The simulator's truth only checks the result.
    - Covered: the bases found, a refused plan, a dry run, a full pick and place (on target within 2 cm), a handover (below), and a lost cube (stop and reset).
    - Calibration and logs go to a temp directory, not the lab's.
  - `localize` now accepts a frames directory outside the repo.
- **Bug: cubes went through each other** (a cube put onto a 3-stack sank into it). Fixed in the simulator's physics:
  - A carried cube that comes down onto cubes lands on the top of that column and slides up between the pads. One that comes in from the side shoves the column (with what's stacked on it).
  - A released cube rests on the highest cube its centre is over, or slides off beside it. It never settles inside a stack.
  - The overlap test is a separating-axis test on the cubes' outlines.
  - The simulator's tests cover landing on a stack, releasing inside one, a side shove, and a handover.
- **pick_place `handover` toggle.** Two arms work together when the picking arm can't reach the target.
  - The giver (the arm that reaches the cube) picks it as usual. It then turns its tool horizontal toward the other arm and holds the cube at the handover point, midway between the bases and 20 cm up.
  - The taker comes in facing it, turned 90° about that axis, so each pair of pads sits on its own pair of faces with no contention. The taker's tool point goes 4 mm past the cube's centre for a deep grip.
  - The taker closes, the giver opens, backs off 8 cm and rests. The taker turns top-down and places the cube square to W.
  - New states: `handing_over`, `receiving`, `exchanging`.
  - The planner solves both arms before anything moves, and tries four roll choices for the grip.
  - A solo run that can't reach suggests `handover=true`.
  - The simulator lets a cube held by one gripper be taken by the other. When the carrier opens with the other still holding, the cube passes over instead of dropping.
  - End-to-end test: a cube near the left arm to a target near the right arm. Neither can do it alone; the handover does, and the cube ends within 2 cm of the target by the simulator's truth.
- **Panel: "Table" became "Annos",** a checklist over the 3-D view, remembered in the browser. A opens it.
  - Table: the top camera draped on the table (as before; T toggles it, with a reason when it's blocked).
  - Grid: 3 cm, square to W, on the table top.
  - Reach: each follower's pick-and-place ring about its base, clipped to the table. It comes from the new `GET /api/reach`, built from the calibration and pick_place's own reach limits.
- **Docs:** `PROGRESS.md`, `PROGRESS_v2.md`, `PROGRESS_v3.md` and this file moved to `docs/`, with README and CLAUDE.md pointing there.

## Stability of the end-to-end tests (7 runs of `tests/test_pick_place.py`, 2026-09-27)

- **Fixed in pick_place:**
  - **An empty wrist view was ignored.** The cube had been removed, and the tracker kept its track because the arm hovered over the spot (the occlusion hold), so the mod grasped blind and closed on nothing. Now, when the wrist camera looks right where the cube should be and sees none, the cube counts as gone: the task stops and the arms reset, with that reason.
  - **One bad look broke a grasp.** The fallback then used the overhead tracker's yaw, about 15° off, and the fingers jammed at 35 mm. Now the mod looks again with a fresh frame first. Every look is saved in `logs/pick_place/` with what came of it ("found", "no red blob", "only N px", ...).
- **Fixed in the tests:**
  - The cube helper took a stale track from the previous test. It now waits for the cube near where it was put.
  - The dry-run motion check was tighter than the reading jitter at noise 1.
- **Seen once each, not reproduced since:**
  - locate_cubes "waiting on cam_params" while cam_params served (the first run).
  - No red cube reported near a freshly spawned one within 33 s (run 6).
  - The test helper now prints both topics' status and every cube locate_cubes has, so the next occurrence explains itself.
- **The last full suite:** 58 passed (4 min 44 s), with every end-to-end test. The simulator's own tests: 4 passed.

## Open

- **Overhead yaw** can be about 15° off for a lone cube (9 px across at the high camera). The wrist look corrects it before a grasp, but a failed look leaves only that.
- **The tracker holds a track under a low gripper indefinitely** (the occlusion rule). pick_place now checks with its wrist camera; other consumers can't.
- **Carried over from v3:** visual alignment with the rig, blue/purple on shaded faces, ghost columns beside tall stacks, a per-rig data directory, a camera backend for `CameraRig`, and the others in [`PROGRESS_v3.md`](PROGRESS_v3.md).
- **Handover limits:**
  - The handover point is fixed (midway, 20 cm up).
  - The arms don't check each other's links for collisions: the giver backs off before the taker moves on, and the taker waits at rest until the giver is at the handover point.
  - Not tried on the rig.
