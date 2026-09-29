# PROGRESS v3: the simulated rig

Tracking the simulator from 2026-09-27 (the rest of the system: [`PROGRESS_v2.md`](PROGRESS_v2.md)).
Continued in [`PROGRESS_v4.md`](PROGRESS_v4.md), where the simulator became a project of its own.

## Design (settled 2026-09-27, grilling session plus follow-ups)

- **The simulation is an alternative world, not a mode of the system.**
  - It provides the same streams the rig does: arm controller sessions (readings, modes, commands) and camera streams (colour + depth, factory intrinsics).
  - The system is unaware of it: backend, mods, topics, MCP and panel.
  - The one switch: `python -m myaloha panel --simulator HOST:PORT`. The backend then reads its devices from that address instead of the vendor driver and USB.
  - Neither side launches or depends on the other.
- **Q1: two device transports in the backend.** The vendor SDK for the rig, and one network protocol (`myaloha/netdev.py`) for anything else.
  - Arm calls are forwarded as-is, so `ArmClient` stands in for `TrossenArmDriver`. `CameraClient` stands in for LeRobot's `RealSenseCamera`.
  - Emulating the Trossen firmware protocol was rejected, and USB cameras can't be emulated at all.
  - A symmetric hardware server for the rig is possible later with the same protocol.
- **Q2: no seeded calibration.** `localize` runs on the simulated rig exactly as on the real one: SIFT on the rendered table, wrist surveys, fixed-camera registration.
  - Each device set keeps its own calibration: `calib/` for the rig, `calib/HOST_PORT/` with `--simulator` (override: `--calib`).
- **Q3: a top-level `simulator/` package.** It may reuse neutral assets: the URDF and meshes, the kinematics library, and the nominal hand-eye poses copied into its config. It must not use the system's perception constants.
  - For example, cube colours come from its own table in `rig.yaml`, not from `cubes.PROTO`.
- **Q4: simulator-only controls stay on the simulator.** Ground truth, new scenes, and unplug/plug are available three ways:
  - its own page (`ui`, default `:8811`);
  - its CLI (`python -m simulator truth | spawn | unplug | plug | info`);
  - its control connection.
  - The system's API, panel and MCP never see them.
- **Q5: tests use the simulator as their environment.**
  - `tests/simrig.py` / `conftest.py` start one on a free port.
  - The in-process fakes are gone: `myaloha/sim.py` with its `SimDriver` monkeypatching, the test-pattern `SimCamera` and the `OBJECT` stall.
- **Q6: physics just enough for pick and place.**
  - Arms track commands exactly, then get joint zero offsets and gravity droop that the encoders don't see.
  - The fingers stall on a cube between the pads.
    - A square grip carries the cube.
    - A twisted grip, or one only grazing the pad tips, slips when lifted.
  - Released cubes settle onto the table or a stack.
  - Arms pass through cubes (no knocking over).
  - Leaders in gravity comp drift as if hand-moved.
  - Real time only.
- **Q7: noise.** Every error is 1-sigma in `rig.yaml` `errors:`, scaled together by `noise` (0 is a perfect world). The seed is printed at launch.
  - Base and fixed-camera poses have no explicit error, because `localize` estimates them.
- **Q8: cameras at 15 fps** (`render_fps`), where the rig runs at 30. 640×480 like the real captures.
- **Configured by one YAML file at launch:** `simulator/rig.yaml`. Override with `--config` and `--set key=value` (dotted keys, e.g. `--set seed=7 --set errors.depth=0`).

## Architecture

| Piece | What it does |
|---|---|
| `myaloha/netdev.py` | The protocol: length-prefixed JSON head plus binary blobs. A connection opens with `{"open": "arm" \| "camera" \| "control"}`. `ArmClient` (every driver call as one request/reply), `CameraClient` (optics on open, then frames stamped on arrival), `control()`. |
| `myaloha/robot.py`, `cameras.py` | `Aloha(hardware=)`, `CameraRig(hardware=)`: with an address, the LeRobot driver is swapped for `ArmClient`, and cameras open as `CameraClient` (no root needed). Nothing above them changed. |
| `simulator/world.py` | The truth. Follower mounts, joint offsets, droop, hand-eye, per-camera true vs reported K, exposure/tint/depth scale/range bias, cubes, the grasp model, settling, sensor post-processing. |
| `simulator/scene.py` | MuJoCo, rendering only. Followers built from the URDF with their meshes (including the D405 and its mount), a finite textured table with an edge, the floor, 24 mocap cube slots, the four cameras with their true intrinsics. One render thread owns the GL context. |
| `simulator/controller.py` | An arm controller as the driver sees it: modes, goal-time interpolation, robot output; followers act in the world. |
| `simulator/server.py` | Serves arms by IP, cameras by serial, and control calls, on `listen` (default `:8810`). |
| `simulator/ui.py`, `ui.html` | The simulator's own page: truth top view, the four cameras, new scene, unplug/plug per device. |

## Log

- **First, an in-process numpy renderer (ray-cast boxes) behind `--sim`.** It worked for perception checks, but the user rejected the design: the system must not be sim-aware. It was rebuilt as the separate process above.
- **Box arms replaced by URDF meshes via MuJoCo** (user: "lacking surfaces of the robot arms"). About 15 ms per camera for colour and depth.
  - The principal point x sign was wrong at first. Now sub-pixel: a cube is drawn within 0.3 px of where K projects it.
  - Wrist R showed sliced faces: the near plane (4 cm) cut the closed fingers. It is now 4 mm; depth still drops below 7 cm like a D405.
- **localize on the sim works unchanged** (seed 7, noise 1):
  - 1069 SIFT inliers, fit RMS 2.04 mm.
  - Bases estimated 940.7 mm apart against a true 932 mm; the joint offsets and droop show up as that 8.7 mm.
  - High camera 5.1 mm / 0.0° off; Low camera 2.7 mm / 0.28° off (Low registered via the rectified method, 2.9 px RMS).
- **locate_cubes on the sim** (8 cubes, stacks up to 3): 8/8 found, median 3.3 mm, max 9 mm.
- **pick_place end to end through the API**, judged by the simulator's truth:
  - Cube at yaw 80°: placed 8.5 mm from the target.
  - The failing case (yaw 77.6°) reproduced every time: "missed the cube: the fingers closed to 37 mm". The saved look frame showed:
    - the fingers were still closed while looking, so their tips hid the cube;
    - the arm's own shadow fell on the top face, so "orange" was a sliver along one edge and the yaw came out 8°. The pads then jammed on the twisted cube.
  - Fixed in `pick_place`: open the fingers before the look, and ignore a blob under 40% of a top face at that range. After the fixes: yaw 77° measured, grasp at 28.7 mm, placed 3.6 mm from the target.
  - `pick_place` now saves each look (frame, blob, top points, yaw) in `logs/pick_place/`, for the rig too.
- **Softer sim lighting**: a key light with a shadow, plus a fill light and more ambient, closer to the lab.
- **Tests on the simulator: 56 pass** (≈1:45).
  - `test_pick_place` now asserts the cube really ends up within 5 mm of the target.
  - `test_cameras` uses real dropouts (`unplug` / `plug`).
- **Depth made as bad as the rig's** (user: "real-world depth is even worse").
  - Compared with the real captures, the rig's depth has four errors:
    - a band along the left edge with no depth;
    - black shadows to the left of near objects;
    - blotchy noise growing with Z²: 0.41 mm at 0.22 m, 11 mm at 1.03 m on a flat table patch;
    - a grid of zero seams from the backend re-aligning the depth stream to colour.
  - The simulator now models the D405 as passive stereo from its left imager, with settings in `rig.yaml` under `sensor:` and `errors:`:
    - disparity noise that is blotchy over `correlation_px`;
    - no depth where the right imager can't see (the left band, shadows);
    - holes on dark, textureless surfaces;
    - lost edges and random dropouts.
  - It streams depth in its own intrinsics (`depth_focal`), unaligned, so the backend's `align_depth` makes the same seams as on the rig.
  - Result: wrist 0.44 mm (rig 0.41), high 6.6 mm (rig 11).
  - `locate_cubes` still finds 8/8: median 4.7 mm, worst 12.8 mm.
  - The simulator now logs a device session that dies, where before a camera thread failed silently.
  - **Off for now** (`sensor.artifacts: false`, 2026-09-27, user's call). The band, shadows, dark-surface holes and unaligned stream are skipped, and depth is sent aligned. The noise, scale, range bias, dropouts and edge losses in `errors:` still apply.
- **Panel: hovering a cube** puts edges in a darker shade of its colour on it, plus a label: `cube <track> · colour`, level or gripper, (x, y).
  - `pick_place` given a missing track now lists the tracks there are (ids start at 1).
  - Fixed a startup race in the panel: `majorStrip` read `S.mods` before the first state frame.
- **Found on the way:** `calib/` was root-owned, because the real backend runs under sudo. The user chowned it. Files the sudo backend writes will be root-owned again (open item).

## Now

- Nothing running a task. The simulator (seed 7) and a backend on it may be left running from the session.

## Open

- **Visual alignment with the real captures** (deferred by the user):
  - the finite table's real size and wood, the large tag board at the front, the green carpet and aluminium frame;
  - lens distortion with the real coefficients;
  - configurable artifacts in `rig.yaml`: vignetting, motion blur, auto-exposure drift, colour/depth misregistration, IR speckle and dropouts on dark or shiny surfaces.
  - A photo of the table can already be used: `table.texture: path`.
- **Perception issues the sim shows (likely real too):**
  - blue reads as purple on shaded side faces (the middle of a 3-stack);
  - a ghost column about 2 cm beside a 3-stack;
  - the overhead yaw can be about 13° off (the wrist look corrects it before a grasp).
- **The same pick_place fixes on the rig:** fingers open for the look, sliver rejection. Also check the grasp height there: the pads are the last 20 mm before the tool point.
- **Files written by the sudo backend** (calibration, logs) should belong to the desktop user.
- **Physics not modelled:** fingers knocking cubes over, arm–cube collisions, cube friction and sliding.
- **Frame rate** is 15 fps against the rig's 30; one render thread serves all four cameras.
- **High camera depth** is a little clean: 6.6 mm against 11 mm on a flat patch. Raising `depth_subpixel` would make the wrists noisier than the rig's; the rig's far-range degradation seems to grow faster than Z².
- **From /simplify (2026-09-27), deeper changes left for later:**
  - **Camera backend:** `CameraRig` still branches on `hardware`, and its retry rule is the RealSense USB-replug one even for network cameras. Inject a camera backend (open, needs root, when to retry) like `Arm`'s driver.
  - **Per-rig data directory:** calibration and logs should live under one root per rig, set on `Aloha`. Now `localize.CALIB` is patched from launch, while `logs/` (localize frames, pick_place looks) is shared by the rig, the sim and tests.
  - **`pick_place._measure`** runs its own cube detector. Use `cubes.observe` on the wrist view, so detection fixes help both.
  - **Held cubes** drop observations within 4 cm in xy of the gripper at any height, so the cube under a picked one is lost. Make it a 3-D test.
  - **Occlusion** is "a tool point within 6 cm and below 0.35 m", kept with no time limit. A per-track visibility test would cover every occluder.
  - **`pick_place._watch`** re-associates changing track ids itself. Fix id stability in `Tracker` if it still happens.
  - **Units** are keyed by param name in the panel (`UNIT`). Put `unit` in each mod's Params metadata.
  - **Topic requirements:** `topics` is set on the robot by the server, and a `waiting` topic counts as up.
- **Dark knots in the procedural wood** lose depth (the real table has none); this goes away with a real table texture.
