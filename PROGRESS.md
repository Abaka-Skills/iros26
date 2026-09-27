# PROGRESS

> Frozen on 2026-09-26; tracking continues in `PROGRESS_v2.md`.

## Now

The panel is wired to the real backend. The next step needs someone at the rig: the first powered
runs of the mods, and the first live camera run under sudo.

## Panel requirements (from review, 2026-09-26)

- There will be many mods, not three. The picker must scale: search, categories, ⌘K.
- The FSM needs a careful visual at two levels: the robot's major state (offline / idle / running) with its history, then the running mod's own machine with time spent per state.
- A log panel is always on screen.

## Done

**2026-09-26: rails parallel in the table projection; Table button gated**

- The draped High image made the table's parallel rails converge by 2.2°. There were two causes:
  - PnP from matches that cluster on the ~20 cm sheet fixes the camera's tilt poorly: two registrations minutes apart differed by 1.6° and 30 mm.
  - The lens distortion was ignored.
- `register_fixed` now takes tilt and height from the High camera's own depth. It uses the table plane, pre-selected by the PnP plane (±5 cm), RANSAC within 2 cm, and a 15-frame median, since D405 noise at 1 m is about 6 mm. The SIFT matches then solve only the in-plane yaw and offset, with a 2-D similarity whose scale checks the depth scale; on the rig it is 1.0014.
- Lens model: RealSense coefficients are read at connect (High: `inverse_brown_conrady`), applied to matched pixels and the depth back-projection, and to the panel's drape UVs.
- Measured on the rectified top view (Hough lines): rail convergence 2.16° → 0.93°. The rest may be the depth plane's tilt error (0.7 of samples within 2 cm) or the rails themselves; averaging several registrations would tell.
- The raw High frame (colour, depth, K, lens) is saved with each registration (`logs/coordinates/<ts>/fixed_high.npz`) for offline checks. `View.load_dir` ignores it when loading wrist views.
- Panel: the Table button is grey and disabled, with the reason as a tooltip, until a coordinate calibration exists and while `establish_coordinates` is running.

**2026-09-26: parallel-rails prior; axes on top; param help**

- The bases sit level on parallel rails, so `establish_coordinates` has `parallel_bases` (default on).
  - Each arm's frame is levelled on its measured table plane, the bases are fixed exactly facing, and only the offset is solved (distance, slide along the rails; heights from the planes).
  - W's x is perpendicular to the rails. `T_W_mount` holds the level, physical poses (what the panel draws). `T_W_base` keeps each arm's apparent 3–4° tilt, which is gravity sag seen through FK, so FK points still land on the table.
  - The report keeps the free fit's yaw as a check on the prior.
- On the rig, the prior holds: the constrained fit is 2.41 mm RMS vs 2.31 mm free, and the table planes agree to 0.01 mm. Registration `20260926_153215`: 932.4 mm apart, left base 10.4 mm along its rail (cube registration: 9.0), heights 21.4 / 29.0 mm.
- Repeatability: the 15:04 and 15:12 surveys agree to 0.2 mm (distance), 0.04° (free yaw) and 0.3 mm (lateral).
- `POST /api/coordinates/resolve?parallel=…` re-solves the latest survey's frames without motion and re-registers the High camera.
- Panel:
  - W axes are solid and drawn over everything, including the table image (they sit in three's transparent pass, after it).
  - Run / Stop labels are short.
  - Every mod param can carry help text and a figure (dataclass field metadata, shown via a (?) popover); `base_distance` shows where to measure (`panel/assets/figures/base_distance.svg`).

**2026-09-26: parallel-rails prior; axes on top; param help**

- The bases sit level on parallel rails, so `establish_coordinates` has `parallel_bases` (default on).
  - Each arm's frame is levelled on its measured table plane, the bases are fixed exactly facing, and only the offset is solved (distance, slide along the rails; heights from the planes).
  - W's x is perpendicular to the rails. `T_W_mount` holds the level, physical poses (what the panel draws). `T_W_base` keeps each arm's apparent 3–4° tilt, which is gravity sag seen through FK, so FK points still land on the table.
  - The report keeps the free fit's yaw as a check on the prior.
- On the rig, the prior holds: the constrained fit is 2.41 mm RMS vs 2.31 mm free, and the table planes agree to 0.01 mm. Registration `20260926_153215`: 932.4 mm apart, left base 10.4 mm along its rail (cube registration: 9.0), heights 21.4 / 29.0 mm.
- Repeatability: the 15:04 and 15:12 surveys agree to 0.2 mm (distance), 0.04° (free yaw) and 0.3 mm (lateral).
- `POST /api/coordinates/resolve?parallel=…` re-solves the latest survey's frames without motion and re-registers the High camera.
- Panel:
  - W axes are solid and drawn over everything, including the table image (they sit in three's transparent pass, after it).
  - Run / Stop labels are short.
  - Every mod param can carry help text and a figure (dataclass field metadata, shown via a (?) popover); `base_distance` shows where to measure (`panel/assets/figures/base_distance.svg`).

**2026-09-26: top camera projected onto the table**

- `coords.register_fixed()`: a fixed camera's pose in W from one colour frame. SIFT on it is matched to the saved wrist-view features (3D in W), then `solvePnPRansac` + LM refine; no depth needed (the D405's depth is poor at 1 m).
  - `establish_coordinates` now registers the `high` camera after each run.
  - `POST /api/coordinates/cameras/{key}` re-registers against the latest run's saved frames, with no arm motion.
  - Synthetic test: an overhead camera is recovered within 10 mm / 0.5°.
- The High camera on the rig: 140/262 matches, 0.78 px RMS; 1.01 m above the table, 38 cm toward the operator, looking down about 61°.
- Panel: a **Table** button (key T) in the view chip drapes the live High frame onto the table plane: a 120×76 grid whose UVs are the camera projection of each vertex; unseen areas are discarded; untone-mapped; about 6 frames/s.
- Fixed at the next restart: the register endpoint's final log line crashed (the `/api/log` handler was also named `log`), after the result had already been saved.

**2026-09-26: 3-D view controls in the panel**

- A view chip at the bottom centre of the rig area: turn ±90° (Q/E), default view (H), top (R) and front (F) views, all on critically damped springs; `camera.topElevation` / `frontElevation` / `springStiffness` in `panel/config.json`.
- Free camera (toggle, V; remembered per viewer): drag to orbit (elevation 3–88°), scroll to zoom (0.15–3×), right- or Shift-drag to pan, double-click to reset. The automatic fit to the rig stays underneath, so zoom and pan are relative to it.

**2026-09-26: establish_coordinates (SIFT, no markers)**

- `myaloha/coords.py` (solver) + `myaloha/mods/coordinates.py` (mod, category calibration). Needs both followers and both wrist cameras.
  - Each follower in turn moves its wrist camera through 5 steep viewpoints over the table between the arms (gripper open), while the other rests folded.
  - SIFT keypoints are lifted to 3D with the aligned depth and FK × hand-eye; descriptors are matched across the arms, then a RANSAC + Kabsch rigid fit gives the left base in the right base frame.
  - Each arm's depth gives the table plane.
  - W: on the table, origin midway between the follower bases, x from left base to right base, z up.
- Nothing is site-specific: SIFT uses whatever texture is there. The base distance (default 0.91 m) only aims the cameras, and planning succeeds from 0.86 to 1.0 m. Synthetic tests (ray-cast desks: clutter texture, AprilTag-like texture, different spacing, twist and offset) recover the layout within 2 mm / 0.2°; a blank desk gives a clear error.
- Reused from `_tmp/aloha_grid_demo` (built on this same rig):
  - the Heron kinematics and URDF, vendored in `myaloha/vendor/` (hash-verified);
  - the hand-eye calibration, in `config/handeye/`;
  - the viewpoints;
  - its cube registration, kept as an evaluation reference in `config/reference_registration.yaml` (remove it at a new site).
- Cameras: colour/depth intrinsics and the depth→colour transform are read at connect; `CameraRig.capture()` returns a 5-frame median depth aligned to colour.
- API: `GET /api/coordinates`, `GET /api/coordinates/matches.jpg`; results in `calib/coordinates/<ts>.yaml` + `latest.yaml`; raw frames in `logs/coordinates/<ts>/` (gitignored) for offline re-solving.
- Panel: the followers are drawn at their measured poses, with W axes at the origin, a dashed base line and a result card with the evaluation.
- First real run, 2026-09-26 15:04 (`calib/coordinates/20260926_150436.yaml`):
  - bases 931.2 mm apart, left yaw −0.98°, lateral 4.9 mm;
  - 900/1204 matches consistent, 2.2 mm RMS; views agree to 2.4 / 2.7 mm; table planes agree to 0.22° / 0.4 mm;
  - vs the cube registration: +0.7 mm, +0.1°, −4.1 mm lateral;
  - split-half: 0.65 mm / 0.6°. Rotation is the weakest part, because the matches cluster on the ~20 cm printed sheet.
- Next, if more accuracy is needed:
  - wider view spread, or a bundle adjustment that also uses the table plane;
  - a pointer check: both TCPs hover over the W origin in turn, and the high camera shows whether they meet.

**2026-09-26: gravity comp at the joint limits; camera socket; leader models; foldable mods panel**

- Leader shaking (left) and lifting (right) at the start of teleop: the leaders were released into gravity comp at the zero pose, which sits exactly on the shoulder/elbow lower limits (joint_1, joint_2 = 0 rad in the URDF), so the controller's limit push fought the compensation.
  - Teleop and instruction now go through `staging`: move to LeRobot's staged pose (`stage_time`), then release. This replaces teleop's `aligning` and instruction's `entering`.
  - `Arm.free()` refuses within 0.1 rad of those limits.
  - Waiting on a re-test at the rig.
- Cameras in the panel now use one WebSocket (`/api/cameras/ws`; binary JPEG messages, switch with `{key, kind}`). One MJPEG stream per tile hit the browser's six-connections-per-host limit: after two depth switches the rest went black, and REST calls, the E-STOP's included, could queue behind the streams. The MJPEG endpoints stay in the API.
- Leaders render from Trossen's official `wxai_leader_left/right.urdf` (handle and leader fingers); the meshes and derived JSON are in `panel/assets/meshes/wxai/`.
- The mods panel folds to a rail of status dots; it folds automatically below 1280 px, and the viewer's choice is remembered.

**2026-09-26: emergency stop, hardware requirements, left/right map**

- Emergency stop:
  - `Aloha.emergency_stop()` brakes every connected arm in parallel (`Mode.idle`) and latches; while latched, every arm command raises `EmergencyStop` and no mod can start. Reset it with `reset_emergency_stop()`.
  - API: `POST /api/estop` (an async handler, so it runs at once), `POST /api/estop/reset`, and `{"type": "estop"}` over the WebSocket.
  - Panel: the E-STOP button fires on press and sends over both paths, Space triggers it, and a banner resets it with two clicks.
  - Verified: in sim it halts a running mod; on the real rig it answered in 6.5 ms.
- Minimal hardware per mod:
  - Each mod declares `REQUIRES` (`Need`: at least N of these arm groups, filtered by the requested sides). It is checked before the robot is claimed; a shortfall raises `RequirementError`, and the API returns 412 with the reason.
  - The panel shows each mod's needs, disables Run with the reason, and disables side toggles that can't run.
  - Current rules: reset needs any arm, teleop a leader+follower pair, instruction a follower.
- Without the leaders: arms connect in parallel (one timeout in total), unreachable leaders show as optional in the preflight, and teleop is blocked while reset and instruction run (tested in sim).
- Arm map fixed (the left/right sides were reversed): leader L `.3`, R `.2`; follower L `.5`, R `.4`; wrist cameras renamed `wrist_left` / `wrist_right`.
- `POST /api/shutdown`; uvicorn now closes lingering camera streams after 2 s, which used to block shutdown.

**2026-09-26: backend; panel and visitor page fused**

- Backend (`myaloha/server.py`, FastAPI on 127.0.0.1:8765). The panel only visualizes it.
  - REST: `/api/status`, `/api/arms[/{name}]`, `POST /api/arms/connect`, `/api/cameras`, `/api/mods[/{name}]`, `POST /api/mods/{name}/run` (202, 409 busy, 422 bad params), `POST /api/stop`, `/api/log`, `/api/timeline`.
  - WebSocket `/api/stream`: state at 30 Hz, new log lines, timeline changes.
  - MJPEG `/api/cameras/{key}/stream?kind=color|depth`.
  - Interactive docs at `/docs`.
- Live arm state (`Aloha.start_monitor`).
  - A 100 Hz thread copies each driver's cached robot output (measured 0.05–0.08 ms per read) and re-reads joint modes at 2 Hz (1.5 ms network round trip).
  - An arm whose output timestamp stops advancing for 0.5 s is reported `lost`.
  - Every driver call goes through a per-arm lock, because the driver's thread safety is undocumented.
- Cameras: `myaloha/cameras.py` uses LeRobot's RealSenseCamera (color + depth). It measures real fps from frame timestamps and colours depth using the device's depth scale. `realsense_macos.py` moved into the package.
- `python -m myaloha panel`: preflight (en14 address, ping, each arm, each camera), then serve and open the browser. It re-runs itself under sudo for the cameras; Ctrl-C stops mods, then releases the hardware.
- `myaloha/sim.py`: simulated arms and cameras for `--sim` and the tests. `--absent` unplugs chosen arms or cameras.
- Mods:
  - `Mod.start()` claims the robot, then runs in a thread.
  - `Mod.params_from()` is shared by the CLI and the API.
  - `Mod.availability(robot)` says what each mod can use right now.
  - Each mod logs under its own name.
- Web:
  - `visitor_panel/` became `panel/visitor.html` + `visitor.json`, sharing `base.css`, `arm.js` and the meshes with the operator panel. Screenshots before and after are pixel-identical in both themes.
  - Pages builds `_site/` from `panel/` with `visitor.html` as `index.html`; the public URL is unchanged.
- Verified:
  - 16 tests (mods + API on the simulator).
  - The panel driven in Chrome against `--sim`: teleop run and stop from the UI, depth toggle, log.
  - On the real rig, read-only: all four arms connected, braked, live positions in the 3D view, clean release on Ctrl-C. No mod has run on hardware yet.

**2026-09-26: panel 01; repo merged into Abaka-Skills/iros26**

- Chose layout 01 "Stage". `panel/index.html` is the panel: rig full-screen, mod browser with live search, category chips and ⌘K; the selected mod's editable params and FSM (the running one with time per state); cameras; log with level/source filters.
- Removed the arms list: arm status is read off the 3D view (tags plus ghosted absent arms). I also dropped the camera list, since each tile carries its own status dot and fps.
- Click a camera tile to switch it between RGB and depth.
- The six-layout board was removed; it is in git history (commit "Add myaloha: …").
- Git:
  - This folder is now the repo for github.com/Abaka-Skills/iros26, on top of its history. The Cube Sandbox moved to `visitor_panel/`, and Pages deploys that folder, so the public URL is unchanged.
  - `thirdparty/lerobot_trossen` is a submodule.
  - `.gitignore` keeps out `.venv/`, `CLAUDE.md` (as iros26 did before), `_claude_tmp/` and `setup/camera_snapshots/`. Without the snapshots, camera tiles in a fresh clone show "no frame".

**2026-09-26: one environment; layouts revised**

- `lerobot_trossen` moved to `thirdparty/lerobot_trossen` and restored to upstream (our local edit was the `trossen-arm` pin, which now lives in the root `pyproject.toml`).
- One uv project at the root: `pyproject.toml` (project `myaloha`), with the thirdparty packages as workspace members, a single `.venv/` and `uv.lock`. The package set is identical to the old env, plus pytest in the `dev` group. Run things with `uv run ...` from ALOHA/, or `sudo .venv/bin/python` for RealSense. `uv run pytest` passes (8 tests).
- Python additions the panel needs:
  - `Mod.category`, for grouping in the browser;
  - timestamped `history` in `Mod.status()`, for time spent per state;
  - `parent` in `Mod.status()`, for the composition breadcrumb;
  - `Aloha.STATES` / `Aloha.state` (offline / idle / running) in `Aloha.status()`.
- All six layouts rebuilt around a mod browser (list, tree or ⌘K palette), the two-level FSM view and an always-visible log. The ten "planned" mods in `mock.js` are illustrative only.

**2026-09-26: mods v0 and panel layout board**

- `myaloha/robot.py`: the `Aloha` wrapper. It holds one LeRobot object per arm (follower or leader teleop). `connect()` only configures drivers and never moves an arm. An arm that fails to connect is recorded as absent, not raised, because the leaders are optional. `status()` reports each arm (connected, mode, position, error) plus the stack of running mods.
- `myaloha/mods/base.py`: the `Mod` interface, `Mod(robot, Params(...)).run()`, with `stop()` from another thread.
  - Each mod declares its own FSM: `STATES` says what the arms do in each state, `TRANSITIONS` lists the allowed moves.
  - `done`, `stopped` and `failed` are shared terminal states, reachable from any state.
  - Composition: a mod started with `parent=` may run while its parent holds the robot. Any other second mod is refused as busy.
- Mods:
  | mod | FSM | notes |
  |---|---|---|
  | `reset` | idle → checking → staging → zeroing → done | All connected arms go to all-zero. They pass through LeRobot's staged pose first (`via_staged`, as its `disconnect()` does). Absent arms are skipped. |
  | `teleop` | idle → pairing → aligning → following → releasing → done | Runs only the sides with both leader and follower, and fails only if no side has both. Followers first move slowly to the leader pose, then mirror via LeRobot `send_action(get_action())` at `loop_rate`. |
  | `instruction` | idle → entering → free → exiting → done | Followers go into gravity comp (external effort 0) and are sampled: latest pos/vel/eff/ext_eff plus the min/max range of each joint. `duration_s=0` runs until stopped; on exit the arms hold their pose. |
- `python -m myaloha status | run <mod> key=value ...`: CLI; Ctrl-C stops the mod cleanly.
- `tests/test_mods.py`: 8 tests against a fake driver (no hardware), covering absent leaders, teleop pairing, instruction stop, illegal transitions, busy refusal and nesting. Run with `uv run pytest`.
- `panel/layouts.html`: six layouts in the IROS26 soft-clay style, all showing the same mock status (`panel/assets/js/mock.js`, same shape as `Aloha.status()`). The mod catalog in it is dumped from the Python classes. The rig uses the real WidowX AI meshes and the camera tiles use the real snapshots. Tunables are in `panel/config.json`.

## Decisions and assumptions (check these)

- "Gimbal teleop" is read as joint-space leader→follower mirroring with the leader in gravity comp. If you meant something else (e.g. an end-effector/gimbal handle mapping), say so.
- "Instruction mode" is read as hand-guiding (kinesthetic teaching) of the followers with live measurement. Nothing is recorded to disk yet.
- The IP → arm mapping in `myaloha/robot.py` `DEFAULT_ARMS` is a **placeholder** (.2/.3 leaders, .4/.5 followers).
- The leaders mirror the follower layout (same x, facing each other) on a stand in front; the stand's depth and distance are guesses (`panel/config.json` → `rig`).
- Cameras are not in the `Aloha` wrapper yet. RealSense needs root on macOS, so adding them means the whole backend runs under sudo. That decision was left for later.

## Next

1. ~~Map left/right~~ done 2026-09-26: followers from the top-down camera (base yaw vs joint_0) and the wrist views; leaders inferred (teleop wasn't crossed). Leader L `.3`, R `.2`; follower L `.5`, R `.4`; wrist L …1803, R …2313. To confirm the leaders: in teleop, move the left leader and watch the left follower.
2. First powered runs, with someone at the rig: `reset` ran from the panel on 2026-09-26 13:59 (done, all four arms to zero). `teleop` (both pairs) ran at 14:00 and was stopped from the panel, then `reset` again, all clean. Still to do: `instruction`.
3. Cameras: all four D405s stream together at 30 fps under sudo (2026-09-26 14:04). Left: tune the depth colour range (`CameraRig.depth_range_mm`, now 70–1000 mm), 
4. Show the timeline (`/api/timeline`) in the panel, if wanted.

## Open questions

- Should `teleop` leave the leaders stiff (current: hold) or floating when it ends?
- What should `instruction` produce: live numbers only, or a saved trajectory or pose?
