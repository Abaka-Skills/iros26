# PROGRESS v2

Tracking from 2026-09-26 onward (earlier work: `PROGRESS.md`, frozen).
The simulated rig (`simulator/`, `--simulator`) is tracked in [`PROGRESS_v3.md`](PROGRESS_v3.md).

## Design (settled in the grilling session, 2026-09-26)

- **localize**: the new name of `establish_coordinates`. Its result is still "coordinates":
  `calib/coordinates/`, `/api/coordinates`. It registers the fixed cameras (High, Low) at the end of every run.
- **No feature reuse across runs or mods.** SIFT features live only inside the localize run that
  captured them; the results (poses) persist.
  - "Re-solve" and "register High again" are removed.
  - Saved frames in `logs/` are for debugging only.
  - An AprilTag sheet on the table gives SIFT texture; the operator always provides one.
- **Topics** are perception processes managed by the backend.
  - Each runs as a subprocess on its own localhost port, as the desktop user (not root).
  - Endpoints: `/health`, `/latest`, `/stream` (WS).
  - They declare their dependencies (`DEPENDS`) and requirements (`REQUIRES`, same `Need` as mods).
  - Only on demand; no automatic restart (a dead topic shows as `crashed`); killed when the backend exits; logs go to the shared log.
  - Starting a topic whose dependencies are stopped returns 409 with `missing`, and the panel asks whether to start them too. Stopping a topic with running dependents returns 409 with `dependents`, and the panel asks the same way.
- **MCP**: one server at `http://127.0.0.1:8765/mcp`, mounted in the backend.
  - Mods are the "do" side: `<mod>_run`, generated from each mod's Params (type, default and help text become the schema). It returns at once; `mod_status` and `robot_status` report progress.
  - `stop` and `estop` are available to agents. Releasing the e-stop is panel/REST only.
  - Topics are the "read" side only: `<topic>_latest` and `topics_list`. Starting and stopping topics is REST/panel only.
  - Runs started from MCP are tagged `source: agent`.
- **cam_params** (topic): intrinsics and live pose in W of every registered camera.
  - Fixed cameras (High, Low): the pose from the last localize run.
  - Wrist cameras: T_W_mount · FK(joints) · T_ee_cam.
- **locate_cubes** (topic; depends on cam_params): 2.5 cm cubes in 6 colours.
  - The six colours are red, orange, yellow, green (teal), blue, and purple (dark navy on camera).
  - Output per cube: {color, x, y, z, level, level_measured, yaw, visible, confidence, seen_by}.
  - Hidden cubes under a stack are reported with color null.
  - It keeps no memory between frames.

## Done

**2026-09-26**

- Rename: the mod `localize` is `myaloha/mods/localize.py`. The re-solve and re-register endpoints and code are removed; the panel's Table button requires High to be registered by the last run.
- Backend additions:
  - `/api/cameras/{key}/optics` and `/api/cameras/{key}/raw` (npz with colour, aligned depth in metres, K, lens, robot-clock stamp).
  - Arm readings carry `t` (perf_counter, the same clock as the frame stamps).
  - `last_run`; the mod `source`.
- Topics framework:
  - `myaloha/topics/`: `base.py` (Topic, Subscription, the process runner) and `manager.py` (TopicManager).
  - `/api/topics` with start/stop/restart.
  - Tested live: cam_params runs as the user at about 28 Hz; its log lines reach the panel.
- MCP: `myaloha/mcp_server.py`, served at `/mcp`. Every mod param now has help text; the tool schemas were checked.
- `cam_params` topic: uses T_W_mount because the arm's sag depends on the pose.
  - With T_W_base (which holds the sag at the survey poses), the table seen by the wrists at rest was off by 4 deg / 30 mm; with T_W_mount it is off by 0.4-0.6 deg / 5-7 mm.
- `locate_cubes` topic and `myaloha/cubes.py`:
  - Colours are normalised to each camera's own table colour.
  - High gives position and colour; its depth (about 2 cm noise at 1 m) can't resolve levels. Wrist and Low views give levels and side colours.
  - Wrist poses are refined by the table plane in their own depth.
  - First offline run on live frames (High and wrists): about 30 cubes with the right colours; about 200 ms per pass.
- `localize`:
  - Fewer than 30 agreeing matches is now accepted down to 20, but only if split-half < 5 mm / 1 deg and rms < 5 mm.
  - Run with the tag sheet: 304 inliers, 2.7 mm rms, bases 932.9 mm apart. High registered (76/197 matches, 0.88 px).
- `coords.fit_plane`: thin SVD (the full one built an N×N matrix: 90 ms → 12 ms).
- Arms that drop off the network are now reported lost: `lost` counts from the last fresh output, not from the last successful read. "Connect arms" retries lost arms, and a lost leader shows as offline.
- Low camera registration: PnP fails at its grazing view (6/113 matches), so `register_fixed` now falls back to a rectified top-down view of the table (from Low's own depth) with a 2-D fit.
  - Offline, on the 17:18 run's saved Low frame: 51/137 matches, 2.6 mm rms in the plane. Cubes seen by High project onto the cubes in Low's image.
  - Stored only by a `localize` run; see Now.
- Panel checked in Chrome:
  - The Topics section, both dependency dialogs, and the overlays (cubes, frusta) work.
  - Overlays draw on the status frame, not per message (a 30 Hz stream starved the page).
  - The topic list no longer redraws on every rate change (the redraws swallowed clicks).
- MCP checked with a real client over HTTP. Refusals carry their reason (ToolError).
- `locate_cubes` levels come from each blob's top edge: the blob's highest points are its cube's top, whether the camera sees the top face or only a side.
- Tests: 42 pass: `test_cubes`, `test_mcp`, `test_topics`, and the grazing-camera registration.

**2026-09-26 (later): camera glyphs; cube tracking**

- Cameras in the 3-D view:
  - Each camera is now a small frustum glyph at its pose: 6 cm deep, with a tinted image plane, an up tick, depth-tested, and labelled.
  - This replaces full-length pyramids drawn over everything. High's pyramid (1 m up) and Low's (looking at the horizon) cut across the whole scene.
  - Footprints are dropped: High sees the whole room, and the Table button already shows the check that matters.
- `locate_cubes` is tracked over time (`cubes.Tracker`):
  - A cube shows after 2 detections and is held up to 1 s after its last one (`held`).
  - Its position is smoothed, and its colour is voted over the last 1 s. This relaxes the earlier "no memory" decision, as the user asked.
  - Each cube has a stable `track` id and `sigma` (expected position error).
- Sources of blinking, found by logging track births and deaths on a static table:
  - Wrist positions read 25-40 mm long along the viewing ray at 0.5-0.6 m (D405 range error), and oblique top faces were taken for sides and pushed half a cube. Fixes: range error is now part of each wrist observation's sigma; a same-colour observation merges within a sigma-sized gate (at most 1.6 cubes); an oblique top face now counts as a top.
  - One overhead depth frame puts a cube top anywhere within about ±0.1 m, and the old ±3 cm window dropped real cubes. The window is now -0.15 to +0.25 m (the floor is 0.8 m down).
  - Colour: white balance is now per-channel gains, not additive Lab shifts (those made the black arms purple in Wrist L). The tag sheet's blue-grey ink gets its own non-cube colour.
  - The tracker matches nearest pairs first, drops duplicates of a matched cube, and starts tracks only from detections with confidence ≥ 0.5.
- Result on the live table: 5 track changes in 60 frames (20 s), down from 1-2 per frame. The one left is a wrist-only duplicate 42 mm from High's cube.
- Tests: 44 pass.

**2026-09-26 (later): the Low camera in locate_cubes**

- Low registered for run 20260926_171821 from that run's own frames: its wrist views and the Low frame taken at the same time. The report is marked "registered after the run".
  - Rectified view: 51/137 matches, 2.6 mm rms in the plane.
  - The "depth_scale" it reports (1.024) comes from the camera being 33 mm above the plane, not from depth reading bias, so it is not applied to Low's depth.
- Fixes needed so Low's views help rather than hurt:
  - A camera below a cube's top can't see its top face: Low's top-edge points had been taken as top faces, placing cubes half a cube toward it.
  - A camera at table level (below 12 cm) gets levels from each point's height band, ignoring points within 3 mm of a boundary. The top-edge rule stays for the wrists.
  - No white balance when the table is seen at under 15 deg: it mirrors the room light (Low's table read bluish-white, which turned blue cubes purple).
  - Colour masks are closed (5 px) before splitting into blobs: glare split Low's dark navy faces into specks below the size limit.
  - Merge gate across colours: a stack's lower cubes differ from its top. Different colour: up to 0.8 cube; same colour: up to 1.6.
- Offline on live frames of all four cameras (34-38 cubes, 8-14 seen by Low), the 3-high stack reads blue / purple / green (bottom to top). The middle cube comes out purple in about 1 frame in 4 and hidden in the others; the tracker's 1 s colour vote should hold it purple. Not yet checked live: the backend restart was waiting on the sudo password.

**2026-09-26 (later): Low as a probe; cameras resume on their own**

- Cameras resume on their own:
  - A camera down for 3 s is reopened every 3 s until it comes back; this also covers one plugged in after start.
  - `capture_realsense()` captures only devices it doesn't hold yet (a replugged camera has a new USB address).
  - Also `POST /api/cameras/connect`, and the panel's button is now "Reconnect" (arms and cameras).
  - Tested with a fake camera that stops (`test_cameras.py`). On the rig, Low dropped once (the read loop died after repeated failures); a backend restart brought it back before resume existed.
- Low's blobs replaced by probes. With about 1 cm position error, a camera at table level can't tell a stack's base from the cube touching it: live, its own observations raised churn and mislabelled stack bases. Now:
  - Low only answers, for each column the other cameras found and each level: project the face that cube would turn to it and compare depths. A match is a cube (colour read off it), seeing past it is an empty level, something nearer hides it.
  - Levels come from these answers as well as from the wrists.
- Live, with all four cameras, on the current table:
  - The 3-high stack reads blue / purple / green (correct) in 36-41 of 44-48 frames.
  - A red-on-navy stack reads hidden / red: Low's view of its base is blocked.
  - About 2.4 Hz.
- Churn is 22 track changes in 45 frames, still above the 5/60 before Low. The main source: from above, High sees a stack's lower cube as a sliver of side (parallax) and takes it for another cube beside the stack.
- The cube area is ±0.37 m from the bases' line (was ±0.45): the rails at the table's front edge flickered as cubes.
- Tests: 46 pass (`test_cameras`; the table-level camera stack case).

**2026-09-26 (later): cubes never overlap; camera resume made safe**

- No two cubes overlap (`cubes.resolve_overlaps`, `OVERLAP_IOU` = 0.2):
  - At one level, footprints (rotated 2.5 cm squares, `cubes.iou`) overlapping beyond the limit are one cube. Of two visible cubes, the more reliable one is kept (visible, seen now, confidence, sigma).
  - A hidden cube under a stack that overlaps a visible one takes its colour at the stack's position: from above, a stacked cube's side can look like a cube beside it.
  - Applied after fusion and again after tracking (a held track can overlap a new one).
  - Live: the worst overlap over 15 frames was IoU 0.200 (the limit).
- Tracker:
  - A track follows a level change of one (a stack read from a new angle).
  - A detection with sigma > 1 cm needs 3 sightings, not 2.
- Camera resume, fixed after it stalled the rig. A failed RealSense open blocks for seconds holding the GIL, which stalls every thread: arm monitor, API, e-stop. The 3 s retry loop took all cameras to 1.6 fps and the API to 3-10 s.
  - Now: one try when a camera goes down, then only when the RealSense devices on USB change (`realsense_macos.usb_addresses()`, a libusb list, cheap), or at most once a minute where USB can't be watched.
  - Tested (`test_cameras.py`).
- Live, all four cameras at about 30 fps, 2.1-2.4 Hz:
  - The 3-high stack reads blue / purple / green in about 90% of frames; the red stack reads hidden base / red.
  - Churn is about 29 changes in 40 frames, nearly all in one tight cluster at (0.10-0.12, 0.03-0.06) m: a navy-on-blue stack with touching neighbours, whose reading flips between frames. Elsewhere, stable.
- Tests: 49 pass.

**2026-09-26 (later): cleanup pass (/simplify)**

- Reuse:
  - coords: `table_frame` is used by both fixed-camera registrations, and they share `_plane_fit_to_W` and `_fixed_camera`.
  - cubes: uses `apply_T`/`inv_T`.
  - One arm-model loader (`localize.load_arm_model`) for localize and cam_params.
  - Mods and topics share one requirement check (`mods.base.evaluate`).
  - The catalogs are `Mod.catalog()` / `Topic.catalog()`, for REST and MCP both.
  - One USB device walk in realsense_macos (one libusb context; captures of unplugged cameras are released).
- Speed: a `locate` pass takes about 76 ms with the topic's thread pool (about 250 ms before).
  - The classifier is float32 matrix algebra over precomputed prototypes.
  - Cameras are processed in parallel.
  - `probe` projects all four levels of a column at once.
  - `capture(n=1)` skips the median and the trailing sleep.
  - Wrist frames aren't fetched while their arm moves.
  - The panel rebuilds overlays only when they change.
- Simpler:
  - `capture` returns the frame time; `capture_stamped` is gone.
  - In fusion, a camera can't join a column where it already has that level. This replaces the post-hoc "one observation per camera per level" filter, which could drop a real cube.
  - Knobs nobody set became constants (`MERGE_RADIUS`, `OVERLAP_IOU`), and the tracker copies fields by one key list.
- `Arm.lost` requires polling without new output, so arms not yet monitored (the launcher's second connect) aren't treated as lost.
- MCP catches only refusals: bad params, requirements, busy, e-stop. A bug is no longer reported as "not started".

**2026-09-26 (evening): relaunch on the cleaned-up code**

- About a minute after the restart, both followers lost their UDP stream (the controllers logged 3178 and 3380 of 5000 messages lost), and both controllers then hung on TCP ("Resource temporarily unavailable") while still answering ping.
  - "Connect arms" (now able to release lost arms) brought the right one back once; later both hung.
  - It is not USB contention: with no cameras open, a read-only connect failed the same way.
  - Power-cycling both follower controllers fixed it.
  - USB layout, worth fixing anyway: the arm Ethernet adapter (AX88179B, en14) shares a USB 3 hub with a RealSense, one RealSense is on a USB 2 hub, and one USB controller is unused.
- Live, everything up (followers, 4 cameras at 30 fps, topics at 3.0 / 28 Hz, MCP):
  - The 3-high stack reads blue / purple / green in 61 of 61 frames once settled; in the first ~20 s after start the base read purple.
  - No overlapping cubes (worst IoU 0.13).
  - Churn is about 38 changes in 58 frames, all in the tight cluster at (0.10-0.12, 0.03-0.06) m.

## Now

- The ambiguous cluster above: add hysteresis per column (keep the current reading until another one wins for several frames), or look closer with a wrist.
- A fresh `localize` run would register Low from scratch (optional; about 2 minutes of arm motion).

## Open

- Leader L/R mapping still inferred.
- The `instruction` mod has not run on hardware.
