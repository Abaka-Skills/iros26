# PROGRESS

## Now

The panel (layout 01) runs on mock data. Next: the backend, so it shows live status and runs mods.

## Panel requirements (from review, 2026-09-26)

- There will be many mods, not three. The picker must scale: search, categories, ⌘K.
- The FSM needs a careful visual at two levels: the robot's major state (offline / idle / running) with its history, then the running mod's own machine with time spent per state.
- A log panel is always on screen.

## Done

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
- Leader stand placement in the 3D rig is a guess (`panel/config.json` → `rig.arms`).
- Cameras are not in the `Aloha` wrapper yet. RealSense needs root on macOS, so adding them means the whole backend runs under sudo. That decision was left for later.

## Next

1. Map the IPs: run `uv run python -m myaloha status`, move each arm by hand, and fill in `DEFAULT_ARMS`.
2. First hardware run: `reset` → `instruction` → `teleop` (one side), watching the FSM log.
3. Backend: a small HTTP server over `Aloha`. It needs to serve:
   - status polling;
   - run/stop a mod with params;
   - the mod catalog (name, category, params, STATES, TRANSITIONS);
   - a ring buffer of log records (a `logging` handler);
   - the robot's major-state timeline (one segment per state or mod run, with the result).

   Then point `panel/` at it, replacing `mock.js`. Run should send the dock's params.
4. Cameras into the wrapper (roles from CLAUDE.md: high, low, two wrists), and map which wrist is left or right.

## Open questions

- Should `teleop` leave the leaders stiff (current: hold) or floating when it ends?
- What should `instruction` produce: live numbers only, or a saved trajectory or pose?
