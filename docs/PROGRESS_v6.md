# PROGRESS v6

Tracking from 2026-09-28 (after [`PROGRESS_v5.md`](PROGRESS_v5.md): reachability measured, handover
placed by it, pick and place as a retry loop, tracks living on the cameras' depth, MuJoCo physics in the
simulator, the camera relay). The simulator is ready; accurate cube tracking is being built in parallel
(a separate line of work). This round: a new mod, Cube Craft.

## Rules (carried over)

- The simulator is its own project; the backend never reads its ground truth (tests use it only to judge).
- Perception is grounded in the cameras only: nothing is inferred from the arms (a closed gripper is not a
  cube, an arm hanging over a track does not keep it alive).

## Plan

**Cube Craft** (`cube_craft`): visitors arrange the cubes in `visitor.html`, then the arms rebuild their
arrangement on the real table.

1. **The visitor designs a layout.** With the mod enabled, `visitor.html` opens on the cubes the cameras
   see now: their places and colours, as `locate_cubes` reports them. Only those cubes: their number and
   colours are fixed by the table. Visitors grab cubes and move them where they want them; the layout they
   leave is the target.
2. **Match the layout, not the moves.** The arms don't replay the visitor's moves, and a cube keeps no
   identity between the layout before and after: any cube of the right colour fills a target place. What
   counts is how close the final table comes to the target.
3. **Clear the canvas.** First, every cube in the region the target covers is picked and parked at the
   edge of the arms' reach (the reach map), so the canvas is empty before building starts.
4. **Build from the inside out.** Then, for each target place, a parked cube of its colour is picked and
   placed. The order is planned before the first move: the innermost cubes first, then outward. Outer
   cubes surround the inner ones; placed first, they crowd the way in to the inner places and the build
   turns chaotic. Each placement is a `pick_place` child, checked and retried like any other.

## Done

- The Cube Craft concept, written down (this plan).
- Simulator: the tabletop no longer flickers (the physics table's box was drawn at the same height as the
  textured top; it is collision-only now, and the edge is a visual box just below).
- pick_place sets a cube down where it meets something, not at a planned height: lowering stops when
  the vertical force at the tool (the joints' external efforts through the Jacobian) jumps by
  `CONTACT_N` (1.5 N) from its running median, and the arm lets go there. It goes no lower than just above
  the table, as before. Stacking on a cube no longer presses the stack down to table height. The retreat
  rises from where it let go (the travel height alone left the open fingers around a cube set on a stack).
  The simulator now reports efforts from its physics (the servo forces beyond gravity; jitter
  `effort_jitter`). Test: a cube placed onto another lands on it, and the one below doesn't move.
  On the rig: `CONTACT_N` and `effort_jitter` are guesses until the arm's external-effort noise is measured.
- The camera relay is installed on the rig (`sudo camrelay/install.sh`, 2026-09-28): it serves all four
  cameras and keeps 192.168.10.200 on en14. Fixed there: the once-a-second hot-plug check listed the cameras
  through librealsense every time, which fails ("failed to set power state") while they stream; it now
  lists them only when the RealSense devices on USB change (`usb_addresses()`).
- Fixed on the rig: `locate_cubes` failed every step or two with "cannot convert float NaN to integer".
  The wrist cameras' lens model (`inverse_brown_conrady`) is solved iteratively, and for a point far off
  to a camera's side (x/z beyond ~1.5) the iteration diverged to NaN; `cubes.evidence()` then rounded a
  NaN pixel. `coords._bc_inv` now leaves a point where it doesn't converge (still outside the image);
  inside the image nothing changes (0.0 px over the whole frame).
- pick_place `auto_target` (off by default): the mod keeps going until stopped. Each round it picks a random
  cube the cameras see (on the table, nothing stacked on it) and a random free place: on the table (the
  tracker's area, 3 cm inside), where either follower reaches straight down (the reach map), and at least
  `AUTO_GAP` (6 cm) from every cube, its own included. The first of up to 20 random pairs the planner can do is
  moved like any task (tries, recovery, handover). A round that gives up leaves the arm safe and another
  follows; 3 failed rounds in a row end the run. Test: two cubes, two rounds and a stop.
- Fixed on the rig: after a pick and lift the gripper let the cube go at once. The grasp check had called it
  not held ("something between the fingertips, but only 0-24% of it has a cube's colour"), and the recovery
  that follows a failed try opens the gripper. In the fingers' shadow, against the white table, the cubes read
  too dark or shifted to classify (a dark purple one averaged BGR 20, 21, 21). Every colour failure among the
  55 saved checks (`logs/pick_place/*_held.npz`) was a cube held. The check now takes the gripper (stopped at
  a cube's width) and the wrist camera's depth between the fingertips (`HELD_FILL`, 20%). Replayed on the
  saved checks: misses fill 0%, a cube held off-centre 31%, held as planned 50-97%. The same after a
  handover: the taker's wrist camera looks across the hall, with no table to balance colours by, and a
  bright red cube it held (94% filled) read as "0% of a cube's colour".
- Planning 9x faster (a handover took 31-47 s on the rig). All of it was IK: `_ik` tries its own starts (the
  seed, the reach map's solution, mid-range) and called the solver with `seeds=6` from each, and the solver
  runs every seed to the end (five random restarts, most of them 200 iterations that don't converge): up to
  18 descents per pose, all 18 for a pose with no solution. Now one descent per start (`IK_SEEDS`). On 15
  random tasks from the rig's calibration and cubes the plans are the same (same arm, handover or refusal);
  320 s -> 36 s in all: one arm ~1 s -> under 0.1 s, a handover ~23 s -> ~2.7 s, no plan 11-91 s -> 2-12 s.
- Arms reconnect on their own. An arm that dropped (power, cable) stayed lost until someone pressed
  connect. Now, alongside the monitor, every `RECONNECT_S` (3 s) an arm that is lost or absent is connected
  again once its controller answers: a 0.3 s TCP check on port 50001 first, so a powered-off arm costs no
  driver retries and no log flood. Then the dead session is released and the driver configured afresh
  (no motion). A failure that repeats is logged once. Leaders powered on later connect the same way.
  `tests/test_arms.py`: a follower unplugged and replugged comes back live; a leader absent at start is
  connected when it appears (both fail with the thread off).
- Fixed on the rig: a placed cube was dropped from mid-air. A false touchdown ("-7.3 N from -5.4 N, the tool
  40.9 mm above the table", a second into the descent) let go ~3 cm up: the real arm's efforts jump by ~2 N
  as it speeds up. A touch now counts only near the surface the cameras see under the target (the table or
  a stack's top, within `CONTACT_BAND` 15 mm); higher up it takes `CONTACT_FAR` (3x) the force (a cube the
  cameras missed). It has to last `CONTACT_HOLD` (2) readings, and none count for the first `CONTACT_WARMUP`
  (8, 0.4 s). The surface is measured when the task is planned (`self.support`): by the time the cube hangs
  over the target, the overhead camera reports it as a cube on the table there. pickandplay.html's default
  view is a little more oblique (68 deg). Both pages' default views are turned 180 deg about z: the
  operator panel's to azimuth 28 (+y toward the viewer), pickandplay.html's to 180 (-y toward the viewer). And in auto_target, a cube not found
  again after a recovery ends that round (`GaveUp`), not the run.
- Fixed: the cube could drop just after a handover. The taker went from the meeting point (tool horizontal)
  to above the target (tool down) in one joint move, and on the way the cube swung down to 25 mm (its bottom
  ~12 mm over the table) across ~19 cm of the table. Now the carry is a straight tool line, the tool turning
  evenly on the way (`_steps`, at most `TURN_SPEED`), so its height goes monotonically from the meeting point
  (at least 10 cm) to the travel height. The planner walks that line with IK before choosing a handover.
  `_line` turns the tool this way everywhere (elsewhere the ends already agree, so nothing changes). The
  handover test checks the carried cube stays above 40 mm.
- Panel log: hovering a line shows the whole record (date and time to the ms, level, full logger name, the
  message unclipped, and the traceback when there is one; the backend now sends `logger` and `exc`).
- **`iros26`: pick and place chosen on a page.** While the mod runs, the backend serves
  `/pickandplay.html` (404 otherwise; the operator panel shows a link to it meanwhile). The operator clicks
  a cube, then where it goes (a spot on the table, or a cube to stack it on), then "Pick and place"; the
  choice arrives through a new generic endpoint, `POST /api/mods/{name}/input` (`Mod.input`; 409 when not
  running or not waiting, 422 for bad input), and `iros26` carries it out with a `pick_place` child, then
  waits for the next, until stopped. A failed choice is listed on the page with its error and the next can
  follow. The page: the followers only (no leaders or their stand: `makeScene(..., {leaders: false})`),
  seen nearly from above (78°) from the far side of the table, cubes from `locate_cubes`, the reach
  regions, a frame on the chosen cube and a ghost cube where it goes (`scene.js`: `pick`, `setMarks`), a
  narrow steps card with the child's state while it moves, Stop and E-stop, and the operator panel's view
  controls (90° turns, home, top at 86°, front, free orbit; a drag in free view chooses nothing), and our
  watermark on the floor as on visitor.html (`watermark.js`, shared by both pages; beyond the rig on the side
  away from the camera, reading left to right). Test (simulated rig):
  page gated, bad and doubled inputs refused, a choice placed on its target, the page gone after stop.
  Checked in Chrome: two rounds chosen by clicks; one placed 7 mm from its target, one ended by
  `pick_place` losing the cube's track at the look pose (a tracking matter, not the page).
- **Placing sets the cube down instead of dropping it.** The place pose still put the tool point at
  `SIZE/2 + RELEASE_GAP` (from the old mid-height grasp) although the fingertips now hold the cube
  `GRASP_TIP` (8 mm) above its bottom: the arm stopped with the cube in the air and opened, a drop of
  about 9 mm in the simulator. Now the cube's bottom is aimed `SET_PRESS` (2 mm) below the surface, so it
  always touches down and the arm stops where it feels it rest (on the table or a stack), and the fingers
  open only `RELEASE_OPEN` (6 mm) wider than the cube, slowly, before the arm lifts (they opened to
  45 mm). `RELEASE_GAP` is gone; the auto_target free-place check uses the same place height. Test: the
  cube's bottom is on the table when the fingers let go, and it stays there, flat, not pushed. (The table
  height itself was already right: localize measures it, the bases 19.5 and 26.6 mm above it on the rig.)
- **A safety boundary: a hard limit on where planned tool points may be.** A rectangle on the table,
  each side a margin in from the table's edge (panel/config.json's table, as drawn): up (+y, the far
  edge), down (-y, the operator's), left (-x), right (+x); 3 cm each by default. `myaloha/safety.py`;
  the robot holds it (`robot.boundary`, in the status stream), launch loads the rig's own
  (`calib/.../boundary.yaml`, printed in the preflight), `GET/PUT /api/boundary` reads and sets it (422
  for a negative margin or one that leaves no table). pick_place keeps inside: a cube or a target outside
  is refused before anything moves; IK refuses any tool point outside (a failure that caused says "some
  poses would leave the safety boundary"); every joint move of a plan (from where the arm is, staged,
  look, above the cube, the handover, back to rest) is sampled along its path (`JOINT_CHECKS`), at
  planning and again right before it runs (so a boundary tightened mid-task stops the arm rather than
  cross it); a straight line may not end outside (straight up or down excepted: recovery lifts). Straight
  lines between points inside stay inside (the box is convex). auto_target's free places are inside.
  `/api/reach` is clipped to it. Teleop and reset are not bounded (they are the operator's). Panel: a
  small button right beside the E-stop opens an editor (four margins in cm around a to-scale table; the
  draft shows on the 3-D table as typed; Enter applies, Escape cancels); the boundary (the margin band
  shaded red, the box outlined) shows with the Reach annotation, and always on pickandplay.html. Checked
  in Chrome: margins applied and kept, the reach regions cut at the boundary. Tests: the box and its
  checks and file (`test_safety.py`), the API (`test_server.py`), a target beyond it refused (simulated).
- **pick_place plans around the cubes near it.** The fingers go down (open) and let go only where no
  cube is in their way: of the finger yaws square to the cube (least wrist turn first), the ones whose
  fingers, cut at each neighbour's top (they widen higher up: the gripper meshes, `finger_points`), keep
  `FINGER_CLEAR` (4 mm) plus the neighbour's sigma from it; failing that at `OPEN` (45 mm), at
  `OPEN_NARROW` (35 mm); failing that, "boxed in" before anything moves. The same for letting go at the
  target (the fingers open to `RELEASE_OPEN` past the cube). A target half on a cube (off its centre by
  more than `STACK_OFF`, 8 mm, but less than a cube) or overlapping one where it would rest is refused.
  Found on the way: `cv2.intersectConvexConvex` misses overlaps of a few mm² in metres (its tolerances);
  the check works in mm. Tests (simulated): cubes 1 cm off the faces the fingers would take: the grasp
  turns to the free faces and neither neighbour moves; a cube boxed in on four sides, and a target half
  on a cube, refused before any motion.
- **Tracking: a cube no longer drifts while it is grasped.** Measured on the simulated rig (noise 1.0)
  against its truth, per pick_place state, three tasks (one cube; a cube with a neighbour; stacking):
  the tracked cube was 24-26 mm off through descending and grasping, and up to 40 mm while the wrist
  looked. Causes and fixes (`cubes.py`):
  - The gripping wrist's camera, 12 cm over the cube, counted as a table-level (side) camera (`is_side`
    was height only), so its close top view was dropped and the other arm's wrist, half a metre away,
    placed the cube alone, 25 mm long. A side camera is now low *and* looking across (less than
    `SIDE_DOWN`, 30 deg, down).
  - That far wrist's observations (sigma 17-27 mm: pose, range bias) placed ghost cubes beside real ones
    and dragged resting tracks. An observation expected half a cube off or more (`VAGUE_SIGMA`) now joins
    a cube a precise camera sees but makes none alone; the track then lives on the depth evidence. The
    same camera may not say a cube is gone (`evidence` "empty"): off by more than its window, it saw the
    table beside a cube that was there. `near_sigma` is the one formula for both.
  - The overhead camera's partly hidden face (a gripper coming down over it) read 16 mm off at full
    confidence; its sigma now grows with the share of the face missing (`FACE_SEEN`).
  - Association counts another colour half a cube further: a misread neighbour 0.5 mm nearer took a
    track.
  After: through descending and grasping 3-4 mm (8-11 mm with the neighbour, the high camera half
  covered), looking 8-12 mm; carrying 2-4 mm at the frames' time. Tried and dropped: a static prior in
  the Kalman filter (a resting track barely follows vague detections): it cut the resting spikes but made
  the lift lag by up to 37 mm, and with the fusion fixes the spikes were gone anyway. Tests: the low wrist
  over a cube finds it; a vague observation makes no cube alone; a half-covered face is less sure; a
  track keeps its colour over a nearer misread; a far wrist can't drop a track.
- **Tracking under handover, measured and fixed** (`docs/TRACKING_REPORT.md`). 10 simulated tasks, each
  needing a handover, are scored against the simulator's truth frame by frame. Before the fixes: 7 of 10 tasks
  done, the held cube 12 mm off while the other arm took it, missing in about a fifth of the swing and carry
  frames, 245 ghost track-frames. After: 9 to 10 of 10 done, 7 mm while receiving, 2 to 6% missing, 35 ghosts,
  resting cubes 4.0 mm (from 5.1).
  - **Fusion:** vagueness judged on a group of views; far views in the air carry the height's error; views
    within 1.2 cubes in the air are one cube; far-only columns dropped where a close camera sees table; the far
    view's side-face parallax taken back ((S/2)·cot(elevation), independent of yaw); the low camera places a
    cube nothing above can see.
  - **Tracker:** a track that missed a frame is also tried as stopped where it was last seen; hidden tracks are
    kept only while resting on something.
  - **pick_place:** plans the pick again when the track moves 4 mm or more (after staging, and when the wrist
    can't measure the cube); recovery never closes the gripper.
  - 8 new unit tests. The remaining colour misreads come from the simulator drawing each cube's albedo again at
    every spawn (to settle: keep albedos across spawns, or calibrate the `sim` profile over several).
- **Auto on the pick-and-place page.** An Auto toggle (bottom left, remembered in the browser): with it on,
  60 s without a mouse event (move, press, wheel) starts an auto round, and 20 s after an auto round ends
  the next one; any mouse event goes back to the 60 s wait (a round under way goes on). The page sends
  `{auto: true}` to iros26, which runs pick_place with `auto_target`, `across` (a place no arm that reaches
  the cube reaches, so every round is a handover) and `rounds=1`; refused when iros26 runs without handover.
  Checked on the simulator in a browser (60 s, then 20 s after the round) and by a new test.
  - Seen: 2 of 4 auto rounds ended as failed after a good handover and placement. A target 8 cm from the
    right base: the cameras didn't see the placed cube there ("nearest 251 mm off"), and `_recover` then
    found no IK lifting the giver, already at rest.

## Open

- Cube Craft, still to settle:
  - Stacks: can a visitor stack cubes? Then the order is also bottom-up, not only inside-out.
  - Targets out of reach: refused by the page as the visitor drags (the reach map), or skipped by the mod?
  - Parking: what if the edge of the reach can't hold every cube?
  - Cubes already in place (or close): keep them instead of clearing them and bringing them back?
  - Hand-off: how a visitor says "done", and whether the page freezes while the arms build.
  - Which arm takes which cube; when a cube must cross the middle, a handover.
- Carried from v5:
  - Simulation batches on Ray (v5 plan 5): the environment is ready on the node; next, a job that runs
    many simulated pick-and-place tasks in parallel and reports success rates and failure kinds.
  - Tracking (in progress, a separate line of work): appearance for association (beyond colour votes),
    the static prior (a cube doesn't move unless a gripper is at it; a plain one lags the lift, see
    above), a carried cube (the overhead camera reports it at level 0 under the gripper: seen again
    2026-09-29 as the cube is set down, a second track under it for about a second, whose id the placed
    cube then keeps). The overhead camera's yaw of a lone cube can be 15 deg off (the wrist's look
    re-measures it before the grasp).
  - More specialised recovery steps as failures show up (a stalled grasp: lift a little and grasp again;
    a nudge for a placement slightly off).
