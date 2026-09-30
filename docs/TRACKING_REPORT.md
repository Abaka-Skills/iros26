# Cube tracking under handover: 10 simulated runs

2026-09-29. What `locate_cubes` gets wrong while `pick_place` moves a cube from one arm to the other, measured
against the simulator's ground truth. Written after the fixes of the same day (PROGRESS_v6: the low wrist, vague
observations, the partly hidden face, colour in association).

## Summary

- **Every started task placed its cube:** 7 of 10 runs completed, each through a handover, 1.6 to 5.3 mm from
  the target. There were no retries, and no neighbouring cube was moved.
- **Tracking causes the other 3 runs.** They never started: the cube was reported in the wrong colour (1 run),
  or not at all because the resting arm hid it (2 runs).
- **Up to the lift, tracking is good:** the moved cube is 3 mm off while descending and grasping (p90 4 mm).
- **It degrades once the cube is in the air, from the handover on:**
  - While the other arm takes the cube, it is 12 mm off (p90 17 mm).
  - It is missing (no track within 2 cm) in 24% of the frames of the swing to the handover point, and in 19%
    of the carry.
  - Ghost cubes appear, mostly a copy of the held cube placed on the table 6 to 17 cm away.
- **Most of the handover damage has one mechanism:** when no close camera can place the held cube in the air,
  the overhead camera's view of it is put on the table. A rule added earlier the same day makes this more
  likely (finding 1).
- **Calibration is not what makes resting cubes inaccurate.** A stale calibration from another simulator seed
  gives 7.6 mm mean error on a 6-cube scene; a fresh one gives 7.2 mm. The resting error is a steady bias
  toward the overhead camera (finding 6).

## Setup

- **Rig:** the simulator (seed 11, noise 1.0: calibration, sensor and colour errors on), calibrated by a fresh
  localize run.
- **Runs:** 10 scenes. In each, one cube only one follower can reach; a target only the other can reach (both
  chosen with the reach map, so every task needs a handover); two resting distractor cubes at least 6 cm away.
  The givers alternate: the left arm in even runs, the right in odd.
- **Task:** `pick_place` with its defaults (speed 0.05 m/s, handover on, 3 tries).
- **Measurements:**
  - *Frame time:* a second `locate_cubes` process, on the same cameras, logged every step's detections and
    tracks. They are compared with the truth interpolated to the frames' capture time. This is the tracker's
    own accuracy.
  - *Live:* `/latest`, polled at 5 Hz, compared with the truth at poll time. This adds the output's latency
    (about 0.3 s), which is what a mod sees.
  - A track counts as a cube when it is within 2 cm of it in 3-D. A visible track 2 cm from every cube is a
    ghost.

## The moved cube, state by state

Frame time, over the 7 completed runs. The xy error counts only frames with a matching track; "missing" is the
share of frames without one.

| state | frames | xy mean / p90 / max (mm) | \|z\| max (mm) | missing | live xy mean / max (mm) |
|---|---|---|---|---|---|
| planning | 23 | 7.5 / 12.9 / 13.1 | 0 | 0% | 6.8 / 13.0 |
| staging | 43 | 5.3 / 8.6 / 11.3 | 0 | 0% | 5.5 / 12.6 |
| looking | 102 | 5.6 / 8.7 / 14.5 | 0 | 0% | 5.5 / 14.5 |
| approaching | 63 | 3.4 / 5.6 / 10.0 | 0 | 0% | 3.8 / 10.0 |
| descending | 83 | 2.7 / 3.7 / 7.3 | 0 | 0% | 2.7 / 7.3 |
| grasping | 31 | 2.7 / 3.9 / 7.3 | 0.4 | 0% | 2.9 / 7.3 |
| lifting | 85 | 2.5 / 4.2 / 4.9 | 7.8 | 0% | 2.5 / 4.7 |
| **handing_over** | 85 | 7.8 / 13.2 / 17.9 | 10.4 | **24%** | 13.3 / 29.4 |
| **receiving** | 282 | **11.8 / 16.7** / 17.2 | 8.6 | 1% | 11.8 / 17.2 |
| exchanging | 312 | 5.9 / 7.9 / 15.7 | 8.6 | 0% | 5.9 / 16.0 |
| **moving** (the carry) | 169 | 5.0 / 9.4 / 17.3 | 18.9 | **19%** | 9.0 / 25.4 |
| lowering | 73 | 3.8 / 7.1 / 9.3 | 8.2 | 0% | 3.7 / 9.4 |
| releasing | 28 | 2.8 / 3.8 / 4.2 | 0.4 | 0% | 2.8 / 4.3 |
| retreating | 78 | 2.6 / 3.5 / 3.8 | 0 | 0% | 2.6 / 3.8 |
| returning | 141 | 5.1 / 9.4 / 10.2 | 9.0 | 0% | 5.1 / 13.0 |

Across the whole scene (all cubes, every frame):
- **Resting distractors:** 5.1 mm mean error (p90 6.4, max 17.0; 3194 frames).
- **Ghosts:** 245 ghost track-frames over the 7 runs, all from lifting on:
  - receiving 108, exchanging 62, moving 52, handing_over 17, lowering 5, lifting 1;
  - 117 of them within 5 cm of the moved cube (a duplicate of it), the rest 6 to 17 cm away (a copy on the
    table).
- **Colour:** wrong in 180 of 4737 matched frames (3.8%), nearly all purple read as blue.
- **Track ids:** in the live output, the track nearest the moved cube changed 5 to 22 times per run, as its
  duplicates came and went.

## Findings, by impact

### 1. A cube held in the air is put on the table, or lost

- **What happens:**
  - While the other arm takes the cube (held about 17 cm up), a yellow or orange copy of it sits on the table,
    6 to 17 cm away, for 30 to 38 frames (over 10 s) in 3 of the 7 runs and 8 frames in a fourth. It is seen by
    the overhead camera alone.
  - While the cube swings to the handover point, or is carried to the target, 11 to 18% of frames have no
    detection within 5 cm of it at all.
- **Evidence (frames captured mid-receiving and replayed per camera, truth (0.006, 0.081, 0.173)):**
  - The low camera places the cube at (0.002, 0.075, 0.172): 7 mm off in xy, 1 mm in height.
  - The giver's wrist camera places it at (−0.001, 0.078, 0.172): 8 mm off in xy. The two agree within 4 mm.
    (An earlier version of this report said 2 mm for the low camera; that was a miscalculation.)
  - Both are side views, with σ about 14 mm, above `VAGUE_SIGMA` (12.5 mm). Fusion drops the in-air group they
    form, because it keeps a group only when one of its observations alone is precise.
  - The overhead camera's view of the cube then has no in-air cube to join. It is placed on the table at
    (0.004, 0.141): the ghost, 60 mm off. The fused result has no cube in the air.
- **Causes:**
  - The vague rule (PROGRESS_v6, the same day) judges each observation alone, not the group. Two independent
    cameras agreeing within 6 mm fuse to about 10 mm, which is precise enough.
  - A far view is put on the table whenever it has no in-air cube to join, although its own depth says the face
    is 17 cm up. At 1 m the overhead depth is off by 1 to 2 cm, which is plenty to tell 17 cm from 0.
- **Fixes:**
  - Judge vagueness on the fused group, after far views have joined it.
  - A far view whose depth puts it clearly above the tallest stack is an in-air observation, never a table
    column.

### 2. The held cube is 12 mm off while the other arm takes it

- Receiving is long and still: 282 frames, 11.8 mm mean error.
- The overhead camera's view joins the in-air group at σ 3 mm and dominates the fused position.
- In the air, though, its xy depends on the height it is placed at, and that height comes from the close views
  (σ 10 to 14 mm), shifted by the tangent of the viewing angle.
- **Fix:** when a far view is placed at an in-air cube's height, its σ should include that height's
  uncertainty times the tangent of its viewing angle.

### 3. A second, wrongly coloured copy of the held cube

- **What happens:** in 2 runs, an in-air copy in another colour, about 24 mm below the real cube, for up to 45
  frames (red read as orange, blue read as purple).
- **Cause:** the low camera sees the held cube from below the gripper. The fingers hide its top edge, so its
  height reads low, and the light from below shifts the colour. With another colour and 24 mm apart
  vertically, it is kept as a separate cube.
- **Fix:** in the air, two cubes closer than a cube (centres, in 3-D) are one. A gripper holds one cube.

### 4. Purple read as blue

- 3.8% of matched frames have the wrong colour, almost all purple read as blue.
- Run 2 never started because of it: its purple cube was blue in every camera for the whole 7 s wait. Respawned
  later with the arms in another pose, the same cube read purple.
- **Fix:** recalibrate the colour profile on the simulator (`setup/calibrate_colors.py`), with purple and blue
  under the arms' shadows. On the rig, check the same pair.

### 5. A blind spot behind the resting arms

- **What happens:** runs 4 and 8 never started. Their cubes, at about (−0.30, +0.05), were not reported for
  about 10 s after they appeared, until the left arm moved.
- **Cause:** at rest (the zero pose) the arm lies along y = 0 above the table. From the overhead camera, in front
  of the table, it covers the cubes just behind it. The low camera does see them, but by design it only answers
  about cubes the others found (it is too coarse to tell a stack's base from a cube touching it).
- **Fixes:** either let the low camera create a cube where no other camera can see (with its σ, so a better view
  replaces it), or rest the arms somewhere that hides less of the table.

### 6. Resting cubes are biased toward the overhead camera

- The distractors are 5.1 mm off on average, 3.7 mm of it along the direction to the overhead camera, about
  equally everywhere on the table (+4.1, +3.5 and +3.3 mm from the front to the back).
- **Cause:** the camera sees the cube's front face at a slant. Placing those pixels on the top face's plane
  drags the centre toward the camera.
- **Not the calibration:** a stale calibration (another seed) and a fresh one give 7.6 and 7.2 mm on the same
  6-cube scene.
- **Fix:** use only the top face, or subtract the parallax the known geometry predicts.

### 7. Fast moves outrun the tracker

- The giver's swing to the handover point is a joint move: the cube averages 11 cm/s and moves faster at
  times.
- During it, the track's median error is 9 mm and its p90 92 mm (lost).
- In 9 of the 20 missing frames there, a detection was within 2 cm, but the track was elsewhere.
- **Cause:** the filter's `ACCEL` (0.15 m/s²) assumes a gripper nudging a cube, not an arm swinging one; the gate
  follows from it.
- **Fixes:** a higher process noise for cubes in the air, or a gate that widens with the time since a track was
  last seen.

### 8. Smaller things

- **The taker's wrist camera contributes nothing while receiving.** Looking horizontally at the giver, it sees no
  table to white-balance on ("no table in view", or no observations, in 35 of 40 frames).
- **Lifting and lowering through a level's height.** A cube passing 37.5 mm reads as resting at level 1 for a
  frame or two: short duplicates about 25 mm off, height errors up to 8 to 19 mm.
- **An arm dropped once.** Under the extra load of this measurement (a second `locate_cubes` process), a
  follower and two cameras dropped for a moment. They reconnected by themselves within seconds; the task
  planned then failed with "a handover needs both followers".

## What to do first

1. **The in-air cube (findings 1 to 3).** Judge vagueness per group, after far views join. A far view with an
   in-air depth is never a table column. Give far views placed in the air the height's uncertainty. Merge in-air
   cubes closer than a cube. These should remove most ghosts and missing frames during the handover and the
   carry.
2. **Colour (4):** recalibrate purple and blue.
3. **The blind spot (5):** decide between the low camera creating cubes and a different rest pose.
4. **The resting bias (6):** about 4 mm everywhere, worth a correction once the above is done.
5. **Fast moves (7):** raise the process noise for cubes in the air.

After each, measure the same way. The scripts (in `_claude_tmp/`, local, not in the repo) run the 10 scenes, log
the frames and score them as above.

## Fixes, 2026-09-29: measured the same way

Same simulator (seed 11), calibration and 10 scenes as above. Tracks from the previous scene are cleared before
each scene: the benchmark respawns cubes, which a real table never does, and the old spots, hidden by an arm at
rest, were kept as occluded (correctly) and counted as ghosts. "Before" is the run above; "after" is the last run
with every fix.

| | before | after |
|---|---|---|
| tasks completed (all by handover) | 7 of 10 | 9 of 10 (10 of 10 in an earlier run the same day) |
| moved cube while the other arm takes it (receiving) | 11.8 mm | 7.2 mm |
| missing while handing over / carrying | 24% / 19% | 6% / 2% |
| descending and grasping | 2.7 mm | 2.5 / 2.8 mm |
| resting distractors | 5.1 mm (3.7 toward the overhead camera) | 4.0 mm |
| ghost track-frames (near the moved cube) | 245 (117) | 35 (26) |

What changed (`myaloha/cubes.py`, unless named):

- **Findings 1 and 3, the cube in the air:**
  - Vagueness is judged on a group of views together, after far views have joined it. Two side views that agree
    now place the held cube.
  - A far view joining a cube in the air takes on the height's uncertainty (`σz · tan` of its viewing angle).
  - Views of the air closer than `AIR_APART` (1.2 cubes) are one cube, which removes the coloured copy.
  - Every view counts by its σ, not the "top" views alone.
  - A column only far views see is dropped when a close camera sees the table there.
- **Finding 6, the resting bias:** a far view's blob is its top face and the sides facing the camera, swept
  along the viewing ray. Its centroid is therefore off by (S/2)·cot(elevation) toward the camera, whatever the
  cube's yaw, and the fix takes that back.
  - Measured with an exact camera, the bias was 4.4 mm and is now under 1.5 mm (unit test).
- **Finding 5, the blind spot:** a cube a table-level camera finds, where no camera able to place it can see (out
  of frame, behind something nearer, or too far away to be precise), becomes a cube as that camera places it.
  Runs 4 and 8 start now.
  - Its position is vague (13 mm here). `pick_place` therefore plans the pick again when the track moves by
    `REPLAN_SHIFT` (4 mm) or more: after staging (the arm off its rest pose, the overhead camera sees the cube)
    and when the wrist camera can't measure it.
  - Recovery never closes the gripper. Fingers that stopped wide, on top of the cube, were told to "open" to
    45 mm, which pressed on it and raised an error.
- **Tracker:**
  - A track that missed the last frame is also tried as stopped where it was last seen. A jittery pair of
    detections (a partial view 12 mm off) had given a resting track about 4 cm/s. One second later its
    prediction was just past the 50 mm gate, and a duplicate was born; after the pick, it stayed as a ghost.
  - A hidden track is kept on "occluded" evidence only if it rests on something (the table, or a track one level
    under it). A cube in the air is in a gripper and moves with it; a lifted cube passing 37.5 mm reads as
    resting at level 1.
- **Colour (finding 4) is no longer a tracking problem.** After the fixes above, colour errors fell from 180 to
  9–47 per 10 runs, spread over cameras and runs, whenever no purple cube came out bluish. What remains is one
  mechanism: the simulator draws each cube's albedo again at every spawn (`errors.albedo`, 3 Lab units). A draw
  about 3σ redder puts a purple cube half on the blue side of the `sim` profile:
  - Every top view splits its pixels about evenly, and the cube reads blue for the whole run.
  - In the last run, one such draw made 173 of the 209 colour errors, and it is why run 2 didn't start there.
  - Physical cubes keep their colour, and the rig's profile is calibrated on the very cubes in use. Either the
    simulator keeps each cube's albedo across spawns, or the `sim` profile is calibrated over several spawns.

Still open:
- **Fast moves (7).** The swing to the handover point outruns the tracker: a new track is born, and the live
  error is about 12 mm.
- **Live latency.** The output is about 0.3 s behind, twice the frame-time error while carrying.
- **The rig drops under load.** Under this benchmark's load, the simulated rig still drops an arm or a camera
  now and then (seen once: a task planned in that window failed with "a handover needs both followers").
