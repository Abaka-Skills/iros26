# Cube Sandbox

A WebGL sandbox for building structures out of 2.5 cm cubes on a robot workbench, with two
Trossen **WidowX AI** follower arms (the Trossen AI Stationary pair) parked at the table edges.

**Live:** https://abaka-skills.github.io/iros26/

Style exploration that preceded the build: [`styleboard.html`](styleboard.html)

## Running locally

```sh
./run.sh            # serves on :8000 and opens the page
```

A server is required — browsers block `file://` pages from reading `config.json`.

## Controls

| | |
|---|---|
| Left click | place a cube |
| `Alt` + left / `0` | grab a placed cube and move it |
| `X` / right click | delete a cube (the stack above drops) |
| `Shift` + drag | keep placing while dragging |
| `Q` `E` | rotate the view 90°, animated |
| `R` | top view / eye level |
| `H` | reset the view |
| Wheel / middle drag | zoom / pan |
| `1`–`6` | pick a colour |

Cubes obey gravity: a block always lands on the lowest free cell of the column you point at,
so nothing floats. **Save** writes the layout to JSON; **Reset** clears the table (click twice).
**ALOHA** shows or hides the arms and reframes the camera accordingly.

## Configuration

Everything tunable lives in [`config.json`](config.json) — no build step, no rebuild:

- `grid` — table size in cells and `cellMeters` (0.025, i.e. 2.5 cm cubes). With `board.margin*`
  the plate measures 43.8 × 30 in; the 30 in side is the measured depth of the real bench
- `cubes` — the six colours (palette swatches and 3D blocks share these values)
- `style`, `themes` — material, lighting and the dark/light palettes
- `camera` — rotation step, elevations, `fitPadding` (framing is computed from the table size)
- `interaction` — spring stiffness, hover height, cursor lead
- `robots` — rest `pose`, `zFrac`, `baseCells` (the grid is cut away under each base) and
  `gapMeters`: the measured clear gap between the two robot bases (0.8382 m = 33 in). Arm
  spacing and scale are derived from it and from `cellMeters`, so both follow the real rig

## Attribution

Arm meshes and kinematics: [TrossenRobotics/trossen_arm_description](https://github.com/TrossenRobotics/trossen_arm_description),
BSD-3-Clause — see [`assets/meshes/wxai/LICENSE`](assets/meshes/wxai/LICENSE) and
[`NOTICE.md`](assets/meshes/wxai/NOTICE.md).
Rendering via [three.js](https://threejs.org) r169, loaded from jsDelivr.
