# myaloha

Bimanual Trossen setup (4× WidowX AI arms, 4× RealSense D405) driven from macOS through
[LeRobot](https://github.com/huggingface/lerobot), plus the public visitor page.

| | |
|---|---|
| [`myaloha/`](myaloha) | Robot wrapper over LeRobot's WidowX AI classes (leaders optional) and mods: units of behaviour, each with its own state machine |
| [`panel/`](panel) | Operator panel: the rig in 3D, cameras, mod browser, state machines, log |
| [`visitor_panel/`](visitor_panel) | Cube Sandbox, the public page at **https://abaka-skills.github.io/iros26/** |
| [`setup/`](setup) | Arm and camera bring-up scripts |
| [`thirdparty/`](thirdparty) | `lerobot_trossen` (submodule, unmodified) |

## Setup

```sh
git clone --recurse-submodules https://github.com/Abaka-Skills/iros26.git
cd iros26
uv sync                                  # one environment for the whole repo
uv run pytest                            # mods against a fake driver, no hardware
uv run python -m myaloha status          # connect and report; moves nothing
uv run python -m myaloha run reset       # teleop / instruction likewise; key=value params
python3 -m http.server 8000              # then open http://localhost:8000/panel/
```

The panel runs on mock status until the backend lands; see [`PROGRESS.md`](PROGRESS.md).
