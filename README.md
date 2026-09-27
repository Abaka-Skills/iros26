# myaloha

Bimanual Trossen setup (4× WidowX AI arms, 4× RealSense D405) driven from macOS through
[LeRobot](https://github.com/huggingface/lerobot), plus the public visitor page.

| | |
|---|---|
| [`myaloha/`](myaloha) | Robot wrapper over LeRobot's WidowX AI classes (leaders optional), cameras, mods (units of behaviour, each with its own state machine), topics (perception processes), and the localhost API + MCP server |
| [`panel/`](panel) | Web: the operator panel (`index.html`, a client of the API) and the Cube Sandbox visitor page (`visitor.html`, public at **https://abaka-skills.github.io/iros26/**) |
| [`setup/`](setup) | Arm and camera bring-up scripts |
| [`thirdparty/`](thirdparty) | `lerobot_trossen` (submodule, unmodified) |

## Setup

```sh
git clone --recurse-submodules https://github.com/Abaka-Skills/iros26.git
cd iros26
uv sync                                  # one environment for the whole repo
uv run pytest                            # mods and API against simulated hardware
uv run python -m myaloha panel           # check the hardware, serve API + panel, open it
uv run python -m myaloha panel --sim     # the same without hardware
uv run python -m myaloha status          # connect and report; moves nothing
uv run python -m myaloha run reset       # teleop / instruction likewise; key=value params
```

`panel` prints a preflight report (network, each arm, each camera), then serves on
`http://127.0.0.1:8765/`, with every endpoint at `/docs`. The cameras need root on macOS, so it
re-runs itself under sudo unless given `--no-cameras`. Status and plans: [`PROGRESS_v2.md`](PROGRESS_v2.md).

**Mods** do things with the robot, one at a time (`reset`, `teleop`, `instruction`, `localize`).
**Topics** are perception processes the backend starts on demand, each on its own localhost
port (`/latest`, `/stream`): `cam_params` (every camera's intrinsics and live pose) and
`locate_cubes` (every cube on the table). Start and stop them from the panel or
`/api/topics/{name}/start|stop`.

**Coding agents** reach the robot over MCP at `http://127.0.0.1:8765/mcp` (while the panel runs):

```sh
claude mcp add --transport http myaloha http://127.0.0.1:8765/mcp
```

It offers `<mod>_run` for every mod (returns at once; follow it with `mod_status`), `stop`,
`estop` (only a human releases it), `robot_status`, and `<topic>_latest` for every topic
(read only; topics are started from the panel).
