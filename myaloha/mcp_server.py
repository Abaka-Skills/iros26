"""The robot for coding agents: one MCP server, served by the backend at /mcp.

Mods are the "do" side: one `<mod>_run` tool per mod, generated from its Params (types,
defaults and help text become the tool's schema), plus stop and estop. Topics are the "read"
side: one `<topic>_latest` tool per topic. Agents never control topics (start/stop is the
panel's and the REST API's) and can't release the emergency stop.

Every call returns at once. A run started here is tagged `source: agent` in the timeline and log.
"""
import inspect
from dataclasses import fields
from typing import Annotated

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import Field

from myaloha.mods import MODS
from myaloha.topics import TOPICS

INSTRUCTIONS = """Bimanual robot (two WidowX AI followers, optional leaders, four RealSense cameras).
Only one mod runs at a time; `<mod>_run` returns immediately, then poll `mod_status` (or
`robot_status`) to follow its state machine to done / stopped / failed. `stop` ends a mod
gracefully; `estop` brakes every arm at once and only a human can release it. Topics are live
perception results: read them with `<topic>_latest`; they must be started from the panel."""


def build(robot, topics) -> MCPServer:
    mcp = MCPServer(name="myaloha", instructions=INSTRUCTIONS)

    def robot_status() -> dict:
        """Everything live: major state (offline/idle/running/estop), each arm (connected, mode, joint
        positions in rad, gripper in m), each camera (ok, fps), the coordinate calibration id, the
        running mod with its state, the last finished run, and each topic's state."""
        s = robot.status()
        return {
            "state": s["state"],
            "arms": {n: {k: a[k] for k in ("role", "side", "connected", "lost", "mode", "pos", "error")}
                     for n, a in s["arms"].items()},
            "cameras": {k: {"ok": c["ok"], "fps": c["fps"], "error": c["error"]} for k, c in s["cameras"].items()},
            "coordinates": s["coordinates"],
            "running": [{k: m[k] for k in ("name", "state", "source", "error")} for m in s["mods"]],
            "last_run": _brief(robot.last_run),
            "topics": {n: t["state"] for n, t in topics.status().items()},
        }

    def mods_list() -> list:
        """Every mod: what it does, its params (type, default, help), its states, and whether the
        connected hardware meets its requirements right now."""
        out = []
        for cls in MODS.values():
            r = cls.requirements(robot)
            out.append({**cls.catalog(), "doc": inspect.cleandoc(cls.__doc__ or ""), "tool": f"{cls.name}_run",
                        "ready": r["met"], "unmet": [n["text"] for n in r["needs"] if not n["met"]]})
        return out

    def mod_status() -> dict:
        """The running mod (state, time in each state, error) and the last finished run with its
        result. A run is over when its state is done, stopped or failed."""
        return {"running": [_brief(m.status()) for m in robot.mods], "last_run": _brief(robot.last_run)}

    def stop() -> dict:
        """Stop every running mod gracefully (each leaves its arms holding). Returns their names."""
        robot.stop()
        return {"stopping": [m.name for m in robot.mods]}

    def estop() -> dict:
        """EMERGENCY STOP: brake every arm now. Latched: mods can't start and arms refuse commands
        until a human resets it in the panel. Use when anything looks unsafe."""
        robot.emergency_stop()
        return {"estopped": True, "release": "only from the panel or the REST API, by a human"}

    def topics_list() -> dict:
        """Every topic: state (stopped / starting / running / waiting / crashed), what it needs and
        what one result holds. Start or stop topics from the panel; agents only read them."""
        st = topics.status()
        return {n: {**cls.catalog(), "doc": inspect.cleandoc(cls.__doc__ or ""), "tool": f"{n}_latest",
                    "state": st[n]["state"], "waiting": st[n].get("waiting")} for n, cls in TOPICS.items()}

    for fn in (robot_status, mods_list, mod_status, stop, estop, topics_list):
        mcp.add_tool(fn)
    for cls in MODS.values():
        mcp.add_tool(_run_tool(robot, cls), name=f"{cls.name}_run", description=_run_doc(cls))
    for name, cls in TOPICS.items():
        mcp.add_tool(_latest_tool(topics, name), name=f"{name}_latest",
                     description=f"{inspect.cleandoc(cls.__doc__ or '')}\n\nReturns {{seq, t, wall, data}} where "
                                 f"data holds {cls.OUTPUT}. Fails if the topic is not running (start it from the panel).")
    return mcp


def _plain(v):
    return list(v) if isinstance(v, tuple) else v


def _brief(status: dict | None) -> dict | None:
    if not status:
        return None
    keep = ("name", "state", "source", "error", "params", "history", "result")
    return {k: status[k] for k in keep if k in status}


def _run_doc(cls) -> str:
    states = "; ".join(f"{s}: {d}" for s, d in cls.STATES.items() if s != "idle")
    needs = "; ".join(n.text for n in cls.REQUIRES) or "nothing"
    return (f"{inspect.cleandoc(cls.__doc__ or '')}\n\nNeeds: {needs}.\nStates: {states}.\n"
            f"Starts the mod and returns at once; poll mod_status until done / stopped / failed.")


def _run_tool(robot, cls):
    """A tool whose signature is the mod's Params, so the MCP schema shows each param's type,
    default and help."""
    def run(**params) -> dict:
        try:
            p = cls.params_from(params)
        except (ValueError, TypeError) as e:
            raise ToolError(f"{cls.name} not started: bad params: {e}") from e
        m = cls(robot, p, source="agent")
        try:
            m.start()
        except RuntimeError as e:        # RequirementError, robot busy, emergency stop: refusals
            raise ToolError(f"{cls.name} not started: {e}") from e
        return _brief(m.status())

    defaults, sig = cls.Params(), []
    for f in fields(cls.Params):
        d = getattr(defaults, f.name)
        typ = list[str] if isinstance(d, tuple) else type(d)
        sig.append(inspect.Parameter(f.name, inspect.Parameter.KEYWORD_ONLY, default=_plain(d),
                                     annotation=Annotated[typ, Field(description=f.metadata.get("help", ""))]))
    run.__signature__ = inspect.Signature(sig, return_annotation=dict)
    run.__name__ = f"{cls.name}_run"
    return run


def _latest_tool(topics, name: str):
    def latest() -> dict:
        m = topics.latest(name)
        if m is None:
            raise ToolError(f"{name} is {topics.status()[name]['state']}; it must be started from the panel")
        return m

    latest.__name__ = f"{name}_latest"
    return latest
