"""The MCP server (in-process client) against simulated hardware: mods are the "do" side, topics
the "read" side; an agent can trigger the emergency stop but never release it."""
import asyncio
import json
import time

import pytest
import trossen_arm
from mcp.client.client import Client

from myaloha import mcp_server, sim
from myaloha.cameras import CameraRig
from myaloha.robot import Aloha
from myaloha.topics.manager import TopicManager


@pytest.fixture
def robot(monkeypatch):
    monkeypatch.setattr(trossen_arm, "TrossenArmDriver", sim.SimDriver)
    monkeypatch.setattr(sim.SimDriver, "ABSENT", {"192.168.10.2", "192.168.10.3"})   # no leaders
    rig = CameraRig()
    r = Aloha(cameras=rig)
    r.connect(timeout=0.1)
    sim.add_cameras(rig)
    r.start_monitor(hz=100)
    time.sleep(1.2)
    yield r
    r.stop()
    time.sleep(0.3)
    r.disconnect()


def call(robot, steps):
    """Run `steps(client)` against a fresh server for `robot`; returns its result."""
    server = mcp_server.build(robot, TopicManager(robot, "http://127.0.0.1:9"))

    async def main():
        async with Client(server) as c:
            return await steps(c)
    return asyncio.run(main())


def text(r):
    return r.content[0].text


def test_tools_cover_every_mod_and_topic_but_not_estop_release(robot):
    async def steps(c):
        return [t.name for t in (await c.list_tools()).tools]
    names = call(robot, steps)
    for n in ("robot_status", "mods_list", "mod_status", "stop", "estop", "topics_list", "reset_run", "teleop_run",
              "instruction_run", "localize_run", "cam_params_latest", "locate_cubes_latest"):
        assert n in names
    assert not [n for n in names if "reset" in n and n != "reset_run"]            # no estop release
    assert not [n for n in names if n.startswith("topic_") or n.endswith(("_start", "_stop"))]


def test_run_a_mod_as_an_agent(robot):
    async def steps(c):
        started = json.loads(text(await c.call_tool("reset_run", {"goal_time": 0.2, "via_staged": False})))
        for _ in range(100):
            s = json.loads(text(await c.call_tool("mod_status", {})))
            if not s["running"] and s["last_run"]:
                return started, s
            await asyncio.sleep(0.05)
    started, s = call(robot, steps)
    assert started["source"] == "agent"
    assert s["last_run"]["state"] == "done" and s["last_run"]["source"] == "agent"
    assert robot.timeline[-2]["mod"] == "reset" and robot.timeline[-2]["source"] == "agent"


def test_refusals_carry_their_reason(robot):
    async def steps(c):
        return await c.call_tool("teleop_run", {}), await c.call_tool("locate_cubes_latest", {})
    teleop, cubes = call(robot, steps)
    assert teleop.is_error and "teleop not started" in text(teleop) and "leader" in text(teleop)
    assert cubes.is_error and "started from the panel" in text(cubes)


def test_estop_latches_until_a_human_resets_it(robot):
    async def steps(c):
        await c.call_tool("estop", {})
        return await c.call_tool("reset_run", {})
    refused = call(robot, steps)
    assert robot.estopped and refused.is_error and "emergency stop" in text(refused)
    robot.reset_emergency_stop()      # the panel / REST path
    assert not robot.estopped
