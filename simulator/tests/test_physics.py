"""The simulator's physics (MuJoCo): a cube dropped on another rests on it, one that overhangs its
support too far topples off, a little off centre it stays; fingers closing on a cube stop on it,
lift it by friction and drop it when they open; an arm sweeping through a cube pushes it away."""
import numpy as np
import pytest

from simulator import load_config
from simulator.world import World

S = 0.025
LEFT = "192.168.10.5"
DOWN = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])   # tool x (approach) down, fingers close along W y


class Ctl:
    """A controller whose motor targets the test sets."""
    plugged = True

    def __init__(self, motor):
        self.m = list(motor)

    def motor(self):
        return list(self.m)


@pytest.fixture
def world():
    w = World(load_config(overrides={"seed": 1, "noise": 0.0, "cubes.count": 0}), realtime=False)
    yield w
    w.scene.close()


def cube(w, color):
    return next(c for c in w.truth()["cubes"] if c["color"] == color)


def ik(w, p, R=DOWN, q=(0.0, 1.5, 1.0, -1.0, 0.0, 0.0)):
    """Joint angles putting the left follower's tool at (p, R) (no noise here: links = motors)."""
    q = np.array(q, float)

    def err(q):
        T = w.scene.tool_pose(LEFT, q)
        E = R @ T[:3, :3].T
        angle = np.arccos(np.clip((np.trace(E) - 1) / 2, -1, 1))
        axis = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
        rot = axis / (2 * np.sin(angle)) * angle if angle > 1e-9 else axis / 2
        return np.concatenate([p - T[:3, 3], 0.2 * rot])
    for _ in range(200):
        e = err(q)
        if np.linalg.norm(e) < 1e-6:
            break
        J = np.stack([(err(q + dq) - e) / 1e-6 for dq in np.eye(6) * 1e-6], 1)
        q -= np.linalg.lstsq(J, e, rcond=None)[0]
    assert np.linalg.norm(err(q)) < 1e-4
    return q


def test_a_cube_dropped_on_another_rests_on_it(world):
    world.spawn([{"color": "red", "x": 0.0, "y": 0.0}, {"color": "blue", "x": 0.0, "y": 0.0, "z": 0.07}])
    world.step(1.0)
    top = cube(world, "blue")
    assert abs(top["z"] - 1.5 * S) < 0.002 and top["tilt"] < 2 and top["level"] == 1, top


def test_a_cube_overhanging_its_support_too_far_topples_off(world):
    """Its centre of mass beyond the edge below (12.5 mm from the centre): it tips over and falls."""
    world.spawn([{"color": "red", "x": 0.0, "y": 0.0}, {"color": "blue", "x": 0.019, "y": 0.0, "level": 1}])
    world.step(2.0)
    top = cube(world, "blue")
    assert top["level"] == 0 and top["x"] > 0.02, top                      # on the table, beside it
    assert cube(world, "red")["level"] == 0


def test_a_cube_a_little_off_centre_stays(world):
    world.spawn([{"color": "red", "x": 0.0, "y": 0.0}, {"color": "blue", "x": 0.006, "y": 0.0, "level": 1}])
    world.step(2.0)
    top = cube(world, "blue")
    assert top["level"] == 1 and top["tilt"] < 2 and abs(top["x"] - 0.006) < 0.002, top


def test_fingers_stop_on_a_cube_lift_it_by_friction_and_drop_it_when_they_open(world):
    x = -0.166                                                             # 0.30 m in front of the left base
    world.spawn([{"color": "red", "x": x, "y": 0.0}])
    above, at = ik(world, [x, 0, 0.08]), ik(world, [x, 0, 0.008])
    ctl = world.controllers[LEFT] = Ctl([*above, 0.022])                   # fingers open: 44 mm
    world.step(2.0)
    ctl.m = [*at, 0.022]                                                   # down around the cube
    world.step(2.0)
    ctl.m[6] = 0.0                                                         # close
    world.step(1.0)
    stopped = world.measured(LEFT, ctl.m)[6]
    assert abs(2 * stopped - S) < 0.002, stopped                           # on the cube's faces
    ctl.m = [*above, stopped - 0.004]                                      # squeeze, and lift
    world.step(2.0)
    held = cube(world, "red")
    assert held["held_by"] == "follower_left" and held["z"] > 0.07, held
    ctl.m[6] = 0.022                                                       # open: it falls
    world.step(1.5)
    dropped = cube(world, "red")
    assert dropped["held_by"] is None and dropped["level"] == 0 and abs(dropped["x"] - x) < 0.02, dropped


def test_an_arm_sweeping_through_a_cube_pushes_it(world):
    x = -0.166
    world.spawn([{"color": "red", "x": x, "y": 0.0}])
    ctl = world.controllers[LEFT] = Ctl([*ik(world, [x - 0.06, 0, 0.012]), 0.0])
    world.step(2.0)
    for dx in np.linspace(-0.06, 0.06, 60):                                # the closed fingers sweep along x
        ctl.m = [*ik(world, [x + dx, 0, 0.012], q=ctl.m[:6]), 0.0]
        world.step(0.05)
    world.step(0.5)
    assert cube(world, "red")["x"] > x + 0.03                              # pushed ahead of the fingers
