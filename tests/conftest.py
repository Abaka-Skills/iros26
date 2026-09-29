import pytest

from simrig import connect, release, simulated_rig


@pytest.fixture
def sim_robot(request):
    """(robot, Sim) with the devices named by `@pytest.mark.unplugged(...)` absent and the cubes
    of `@pytest.mark.cubes([...])` on the table."""
    unplugged = request.node.get_closest_marker("unplugged")
    cubes = request.node.get_closest_marker("cubes")
    with simulated_rig(unplugged.args if unplugged else (), cubes.args[0] if cubes else None) as sim:
        robot = connect(sim.address)
        yield robot, sim
        release(robot)
