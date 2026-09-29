"""The safety boundary: margins in from the table's edges, the box they leave, kept per rig."""
import pytest

from myaloha.safety import Boundary, table


def test_margins_move_each_edge_in():
    x0, y0, x1, y1 = table()
    b = Boundary(up=0.05, down=0.10, left=0.02, right=0.04)
    assert b.box() == pytest.approx((x0 + 0.02, y0 + 0.10, x1 - 0.04, y1 - 0.05))
    assert b.contains([0.0, 0.0]) and not b.contains([0.0, y0 + 0.09]) and not b.contains([x1 - 0.03, 0.0])
    assert list(b.contains([[0.0, 0.0, 0.3], [x0 + 0.01, 0.0, 0.1]])) == [True, False]


def test_changes_are_checked_and_kept(tmp_path):
    path = tmp_path / "rig" / "boundary.yaml"
    b = Boundary.load(path)
    assert b.margins == Boundary().margins and not path.exists()           # the default until changed
    b.set(down=0.12)
    assert Boundary.load(path).margins == {**Boundary().margins, "down": 0.12}
    for bad in ({"up": -0.01}, {"left": 0.7, "right": 0.6}, {"front": 0.1}, {"up": float("nan")}):
        with pytest.raises(ValueError):
            b.set(**bad)
    assert Boundary.load(path).down == 0.12                                  # a refused change keeps nothing
