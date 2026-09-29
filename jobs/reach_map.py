"""Reachability of one WidowX AI follower in its base frame, by the pick_place planner's rules (a joint
solution with every joint at least 8 deg inside its limits). Joint 0 turns the whole arm about the base's
z, so reachability depends only on the radius r and height z of the tool point and the tool's approach
direction relative to the radial one: elevation `el` (-90: straight down) and azimuth `az` (0: pointing
away from the base, 90: to its left). The tool's roll about the approach is joint 5 (it turns fully), so
it is not sampled. Per cell: the largest joint margin among the solutions IK finds (rad; NaN: none),
that solution's lowest joint origin past the shoulder (m, base frame; the table is about -0.025 there),
and its joints `q` (at bearing 0, tool frame `frame(el, az)`): a starting point for IK anywhere near.

Run on the Ray cluster: `uv run python jobs/ray_job.py reach_map config/reach/wxai_follower.npz` (~25 min
on 24 CPUs). Read by myaloha.reach."""
import base64
import io
import time

import numpy as np
import ray
from heron_kinematics import Kinematics, rotvec

FILES = ["myaloha/vendor/heron_kinematics.py", "myaloha/vendor/wxai_follower.urdf"]   # shipped with it
R = np.arange(0.0, 0.73, 0.02)
Z = np.arange(-0.04, 0.57, 0.02)
DIRS = [(-90.0, 0.0)] + [(el, az) for el in (-60.0, -30.0, 0.0, 30.0, 60.0) for az in range(0, 360, 30)]
MARGIN = np.radians(8)


def frame(el, az):
    """A tool rotation whose x (the approach) points at elevation el, azimuth az (deg) from the +x radial."""
    e, a = np.radians(el), np.radians(az)
    x = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    up = np.array([0, 0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0, 0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    return np.c_[x, y, np.cross(x, y)]


def solutions(kin, p, Rt, seeds, rng, iters=150):
    """Every converged IK solution from `seeds` starts (the vendored ik keeps only the one nearest its seed)."""
    out = []
    for s in range(seeds):
        q = (kin.lo + kin.hi) / 2 if s == 0 else rng.uniform(kin.lo, kin.hi)
        for _ in range(iters):
            t = kin.fk(q)
            e_p, e_r = p - t[:3, 3], rotvec(Rt @ t[:3, :3].T)
            if np.linalg.norm(e_p) < 1e-4 and np.linalg.norm(e_r) < 2e-3:
                out.append(q.copy())
                break
            jac = kin.jacobian(q)
            err = np.concatenate([e_p, e_r])
            lam = 0.05 + 0.5 * float(np.linalg.norm(err))
            q = np.clip(q + np.clip(jac.T @ np.linalg.solve(jac @ jac.T + lam ** 2 * np.eye(6), err), -0.25, 0.25),
                        kin.lo, kin.hi)
    return out


@ray.remote
def chunk(cells, seeds):
    kin = Kinematics("wxai_follower.urdf")
    rng = np.random.default_rng(0)
    out = []
    for i, j, k in cells:
        p, Rt = np.array([R[i], 0.0, Z[j]]), frame(*DIRS[k])
        best = (np.nan, np.nan, np.full(6, np.nan))
        for q in solutions(kin, p, Rt, seeds, rng):
            m = float(np.min(np.minimum(q - kin.lo, kin.hi - q)))
            if m >= MARGIN and not (m <= best[0]):
                best = (m, float(kin.link_points(q)[2:-1, 2].min()), q)
        out.append((i, j, k, *best))
    return out


if __name__ == "__main__":
    ray.init()
    t0 = time.time()
    cells = [(i, j, k) for i in range(len(R)) for j in range(len(Z)) for k in range(len(DIRS))]
    rng = np.random.default_rng(1)
    rng.shuffle(cells)                                       # even out the slow (unreachable) cells
    parts = [cells[n::240] for n in range(240)]
    margin = np.full((len(R), len(Z), len(DIRS)), np.nan, np.float32)
    low = np.full_like(margin, np.nan)
    qs = np.full(margin.shape + (6,), np.nan, np.float32)
    for res in ray.get([chunk.remote(p, 10) for p in parts]):
        for i, j, k, m, z, q in res:
            margin[i, j, k], low[i, j, k], qs[i, j, k] = m, z, q
    buf = io.BytesIO()
    np.savez_compressed(buf, r=R, z=Z, dirs=np.array(DIRS), margin=margin, low=low, q=qs)
    print(f"cells {len(cells)}, reachable {np.isfinite(margin).mean():.1%}, {time.time() - t0:.0f} s", flush=True)
    print("RESULT_BEGIN\n" + base64.b64encode(buf.getvalue()).decode() + "\nRESULT_END", flush=True)
