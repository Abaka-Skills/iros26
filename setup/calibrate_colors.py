"""Make a colour recognition profile (config/colors/NAME.yaml) from the live cameras of a running
panel, for a new table, light or venue. Two steps:

    uv run python setup/calibrate_colors.py NAME
        finds the cubes in the top camera and numbers them in config/colors/NAME.jpg. Lay the cubes
        out singly (no stacks), every colour at least once, clear of the arms.
    uv run python setup/calibrate_colors.py NAME --labels=red,-,orange,...
        the colour of each numbered cube, in order ("-" skips one). Samples every labelled cube in
        every camera, white-balanced on the table, and writes the profile: prototypes per camera
        where they differ, the table colour, and a chroma floor under the weakest cube. Prints how
        both this profile and the base one (--base) classify the samples.

Then launch with `panel --colors NAME`. Needs the panel (with a localize calibration) running.
"""
import argparse
import io
import os
import sys
import time

import cv2
import httpx
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root
from myaloha import colors, cubes  # noqa: E402
from myaloha.coords import apply_T, distort_norm, inv_T  # noqa: E402

SIZE = cubes.SIZE
KEYS = ("high", "low", "wrist_left", "wrist_right")


def get(api, path, **params):
    r = httpx.get(f"{api}/api{path}", params=params, timeout=30)
    r.raise_for_status()
    return r


def views(api) -> dict:
    """{key: CameraView} of every camera cam_params has a pose for (it is started if need be)."""
    httpx.post(f"{api}/api/topics/cam_params/start", timeout=30)
    for _ in range(40):                              # a topic just started takes a few seconds
        topic = next(t for t in get(api, "/topics").json() if t["name"] == "cam_params")
        try:
            cams = httpx.get(f"{topic['url']}/latest", timeout=10).json()["data"]["cameras"]
            break
        except (httpx.HTTPError, TypeError, KeyError):
            time.sleep(0.5)
    else:
        sys.exit("cam_params gives no camera poses (is there a localize calibration?)")
    out = {}
    for key in KEYS:
        c = cams.get(key)
        if not c or c.get("T_W_cam") is None:
            print(f"  {key}: no pose, skipped")
            continue
        try:
            z = np.load(io.BytesIO(get(api, f"/cameras/{key}/raw", n=3).content))
        except httpx.HTTPError as e:
            print(f"  {key}: no frames ({e}), skipped")
            continue
        out[key] = cubes.CameraView(key, z["color"], z["depth"], z["K"], z["dist"], str(z["model"]),
                                    np.array(c["T_W_cam"]), fixed=c["source"] == "localize")
    return out


def workspace(api):
    doc = get(api, "/coordinates").json()
    return min(abs(np.array(T)[0, 3]) for T in doc["T_W_mount"].values()) - 0.02, 0.37


def find(view, ws) -> list[tuple]:
    """(x, y, pixel) of each cube-sized blob of colour in the top camera, left to right."""
    table, _ = cubes.table_color(view, ws)
    lab = cubes.balanced_lab(view.color, table, cubes.Palette({**colors.load("default"), "balance": "tint"}))
    table_L = cv2.cvtColor(np.uint8([[table]]), cv2.COLOR_BGR2Lab)[0, 0, 0]
    # a cube is clearly coloured, or clearly darker than the table (not black like the arms)
    mask = (np.hypot(lab[..., 1], lab[..., 2]) >= 12) | ((lab[..., 0] > 40) & (lab[..., 0] < 0.6 * table_L))
    h, w = mask.shape
    v, u = np.mgrid[0:h, 0:w]
    rays = cubes._rays(view.K, view.dist, view.model, u.ravel().astype(float), v.ravel().astype(float)) @ view.T_W_cam[:3, :3].T
    o = view.T_W_cam[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        top = o[:2] + rays[:, :2] * ((SIZE - o[2]) / rays[:, 2])[:, None]   # where each pixel meets a cube top
    mask &= cubes._in_ws(top, ws).reshape(h, w) & (rays[:, 2] < 0).reshape(h, w)
    mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, comp, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    face = (view.K[0, 0] * SIZE / view.T_W_cam[2, 3]) ** 2
    out = []
    chroma = np.hypot(lab[..., 1], lab[..., 2])
    for i in range(1, n):
        if not 0.4 * face < stats[i, 4] < 3 * face:
            continue
        blob = comp == i
        (_, _), (bw, bh), _ = cv2.minAreaRect(np.argwhere(blob).astype(np.float32))
        if stats[i, 4] < 0.6 * bw * bh or max(bw, bh) > 1.8 * min(bw, bh) or np.median(chroma[blob]) < 7:
            continue                                  # not a square, or grey (the arms, the rails)
        ray = cubes._rays(view.K, view.dist, view.model, cents[i][:1], cents[i][1:]) @ view.T_W_cam[:3, :3].T
        o = view.T_W_cam[:3, 3]
        p = o + ray[0] * (SIZE - o[2]) / ray[0, 2]                   # on the top face of a cube on the table
        if cubes._in_ws(p[:2], ws):
            out.append((float(p[0]), float(p[1]), cents[i]))
    return sorted(out, key=lambda c: (c[2][0], c[2][1]))


def sample(view, xy, table, palette) -> np.ndarray:
    """Balanced Lab of the cube at xy in this view: its top face and the side turned to the camera,
    where the depth there says the face is not hidden."""
    lab = cubes.balanced_lab(view.color, table, palette)
    o = view.T_W_cam[:3, 3]
    away = np.array([*(o[:2] - xy), 0.0])
    away /= np.linalg.norm(away) + 1e-9
    pts = np.array([[*xy, SIZE], [*xy, SIZE / 2] + away * SIZE / 2])
    Pc = apply_T(inv_T(view.T_W_cam), pts)
    out = []
    for pc in Pc:
        if pc[2] < 0.05:
            continue
        xd, yd = distort_norm(np.array([pc[0] / pc[2]]), np.array([pc[1] / pc[2]]), view.dist, view.model)
        u, v = int(xd[0] * view.K[0, 0] + view.K[0, 2]), int(yd[0] * view.K[1, 1] + view.K[1, 2])
        r = max(1, int(0.25 * view.K[0, 0] * SIZE / pc[2]))
        h, w = view.depth.shape
        if not (r <= u < w - r and r <= v < h - r):
            continue
        box = np.s_[v - r:v + r + 1, u - r:u + r + 1]
        seen = np.abs(view.depth[box] - pc[2]) < max(0.015, 0.08 * pc[2])      # D405 depth runs long when far
        if seen.mean() >= 0.5:
            out.append(lab[box][seen])
    return np.concatenate(out) if out else np.zeros((0, 3), np.float32)


def background(view, table, palette, ws, k=12) -> np.ndarray:
    """What is not a cube in this view, balanced Lab: every pixel whose depth puts it outside where
    cubes can be (over the table, up to a 4-stack), less the greyest (never cubes anyway), and
    `k` cluster centres of those."""
    P, u, v, _ = cubes._lift(view, step=2)
    zone = cubes._in_ws(P, ws) & (P[:, 2] < (cubes.MAX_LEVEL + 1) * SIZE + 0.01)
    lab = cubes.balanced_lab(view.color, table, palette)[v[~zone], u[~zone]]
    lab = lab[np.hypot(lab[:, 1], lab[:, 2]) >= 3].astype(np.float32)
    if len(lab) < k:
        return lab, lab
    _, _, centres = cv2.kmeans(lab * palette.w, k, None, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.5),
                               3, cv2.KMEANS_PP_CENTERS)
    return lab, centres / palette.w


def classify_lab(lab, palette) -> np.ndarray:
    ok = np.hypot(lab[:, 1], lab[:, 2]) >= palette.min_chroma
    x = lab * palette.w
    d = palette.p2 - 2 * x @ palette.p.T + (x * x).sum(1)[:, None]
    k = d.argmin(-1)
    return np.where(ok & (d[np.arange(len(k)), k] < palette.max_dist ** 2), palette.ids[k], -1)


def report(samples, backgrounds, palette, title):
    print(f"\n{title}: share of each cube's pixels classified right, per camera; and of what is not a "
          f"cube, the share taken for one")
    for key in KEYS:
        rows = [(name, lab) for k, name, lab in samples if k == key and len(lab)]
        if rows:
            wrong = np.mean(classify_lab(backgrounds[key], palette) >= 0) if len(backgrounds[key]) else 0.0
            print(f"  {key:12s}" + "  ".join(f"{name} {np.mean(classify_lab(lab, palette) == cubes.NAMES.index(name)):4.0%}"
                                             for name, lab in rows) + f"   | not cubes: {wrong:.1%}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="the profile to write (config/colors/NAME.yaml)")
    ap.add_argument("--labels", help="the numbered cubes' colours in order, comma separated ('-' skips)")
    ap.add_argument("--base", default="default", help="profile to start from (weights, non-cube colours)")
    ap.add_argument("--balance", default="full", choices=("tint", "full"))
    ap.add_argument("--api", default="http://127.0.0.1:8765")
    args = ap.parse_args()

    ws = workspace(args.api)
    vs = views(args.api)
    if "high" not in vs:
        sys.exit("the top camera is needed to find the cubes")
    found = find(vs["high"], ws)
    image = colors.path(args.name).with_suffix(".jpg")
    vis = vs["high"].color.copy()
    for i, (_, _, c) in enumerate(found, 1):
        cv2.circle(vis, tuple(int(v) for v in c), 9, (0, 255, 0), 1)
        cv2.putText(vis, str(i), tuple(int(v) for v in c + [10, -8]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)
    image.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(image), cv2.resize(vis, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))
    if not args.labels:
        print("\n".join(f"  {i:2d}  at ({x:+.3f}, {y:+.3f})" for i, (x, y, _) in enumerate(found, 1)))
        print(f"{len(found)} cubes numbered in {image}; name their colours with --labels (one of "
              f"{', '.join(cubes.NAMES)}, or - to skip), in order")
        return
    names = args.labels.split(",")
    if len(names) != len(found):
        sys.exit(f"{len(names)} labels for {len(found)} cubes (see {image}); the scene may have changed")
    bad = sorted({n for n in names if n != "-" and n not in cubes.NAMES})
    if bad:
        sys.exit(f"not cube colours: {', '.join(bad)}")

    base = colors.load(args.base)
    tables = {k: cubes.table_color(v, ws)[0] for k, v in vs.items()}
    ref = tables["high"]
    ref_lab = cv2.cvtColor(np.uint8([[ref]]), cv2.COLOR_BGR2Lab)[0, 0].astype(int) - [0, 128, 128]
    draft = cubes.Palette({**base, "balance": args.balance, "table": ref_lab.tolist()})
    samples = [(k, name, sample(v, np.array(xy), tables[k], draft))
               for k, v in vs.items() for (xy0, xy1, _), name in zip(found, names) if name != "-"
               for xy in [(xy0, xy1)]]
    # a prototype per colour and camera (median over its cubes there), merged where cameras agree
    protos = {}
    for name in dict.fromkeys(n for n in names if n != "-"):
        meds = [np.median(lab, 0) for k in KEYS for kk, nn, lab in samples if kk == k and nn == name and len(lab) >= 5]
        keep = []
        for m in meds:
            if all(np.linalg.norm((m - q) * draft.w) > 8 for q in keep):
                keep.append(m)
        protos[name] = [np.round(m).astype(int).tolist() for m in keep]
    missing = [n for n in cubes.NAMES if n not in protos]
    protos.update({n: base["colors"][n] for n in missing})
    chroma = np.concatenate([np.hypot(lab[:, 1], lab[:, 2]) for _, _, lab in samples if len(lab)])
    # what is not a cube, from these very frames: its colours become prototypes that aren't cubes, so
    # the nearest-prototype rule draws the line between a dark cube and the dark arms, rails and shadows
    bg = {k: background(v, tables[k], draft, ws) for k, v in vs.items()}
    others = {k: np.round(centres).astype(int).tolist() for k, (_, centres) in bg.items() if len(centres)}
    doc = {"note": f"setup/calibrate_colors.py on {len(found)} cubes, cameras {', '.join(vs)}",
           "balance": args.balance, "table": ref_lab.tolist(), "l_weight": base["l_weight"],
           "max_dist": base["max_dist"],
           "min_chroma": float(np.clip(np.round(0.6 * np.percentile(chroma, 5)), 3, base["min_chroma"])),
           "colors": protos, "others": {**base["others"], **others}}
    new = cubes.Palette({"name": args.name, **doc})
    backgrounds = {k: lab for k, (lab, _) in bg.items()}
    report(samples, backgrounds, cubes.Palette(base), f"base profile {args.base!r}")
    report(samples, backgrounds, new, f"new profile {args.name!r}")
    print(f"\nwrote {colors.save(args.name, doc)}" + (f" (no {', '.join(missing)} cube labelled: the base "
                                                        f"profile's prototypes kept for it)" if missing else ""))


if __name__ == "__main__":
    main()
