"""The MuJoCo model of the rig, for physics and for the cameras: each follower built from the URDF
(its masses and inertias, its collision meshes as convex hulls, its visual meshes), the table, the
floor and the cubes.

- Physics (the world steps it): the cubes are free rigid bodies (gravity, friction, contacts with the
  table, each other, the fingers and every arm link). A follower's joints are position servos
  holding the world's joint targets, its links gravity-compensated (the world adds its own sag to the
  targets); its two carriages are force-limited position servos, so fingers closing on a cube stop
  on it and squeeze by how far their target is past it. An arm does not collide with itself; the two
  arms collide with each other, with the cubes and with the table (its base and shoulder excepted:
  they sit on it).
- Cameras: a snapshot of the physics state (qpos) drawn from any camera pose.

Rendering runs on one thread (the GL context belongs to it); any thread may call render().
"""
from __future__ import annotations

import math
import threading
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import mujoco
import numpy as np

from simulator.geometry import make_T, rpy

MAX_CUBES = 24
FLOOR_Z = -0.75
TABLE_THICKNESS = 0.03
CUBE_MASS = 0.010            # kg: a 25 mm wooden cube
JOINT_KP, CARRIAGE_KP = 600.0, 2500.0   # servo stiffness: N m/rad (arm joints), N/m (fingers)
CARRIAGE_FORCE = 25.0        # N: the most a finger pushes with
TABLE, ARM = 1, (2, 4)       # collision bits: the table and cubes, each arm (an arm ignores its own bit)
# stiff contacts: a 10 g cube squeezed between two servos would otherwise let the fingers sink ~7 mm
CONTACT = 'solref="0.004 1" solimp="0.99 0.999 0.001"'


def _quat(R: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, float).ravel())
    return q


def _f(v) -> str:
    return " ".join(f"{x:.9g}" for x in np.ravel(v))


def _rpy_quat(text: str) -> str:
    """A URDF roll-pitch-yaw (fixed axes) as an MJCF quaternion."""
    return _f(_quat(rpy(*map(float, text.split()))))


def _attr(el, key: str, default: str) -> str:
    return el.get(key, default) if el is not None else default


def _urdf(path):
    """(root link, {link: [(mesh, xyz, rpy, scale)]} visual, the same collision, {link: inertial},
    {child link: joint}, {parent: [child links]})."""
    root = ET.parse(path).getroot()
    visuals, collisions, inertials = {}, {}, {}
    for link in root.findall("link"):
        name = link.get("name")
        for kind, out in (("visual", visuals), ("collision", collisions)):
            out[name] = []
            for v in link.findall(kind):
                mesh = v.find("geometry/mesh")
                if mesh is None:
                    continue
                o = v.find("origin")
                out[name].append((mesh.get("filename"), _attr(o, "xyz", "0 0 0"), _attr(o, "rpy", "0 0 0"),
                                  mesh.get("scale", "1 1 1")))
        i = link.find("inertial")
        if i is not None:
            o, t = i.find("origin"), i.find("inertia").attrib
            inertials[name] = (_attr(o, "xyz", "0 0 0"), float(i.find("mass").get("value")),
                               [float(t[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")])
    into, children = {}, {}
    for j in root.findall("joint"):
        o, axis, lim = j.find("origin"), j.find("axis"), j.find("limit")
        child, parent = j.find("child").get("link"), j.find("parent").get("link")
        into[child] = {"name": j.get("name"), "type": j.get("type"), "xyz": _attr(o, "xyz", "0 0 0"),
                       "rpy": _attr(o, "rpy", "0 0 0"), "axis": _attr(axis, "xyz", "1 0 0"),
                       "range": (_attr(lim, "lower", "0"), _attr(lim, "upper", "0"))}
        children.setdefault(parent, []).append(child)
    base = next(name for name in visuals if name not in into)
    return base, visuals, collisions, inertials, into, children


class Scene:
    def __init__(self, cfg: dict, table_bgr: np.ndarray):
        self.size = tuple(cfg["size"])
        self.S = S = cfg["cubes"]["size"]
        urdf = cfg["_dir"] / cfg["urdf"]
        base, visuals, collisions, inertials, into, children = _urdf(urdf)
        meshdir = urdf.parent / "meshes"
        meshes = sorted({(f, scale) for part in (visuals, collisions) for geoms in part.values()
                         for f, _, _, scale in geoms})
        mesh_name = {f: f"m{i}" for i, (f, _) in enumerate(meshes)}

        def mesh_file(f: str) -> Path:   # the URDF's path, else the same file name under meshdir
            return meshdir / f if (meshdir / f).exists() else next(meshdir.rglob(Path(f).name))
        self.followers = [ip for ip, a in cfg["arms"].items() if a["role"] == "follower"]

        def body(link: str, prefix: str, pos: str, quat: str, bit: int) -> str:
            j = into.get(link)
            out = [f'<body name="{prefix}{link}" pos="{pos}" quat="{quat}" gravcomp="1">']
            if link in inertials:
                xyz, mass, inertia = inertials[link]
                out.append(f'<inertial pos="{xyz}" mass="{mass}" fullinertia="{_f(inertia)}"/>')
            if j and j["type"] in ("revolute", "continuous"):
                out.append(f'<joint name="{prefix}{j["name"]}" type="hinge" axis="{j["axis"]}" armature="0.02" damping="0.5"/>')
            elif j and j["type"] == "prismatic":
                out.append(f'<joint name="{prefix}{j["name"]}" type="slide" axis="{j["axis"]}" range="{j["range"][0]} '
                           f'{j["range"][1]}" limited="true" armature="0.01" damping="2"/>')
            for f, xyz, r, _ in visuals.get(link, []):
                out.append(f'<geom type="mesh" mesh="{mesh_name[f]}" pos="{xyz}" quat="{_rpy_quat(r)}" material="arm" '
                           f'contype="0" conaffinity="0" group="1" mass="0"/>')
            for f, xyz, r, _ in collisions.get(link, []):
                out.append(f'<geom name="{prefix}{link}_c" type="mesh" mesh="{mesh_name[f]}" pos="{xyz}" '
                           f'quat="{_rpy_quat(r)}" contype="{bit}" conaffinity="{7 & ~bit}" group="3" mass="0" '
                           f'friction="1 0.005 0.0001" condim="4" {CONTACT} rgba="0 0 0 0"/>')
            for child in children.get(link, []):
                cj = into[child]
                out.append(body(child, prefix, cj["xyz"], _rpy_quat(cj["rpy"]), bit))
            out.append("</body>")
            return "\n".join(out)

        arms, actuators, excludes = [], [], []
        for i, ip in enumerate(self.followers):
            x, y, z, yaw = cfg["arms"][ip]["mount"]
            arms.append(body(base, f"a{i}_", _f([x, y, z]), _rpy_quat(f"0 0 {math.radians(yaw)}"), ARM[i]))
            actuators += [f'<position name="a{i}_joint_{k}" joint="a{i}_joint_{k}" kp="{JOINT_KP}" dampratio="1" '
                          f'forcelimited="false"/>' for k in range(6)]
            actuators += [f'<position name="a{i}_{s}_carriage" joint="a{i}_{s}_carriage_joint" kp="{CARRIAGE_KP}" '
                          f'dampratio="1" forcelimited="true" forcerange="-{CARRIAGE_FORCE} {CARRIAGE_FORCE}"/>'
                          for s in ("left", "right")]
            excludes += [f'<exclude body1="a{i}_{link}" body2="world"/>' for link in (base, "link_1")]
        x0, y0, x1, y1 = cfg["table"]["extent"]
        cams = []
        for serial, c in cfg["cameras"].items():
            fx, fy, cx, cy = c["K"]
            w, h = self.size
            cams.append(f'<camera name="c{serial}" pos="0 0 1" resolution="{w} {h}" sensorsize="{w} {h}" focalpixel="{fx} {fy}" '
                        f'principalpixel="{w / 2 - cx} {h / 2 - cy}"/>')
        self.parking = [np.array([3.0 + 0.05 * (i % 6), 0.05 * (i // 6), FLOOR_Z + S / 2]) for i in range(MAX_CUBES)]
        cubes = "\n".join(
            f'<body name="cube{i}" pos="{_f(self.parking[i])}"><freejoint name="cube{i}"/>'
            f'<geom name="cube{i}" type="box" size="{_f([S / 2] * 3)}" mass="{CUBE_MASS}" material="cube" '
            f'contype="{TABLE}" conaffinity="7" friction="1 0.005 0.0001" condim="4" {CONTACT}/></body>' for i in range(MAX_CUBES))
        xml = f"""<mujoco>
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <statistic extent="2" center="0 0 0"/>
  <visual>
    <global offwidth="{self.size[0]}" offheight="{self.size[1]}"/>
    <quality shadowsize="4096"/>
    <map znear="0.002" zfar="20"/>      <!-- x extent: 4 mm; the fingers come close to the wrist lenses -->
    <headlight ambient="0.45 0.45 0.45" diffuse="0.2 0.2 0.2" specular="0 0 0"/>
  </visual>
  <asset>
    {"".join(f'<mesh name="{mesh_name[f]}" file="{mesh_file(f)}" scale="{scale}"/>' for f, scale in meshes)}
    <texture name="table" type="2d" file="table.png"/>
    <material name="table" texture="table" specular="0.1" shininess="0.2"/>
    <material name="edge" rgba="0.55 0.45 0.33 1"/>
    <material name="arm" rgba="0.12 0.12 0.13 1" specular="0.3" shininess="0.4"/>
    <material name="cube" rgba="1 1 1 1" specular="0.05"/>
    <mesh name="tabletop" vertex="{x0} {y0} 0  {x1} {y0} 0  {x1} {y1} 0  {x0} {y1} 0
                                  {x0} {y0} -0.001  {x1} {y0} -0.001  {x1} {y1} -0.001  {x0} {y1} -0.001"
          face="0 1 2  0 2 3  4 6 5  4 7 6  0 4 5  0 5 1  1 5 6  1 6 2  2 6 7  2 7 3  3 7 4  3 4 0"
          texcoord="0 1  1 1  1 0  0 0  0 1  1 1  1 0  0 0"/>
  </asset>
  <worldbody>
    <light directional="true" pos="0 0.5 3" dir="-0.19 0.28 -0.94" diffuse="0.45 0.45 0.45" castshadow="true"/>
    <light directional="true" pos="0 -0.5 3" dir="0.3 -0.2 -0.93" diffuse="0.25 0.25 0.25" castshadow="false"/>
    <geom type="plane" pos="0 0 {FLOOR_Z}" size="4 4 0.1" rgba="0.18 0.26 0.16 1" contype="{TABLE}" conaffinity="{TABLE}"/>
    <geom type="mesh" mesh="tabletop" material="table" contype="0" conaffinity="0"/>
    <geom type="box" pos="{(x0 + x1) / 2} {(y0 + y1) / 2} {-0.001 - TABLE_THICKNESS / 2}"
          size="{(x1 - x0) / 2} {(y1 - y0) / 2} {TABLE_THICKNESS / 2}" material="edge" contype="0" conaffinity="0"/>
    <geom name="table" type="box" pos="{(x0 + x1) / 2} {(y0 + y1) / 2} {-TABLE_THICKNESS / 2}"
          size="{(x1 - x0) / 2} {(y1 - y0) / 2} {TABLE_THICKNESS / 2}" group="3"
          contype="{TABLE}" conaffinity="7" friction="0.6 0.005 0.0001" {CONTACT}/>
    {"".join(arms)}
    {cubes}
    {"".join(cams)}
  </worldbody>
  <contact>{"".join(excludes)}</contact>
  <actuator>{"".join(actuators)}</actuator>
</mujoco>"""
        ok, png = cv2.imencode(".png", table_bgr)
        self.model = m = mujoco.MjModel.from_xml_string(xml, {"table.png": png.tobytes()})
        self.data = mujoco.MjData(m)                                        # the renderer's copy of the state
        self.qadr = {ip: [m.joint(f"a{i}_joint_{k}").qposadr[0] for k in range(6)] for i, ip in enumerate(self.followers)}
        self.fingers = {ip: [m.joint(f"a{i}_{s}_carriage_joint").qposadr[0] for s in ("left", "right")]
                        for i, ip in enumerate(self.followers)}
        self.act = {ip: [m.actuator(f"a{i}_joint_{k}").id for k in range(6)] +
                    [m.actuator(f"a{i}_{s}_carriage").id for s in ("left", "right")] for i, ip in enumerate(self.followers)}
        self.pad_geoms = {ip: {m.geom(f"a{i}_gripper_{s}_c").id for s in ("left", "right")}
                          for i, ip in enumerate(self.followers)}
        self.cube_qadr = [m.joint(f"cube{i}").qposadr[0] for i in range(MAX_CUBES)]
        self.cube_dadr = [m.joint(f"cube{i}").dofadr[0] for i in range(MAX_CUBES)]
        self.dadr = {ip: [m.joint(f"a{i}_joint_{k}").dofadr[0] for k in range(6)] for i, ip in enumerate(self.followers)}
        self.cube_geom = [m.geom(f"cube{i}").id for i in range(MAX_CUBES)]
        self.cam_id = {serial: m.camera(f"c{serial}").id for serial in cfg["cameras"]}
        self.tool_body = {ip: m.body(f"a{i}_ee_gripper_link").id for i, ip in enumerate(self.followers)}
        self._kin, self._kin_lock = mujoco.MjData(m), threading.Lock()      # forward kinematics, off the state
        self._thread = ThreadPoolExecutor(1, thread_name_prefix="render")   # owns the GL context
        self._renderer = None

    def tool_pose(self, ip: str, q) -> np.ndarray:
        """The tool frame (ee_gripper_link) in W of follower `ip` at link angles q (6)."""
        with self._kin_lock:
            d = self._kin
            d.qpos[self.qadr[ip]] = q
            mujoco.mj_kinematics(self.model, d)
            b = self.tool_body[ip]
            return make_T(d.xmat[b].reshape(3, 3).copy(), d.xpos[b].copy())

    def close(self) -> None:
        def release():
            if self._renderer is not None:
                self._renderer.close()
                self._renderer = None
        self._thread.submit(release).result()
        self._thread.shutdown()

    def render(self, serial: str, T_W_cam: np.ndarray, qpos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(BGR uint8 image, depth in m along the optical axis, 0 where nothing) from camera `serial`
        at T_W_cam (optical frame: z forward, y down), of the state `qpos` (a physics snapshot)."""
        return self._thread.submit(self._render, serial, T_W_cam, qpos).result()

    def _render(self, serial, T_W_cam, qpos):
        m, d = self.model, self.data
        if self._renderer is None:
            self._renderer = mujoco.Renderer(m, self.size[1], self.size[0])
        d.qpos[:] = qpos
        c = self.cam_id[serial]
        m.cam_pos[c] = T_W_cam[:3, 3]
        m.cam_quat[c] = _quat(T_W_cam[:3, :3] @ np.diag([1.0, -1.0, -1.0]))   # MuJoCo cameras look down -z, y up
        mujoco.mj_kinematics(m, d)
        mujoco.mj_camlight(m, d)
        r = self._renderer
        r.update_scene(d, camera=c)                                # one scene for both passes
        rgb = r.render()
        r.enable_depth_rendering()
        depth = r.render()
        r.disable_depth_rendering()
        depth = np.where(depth > 0.95 * m.vis.map.zfar * m.stat.extent, 0.0, depth).astype(np.float32)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), depth
