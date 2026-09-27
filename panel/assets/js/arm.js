/* Trossen WidowX AI (the follower of the Stationary AI kit), assembled from the URDF-derived
   link/joint tree; meshes and joints from TrossenRobotics/trossen_arm_description (see
   meshes/wxai/NOTICE.md). Shared by the operator panel and the visitor page. */
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

/* URDF rpy is fixed-axis X→Y→Z, i.e. R = Rz·Ry·Rx, which is three's 'ZYX' */
const euler = rpy => new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');

/* driver order: joint_0..joint_5, left_carriage_joint; the right carriage mirrors the left */
const DRIVER_JOINTS = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'left_carriage_joint'];

/* One arm model: its link/joint spec plus geometries. Pass one `geos` cache to several calls
   (follower and leaders) and shared links load once. */
export async function loadArmAssets(dir, specFile, geos = {}) {
  const spec = await (await fetch(dir + specFile)).json();
  const loader = new STLLoader();
  await Promise.all([...new Set(Object.values(spec.links).map(l => l.mesh))]
    .map(async mesh => { geos[mesh] ??= loader.loadAsync(dir + mesh); geos[mesh] = await geos[mesh]; }));
  return { spec, geos };
}

/* One arm in URDF metres, Y-up. userData.setJoints({name: value}) and userData.setPose(driverOrder)
   pose it; links listed in `hide` get no mesh. */
export function buildArm({ spec, geos }, material, hide = []) {
  const links = {};
  const linkOf = name => (links[name] ||= new THREE.Group());
  for (const [name, l] of Object.entries(spec.links)) {
    if (hide.includes(name)) { linkOf(name); continue; }
    const m = new THREE.Mesh(geos[l.mesh], material);
    m.name = name;                                   // lets measurements find a link by name
    m.position.fromArray(l.xyz);
    m.rotation.copy(euler(l.rpy));
    m.scale.fromArray(l.scale);                      // meshes are in mm; the URDF scales by 0.001
    m.castShadow = m.receiveShadow = true;
    linkOf(name).add(m);
  }
  const joints = {};
  for (const j of spec.joints) {
    const frame = new THREE.Group();                 // joint origin
    frame.position.fromArray(j.xyz);
    frame.rotation.copy(euler(j.rpy));
    const child = linkOf(j.child);
    if (j.axis) joints[j.name] = { child, type: j.type, axis: new THREE.Vector3().fromArray(j.axis).normalize() };
    frame.add(child);
    linkOf(j.parent).add(frame);
  }
  const root = new THREE.Group();                    // URDF is Z-up, the scene is Y-up
  root.rotation.x = -Math.PI / 2;
  root.add(linkOf('base_link'));

  root.userData.setJoints = vals => {
    for (const [name, j] of Object.entries(joints)) {
      const v = vals[name] ?? 0;
      if (j.type === 'prismatic') j.child.position.copy(j.axis).multiplyScalar(v);
      else j.child.quaternion.setFromAxisAngle(j.axis, v);
    }
  };
  root.userData.setPose = q => {
    const vals = Object.fromEntries(DRIVER_JOINTS.map((n, i) => [n, q[i] ?? 0]));
    vals.right_carriage_joint = vals.left_carriage_joint;
    root.userData.setJoints(vals);
  };
  return root;
}

/* The visitor page's arms: one holder per `cfg.arms` place, posed from `cfg.pose`, scaled from
   metres to grid cells and lifted onto the 2 cm plate the real bases stand on. */
export async function loadArms(cfg, material) {
  const assets = await loadArmAssets(cfg.dir, cfg.spec);
  return cfg.arms.map(place => {
    const arm = buildArm(assets, material);
    arm.userData.setJoints(cfg.pose);
    arm.scale.setScalar(cfg.scale);
    const lift = (cfg.lift || 0) * cfg.scale;
    const holder = new THREE.Group();
    holder.position.set(place.x, lift, place.z);
    holder.rotation.y = place.yaw;
    holder.userData.restY = lift;
    holder.add(arm);
    return holder;
  });
}
