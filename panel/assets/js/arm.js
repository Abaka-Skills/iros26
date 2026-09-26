/* WidowX AI assembled from the URDF-derived link/joint tree (see meshes/wxai/NOTICE.md).
   Adapted from IROS26: joints stay addressable so a live pose can be applied later. */
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

/* URDF rpy is fixed-axis X→Y→Z, i.e. R = Rz·Ry·Rx, which is three's 'ZYX' */
const euler = rpy => new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');

/* driver order: joint_0..joint_5, left_carriage_joint; the right carriage mirrors the left */
const DRIVER_JOINTS = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'left_carriage_joint'];

export async function loadArmAssets(dir, specFile) {
  const spec = await (await fetch(dir + specFile)).json();
  const loader = new STLLoader();
  const geos = {};
  await Promise.all([...new Set(Object.values(spec.links).map(l => l.mesh))]
    .map(async mesh => { geos[mesh] = await loader.loadAsync(dir + mesh); }));
  return { spec, geos };
}

export function buildArm({ spec, geos }, material, hide = []) {
  const links = {};
  const linkOf = name => (links[name] ||= new THREE.Group());
  for (const [name, l] of Object.entries(spec.links)) {
    if (hide.includes(name)) { linkOf(name); continue; }
    const m = new THREE.Mesh(geos[l.mesh], material);
    m.position.fromArray(l.xyz);
    m.rotation.copy(euler(l.rpy));
    m.scale.fromArray(l.scale);
    m.castShadow = m.receiveShadow = true;
    linkOf(name).add(m);
  }
  const joints = {};
  for (const j of spec.joints) {
    const frame = new THREE.Group();
    frame.position.fromArray(j.xyz);
    frame.rotation.copy(euler(j.rpy));
    const child = linkOf(j.child);
    if (j.axis) joints[j.name] = { child, type: j.type, axis: new THREE.Vector3().fromArray(j.axis).normalize() };
    frame.add(child);
    linkOf(j.parent).add(frame);
  }
  const root = new THREE.Group();               // URDF is Z-up, the scene is Y-up
  root.rotation.x = -Math.PI / 2;
  root.add(linkOf('base_link'));

  root.userData.setPose = q => {
    const vals = Object.fromEntries(DRIVER_JOINTS.map((n, i) => [n, q[i] ?? 0]));
    vals.right_carriage_joint = vals.left_carriage_joint;
    for (const [name, j] of Object.entries(joints)) {
      const v = vals[name] ?? 0;
      if (j.type === 'prismatic') j.child.position.copy(j.axis).multiplyScalar(v);
      else j.child.quaternion.setFromAxisAngle(j.axis, v);
    }
  };
  return root;
}
