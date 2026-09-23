/* Trossen WidowX AI（Stationary AI 套件里的 follower）——
   按官方 URDF 的关节原点装配 STL。网格与关节数据来自 TrossenRobotics/trossen_arm_description。 */
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

/* URDF 的 rpy 是绕固定轴 X→Y→Z，等价于 R = Rz·Ry·Rx，对应 three 的 'ZYX' */
const euler = rpy => new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');

export async function loadArms(cfg, material) {
  const spec = await (await fetch(cfg.dir + cfg.spec)).json();
  const loader = new STLLoader();
  const geos = {};
  await Promise.all([...new Set(Object.values(spec.links).map(l => l.mesh))]
    .map(async mesh => { geos[mesh] = await loader.loadAsync(cfg.dir + mesh); }));
  return cfg.arms.map(place => buildArm(spec, geos, material, cfg, place));
}

function buildArm(spec, geos, material, cfg, place) {
  const links = {};
  const linkOf = name => (links[name] ||= new THREE.Group());

  for (const [name, l] of Object.entries(spec.links)) {
    const m = new THREE.Mesh(geos[l.mesh], material);
    m.position.fromArray(l.xyz);
    m.rotation.copy(euler(l.rpy));
    m.scale.fromArray(l.scale);          // 网格是毫米，URDF 里按 0.001 缩到米
    m.castShadow = true;
    m.receiveShadow = true;
    linkOf(name).add(m);
  }

  for (const j of spec.joints) {
    const frame = new THREE.Group();     // 关节原点
    frame.position.fromArray(j.xyz);
    frame.rotation.copy(euler(j.rpy));
    const child = linkOf(j.child);
    const q = cfg.pose[j.name] ?? 0;
    if (j.axis) {
      const ax = new THREE.Vector3().fromArray(j.axis).normalize();
      if (j.type === 'prismatic') child.position.addScaledVector(ax, q);
      else child.quaternion.setFromAxisAngle(ax, q);
    }
    frame.add(child);
    linkOf(j.parent).add(frame);
  }

  const zUp = new THREE.Group();         // URDF 是 Z 向上，场景是 Y 向上
  zUp.rotation.x = -Math.PI / 2;
  zUp.add(linkOf('base_link'));
  zUp.scale.setScalar(cfg.scale);        // 米 -> 格

  /* 实机底座下面垫了 2 cm；高台本身属于工作台，画在场景里，不随机械臂显隐 */
  const lift = (cfg.lift || 0) * cfg.scale;          // 米 -> 格
  const holder = new THREE.Group();
  holder.position.set(place.x, lift, place.z);
  holder.rotation.y = place.yaw;
  holder.userData.restY = lift;
  holder.add(zUp);
  return holder;
}
