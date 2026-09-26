/* The rig in soft clay: table, leader stand, four arms posed from status.
   Orthographic, fixed azimuth, rendered on demand (the board is static). */
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { buildArm } from './arm.js';

const D2R = Math.PI / 180;

export function makeScene(host, cfg, assets, status, themeName, { pixelRatio = Math.min(devicePixelRatio, 2) } = {}) {
  const U = cfg.unitsPerMeter, R = cfg.rig;
  const canvas = document.createElement('canvas');
  host.prepend(canvas);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(pixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;

  const scene = new THREE.Scene();
  scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;

  const clay = color => new THREE.MeshPhysicalMaterial({
    color, roughness: cfg.material.roughness, metalness: 0,
    sheen: cfg.material.sheen, sheenRoughness: 0.95, sheenColor: 0xffffff,
  });
  const mats = {
    board: clay('#fff'), stand: clay('#fff'), follower: clay('#fff'), leader: clay('#fff'),
    ghost: new THREE.MeshBasicMaterial({ transparent: true, depthWrite: false }),
  };

  /* table and leader stand: bevelled slabs whose top sits at y = 0 */
  const slab = (s, mat) => {
    const t = s.thickness * U;
    const m = new THREE.Mesh(new RoundedBoxGeometry(s.w * U, t, s.d * U, 4, Math.min(t / 2 - 1e-3, cfg.material.bevel * 3)), mat);
    m.position.set(0, -t / 2, s.z * U);
    m.receiveShadow = m.castShadow = true;
    scene.add(m);
  };
  slab(R.table, mats.board);
  slab(R.stand, mats.stand);

  const floor = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.ShadowMaterial());
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -Math.max(R.table.thickness, R.stand.thickness) * U - 0.01;
  floor.receiveShadow = true;
  scene.add(floor);

  /* arms */
  const arms = {};
  for (const [name, place] of Object.entries(R.arms)) {
    const holder = new THREE.Group();
    holder.position.set(place.x * U, R.lift * U, place.z * U);
    holder.rotation.y = place.yaw * D2R;
    holder.scale.setScalar(U);
    scene.add(holder);
    arms[name] = { holder, place, arm: null };
  }

  /* lights: warm key + hemisphere + weak fill (scene.md) */
  const key = new THREE.DirectionalLight();
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.radius = 7; key.shadow.bias = -0.0009; key.shadow.normalBias = 0.035;
  Object.assign(key.shadow.camera, { left: -11, right: 11, top: 11, bottom: -11, near: 1, far: 80 });
  key.position.set(9, 16, 11);
  scene.add(key, key.target);
  const hemi = new THREE.HemisphereLight();
  const fill = new THREE.DirectionalLight(0xffffff);
  fill.position.set(-8, 5, 9);
  scene.add(hemi, fill);

  const cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 500);
  let insets = { l: 0, r: 0, t: 0, b: 0 };

  function setStatus(st) {
    for (const [name, a] of Object.entries(arms)) {
      const s = st.arms[name];
      const live = s && s.connected;
      const role = name.startsWith('leader') ? 'leader' : 'follower';
      if (a.arm) a.holder.remove(a.arm);
      a.arm = buildArm(assets, live ? mats[role] : mats.ghost, role === 'leader' ? R.hideOnLeader : []);
      a.arm.userData.setPose(live ? s.pos : [0, 0, 0, 0, 0, 0, 0]);
      a.arm.traverse(o => { if (o.isMesh) o.castShadow = live; });
      a.holder.add(a.arm);
    }
  }

  function setTheme(name) {
    const t = cfg.themes[name];
    scene.background = new THREE.Color(t.bg);
    mats.board.color.set(t.board); mats.stand.color.set(t.stand);
    mats.follower.color.set(t.follower); mats.leader.color.set(t.leader);
    mats.ghost.color.set(t.ghost); mats.ghost.opacity = t.ghostOpacity;
    key.color.set(t.key); key.intensity = t.keyIntensity;
    hemi.color.set(t.hemiSky); hemi.groundColor.set(t.hemiGround); hemi.intensity = t.hemiIntensity;
    fill.intensity = t.fill;
    scene.environmentIntensity = t.env;
    floor.material.opacity = t.shadow; floor.material.color.set(t.shadowColor);
    render();
  }

  /* Fit the rig's bounding box into the part of the canvas not covered by panels. */
  function frame() {
    const w = host.clientWidth, h = host.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    const az = cfg.camera.azimuth * D2R, el = cfg.camera.elevation * D2R;
    const box = new THREE.Box3();
    Object.values(arms).forEach(a => box.expandByObject(a.holder));
    box.expandByPoint(new THREE.Vector3(-R.table.w * U / 2, 0, -R.table.d * U / 2));
    box.expandByPoint(new THREE.Vector3(R.table.w * U / 2, 0, R.stand.z * U + R.stand.d * U / 2));
    const c = box.getCenter(new THREE.Vector3());
    cam.position.copy(c).add(new THREE.Vector3(Math.cos(el) * Math.sin(az), Math.sin(el), Math.cos(el) * Math.cos(az)).multiplyScalar(160));
    cam.lookAt(c);
    cam.updateMatrixWorld();
    const lo = new THREE.Vector2(Infinity, Infinity), hi = new THREE.Vector2(-Infinity, -Infinity);
    for (let i = 0; i < 8; i++) {
      const p = new THREE.Vector3(i & 1 ? box.max.x : box.min.x, i & 2 ? box.max.y : box.min.y, i & 4 ? box.max.z : box.min.z)
        .applyMatrix4(cam.matrixWorldInverse);
      lo.min(p); hi.max(p);
    }
    const fw = Math.max(40, w - insets.l - insets.r), fh = Math.max(40, h - insets.t - insets.b);
    const s = Math.max((hi.x - lo.x) / fw, (hi.y - lo.y) / fh) * cfg.camera.padding;   // world per px
    const cx = (lo.x + hi.x) / 2, cy = (lo.y + hi.y) / 2;
    cam.left = cx - (insets.l + fw / 2) * s; cam.right = cam.left + w * s;
    cam.top = cy + (insets.t + fh / 2) * s; cam.bottom = cam.top - h * s;
    cam.updateProjectionMatrix();
    render();
  }

  function render() { renderer.render(scene, cam); host.dispatchEvent(new Event('rendered')); }

  /* screen position (px, relative to host) just above an arm's highest point */
  function project(name) {
    const box = new THREE.Box3().setFromObject(arms[name].holder);
    const p = box.getCenter(new THREE.Vector3()).setY(box.max.y + 0.3).project(cam);
    return { x: (p.x + 1) / 2 * host.clientWidth, y: (1 - p.y) / 2 * host.clientHeight };
  }

  setStatus(status);
  setTheme(themeName);
  new ResizeObserver(frame).observe(host);
  return {
    setTheme, setStatus: s => { setStatus(s); render(); }, project,
    setInsets(i) { insets = { ...insets, ...i }; frame(); },
    get arms() { return arms; },
  };
}
