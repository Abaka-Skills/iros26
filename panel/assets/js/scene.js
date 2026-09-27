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

  /* W (from localize): axes at its origin and the line between the follower bases.
     W is z-up with y away from the operator; the scene is y-up with z toward the operator. */
  const W2S = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);
  const axes = new THREE.Group();
  axes.visible = false;
  const AXIS = [['x', [1, 0, 0], 0xc4513f], ['y', [0, 0, -1], 0x6f9c5c], ['z', [0, 1, 0], 0x4e87b0]];
  /* drawn last, over everything: in three's transparent pass (the table image is transparent too, and
     only renderOrder within the same pass counts), with no depth test */
  const onTop = m => {
    Object.assign(m.material, { transparent: true, depthTest: false, depthWrite: false, toneMapped: false });
    m.renderOrder = 10;
    return m;
  };
  for (const [, d, c] of AXIS) {                  // solid shaft + cone, drawn over the arms and the table image
    const mat = new THREE.MeshBasicMaterial({ color: c });
    const len = 0.14 * U, head = 0.035 * U;
    const shaft = onTop(new THREE.Mesh(new THREE.CylinderGeometry(0.0045 * U, 0.0045 * U, len - head, 12), mat));
    shaft.position.y = (len - head) / 2;
    const cone = onTop(new THREE.Mesh(new THREE.ConeGeometry(0.013 * U, head, 16), mat.clone()));
    cone.position.y = len - head / 2;
    const arrow = new THREE.Group();
    arrow.add(shaft, cone);
    arrow.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...d));
    axes.add(arrow);
  }
  const origin = onTop(new THREE.Mesh(new THREE.SphereGeometry(0.009 * U, 16, 12), new THREE.MeshBasicMaterial({ color: 0x2b2621 })));
  axes.add(origin);
  const baseLine = onTop(new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0x2b2621, dashSize: 0.15, gapSize: 0.1 })));
  axes.add(baseLine);
  scene.add(axes);

  function setCoordinates(doc) {
    axes.visible = !!doc;
    if (!doc) return;
    const ends = [];
    for (const side of ['left', 'right']) {
      const a = arms[`follower_${side}`];
      const T = new THREE.Matrix4().set(...(doc.T_W_mount ?? doc.T_W_base)[side].flat());   // level, physical pose
      const M = W2S.clone().multiply(T).multiply(W2S.clone().invert());
      const p = new THREE.Vector3(), q = new THREE.Quaternion();
      M.decompose(p, q, new THREE.Vector3());
      a.holder.position.copy(p.multiplyScalar(U));
      a.holder.quaternion.copy(q);
      ends.push(new THREE.Vector3(a.holder.position.x, 0.005, a.holder.position.z));
    }
    baseLine.geometry.setFromPoints(ends);
    baseLine.computeLineDistances();
    frame();
  }
  /* label anchors: each axis tip, and the origin ("W"), as [name, [x, y, z]] in scene units */
  /* Fixed-camera pixels draped on the table: a fine grid on the table plane whose UVs are each
     vertex projected into the camera (pose + intrinsics from localize), textured
     with its live frame. Unseen parts are discarded; pixels skip tone mapping. */
  let drape = null;
  const blank = new THREE.DataTexture(new Uint8Array([0, 0, 0, 0]), 1, 1);
  blank.needsUpdate = true;
  /* undistorted -> distorted normalised coordinates for a RealSense lens model (k1 k2 p1 p2 k3):
     Brown-Conrady distorts with the formula; the "inverse" model needs it inverted (iterated). */
  function distorter(c, model) {
    if (!c || !c.some(v => v)) return (x, y) => [x, y];
    const [k1, k2, p1, p2, k3] = c;
    const bc = (x, y) => {
      const r2 = x * x + y * y, f = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
      return [x * f + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * f + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y];
    };
    if (model !== 'inverse_brown_conrady') return bc;
    return (x, y) => {
      let [a, b] = [x, y];
      for (let i = 0; i < 20; i++) { const [fx, fy] = bc(a, b); a += x - fx; b += y - fy; }
      return [a, b];
    };
  }

  function setDrape(cal) {
    if (drape) { scene.remove(drape); drape.geometry.dispose(); drape = null; }
    if (!cal) return;
    const geo = new THREE.PlaneGeometry(R.table.w * U, R.table.d * U, 120, 76);
    geo.rotateX(-Math.PI / 2);                       // table plane: scene y = 0 (W z = 0)
    const Tw = cal.T_W_cam, [[fx, , cx], [, fy, cy]] = cal.K, [iw, ih] = cal.size;
    const lens = distorter(cal.dist, cal.model);
    const Rw = [[Tw[0][0], Tw[1][0], Tw[2][0]], [Tw[0][1], Tw[1][1], Tw[2][1]], [Tw[0][2], Tw[1][2], Tw[2][2]]];  // R^T
    const t = [Tw[0][3], Tw[1][3], Tw[2][3]];
    const pos = geo.attributes.position, uv = geo.attributes.uv;
    for (let i = 0; i < pos.count; i++) {
      const p = [pos.getX(i) / U - t[0], -pos.getZ(i) / U - t[1], -t[2]];   // W point minus camera centre
      const c = Rw.map(r => r[0] * p[0] + r[1] * p[1] + r[2] * p[2]);       // in the camera frame
      if (c[2] <= 0) { uv.setXY(i, -1, -1); continue; }
      const [xd, yd] = lens(c[0] / c[2], c[1] / c[2]);
      uv.setXY(i, (fx * xd + cx) / iw, (fy * yd + cy) / ih);
    }
    const mat = new THREE.MeshBasicMaterial({ map: blank, transparent: true, depthWrite: false, toneMapped: false });
    mat.onBeforeCompile = sh => {
      sh.fragmentShader = sh.fragmentShader.replace('#include <map_fragment>',
        'if (vMapUv.x < 0.0 || vMapUv.x > 1.0 || vMapUv.y < 0.0 || vMapUv.y > 1.0) discard;\n#include <map_fragment>');
    };
    drape = new THREE.Mesh(geo, mat);
    drape.position.y = 0.002 * U;                    // 2 mm above the board: no z-fighting
    drape.renderOrder = 1;
    scene.add(drape);
  }
  function setDrapeImage(bitmap) {
    if (!drape) { bitmap.close?.(); return; }
    const old = drape.material.map;
    const tex = new THREE.Texture(bitmap);
    tex.flipY = false;                               // ImageBitmap: v = 0 is the top row
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.needsUpdate = true;
    drape.material.map = tex;
    if (old !== blank) { old.image?.close?.(); old.dispose(); }
  }

  /* a point in W (metres, z up) as a scene position (units, y up): (x, y, z) -> (x, z, -y) */
  const w2s = ([x, y, z]) => new THREE.Vector3(x * U, z * U, -y * U);

  /* Topic overlays. Cubes (locate_cubes) in clay, in the sandbox's colours; a cube hidden under
     a stack (no colour known) is translucent grey. W -> scene: (x, y, z) -> (x, z, -y). */
  const cubeGroup = new THREE.Group(), cubeMats = {};
  scene.add(cubeGroup);
  const CUBE = cfg.cubes, cubeGeo = new RoundedBoxGeometry(...Array(3).fill(CUBE.size * U * 0.96), 3, CUBE.size * U * 0.12);
  const cubeMat = c => cubeMats[c ?? '_hidden'] ??= c ? clay(CUBE.colors[c] ?? '#999')
    : new THREE.MeshPhysicalMaterial({ color: CUBE.hidden, roughness: 1, transparent: true, opacity: 0.45, depthWrite: false });
  let lastCubes = '';
  function setCubes(list) {
    const key = JSON.stringify(list ?? []);
    if (key === lastCubes) return;
    lastCubes = key;
    cubeGroup.clear();                              // meshes share one geometry and cached materials
    for (const c of list ?? []) {
      const m = new THREE.Mesh(cubeGeo, cubeMat(c.color));
      m.position.copy(w2s([c.x, c.y, c.z]));
      m.rotation.y = (c.yaw ?? 0) * D2R;
      m.castShadow = m.receiveShadow = c.visible;
      cubeGroup.add(m);
    }
  }
  /* Cameras (cam_params): a small frustum glyph at each camera's pose (tinted image plane, a tick
     on its top edge), depth-tested like the rig. Labels come from cameraTips(). */
  const frusta = new THREE.Group();
  const glyphMat = new THREE.LineBasicMaterial({ color: 0x2b2621, transparent: true, opacity: 0.85 });
  const planeMat = new THREE.MeshBasicMaterial({ color: 0x4e87b0, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false });
  let camTips = [], lastCams = '';
  scene.add(frusta);
  function setFrusta(cams) {
    const key = JSON.stringify(Object.values(cams ?? {}).map(c => c.T_W_cam));
    if (key === lastCams) return;                   // fixed cameras and still wrists: nothing moved
    lastCams = key;
    frusta.children.forEach(o => o.geometry.dispose());
    frusta.clear();
    camTips = [];
    for (const [key, c] of Object.entries(cams ?? {})) {
      if (!c.T_W_cam || !c.K) continue;
      const T = c.T_W_cam, [[fx, , cx], [, fy, cy]] = c.K, [w, h] = c.size;
      const W = p => [0, 1, 2].map(i => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
      const px = [[0, 0], [w, 0], [w, h], [0, h]].map(([u, v]) => [(u - cx) / fx, (v - cy) / fy, 1]);
      const o = w2s(W([0, 0, 0])), L = 0.06;                                   // glyph depth (m)
      const near = px.map(r => w2s(W(r.map(x => x * L))));
      const tick = w2s(W([0, (0 - cy) / fy * L * 1.35, L]));                   // above the top edge
      const pts = near.flatMap((q, i) => [o, q, q, near[(i + 1) % 4]]);
      pts.push(near[0], tick, tick, near[1]);
      frusta.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), glyphMat));
      const quad = new THREE.BufferGeometry().setFromPoints([near[0], near[1], near[2], near[0], near[2], near[3]]);
      frusta.add(new THREE.Mesh(quad, planeMat));
      camTips.push([key, [o.x, o.y, o.z]]);
    }
  }
  const cameraTips = () => camTips;

  const axisTips = () => axes.visible
    ? [...AXIS.map(([name, d]) => [name, d.map(v => v * 0.17 * U)]), ['W', [0, 0, 0]]] : [];

  const cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 500);

  /* View: azimuth turns in 90 deg steps (accumulating), elevation switches between presets;
     both follow critically damped springs, and the rig is refitted every frame. */
  const CAMCFG = cfg.camera, HOME_AZ = CAMCFG.azimuth * D2R;
  const EL = { home: CAMCFG.elevation * D2R, top: CAMCFG.topElevation * D2R, front: CAMCFG.frontElevation * D2R };
  const view = { az: { v: HOME_AZ, vel: 0 }, el: { v: EL.home, vel: 0 } };
  const goal = { az: HOME_AZ, el: EL.home };
  let animating = false, lastT = 0;
  let zoom = 1, pan = { x: 0, y: 0 }, spp = 1;   // free camera: zoom (<1 closer), pan (view units); world per px
  const spring = (s, target, k, dt) => { s.vel += ((target - s.v) * k - s.vel * 2 * Math.sqrt(k)) * dt; s.v += s.vel * dt; };
  const nearest = (target, cur) => cur + ((((target - cur + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI;
  function tick(t) {
    const dt = Math.min((t - lastT) / 1000 || 1 / 60, 1 / 30);
    lastT = t;
    spring(view.az, goal.az, CAMCFG.springStiffness, dt);
    spring(view.el, goal.el, CAMCFG.springStiffness, dt);
    const moving = Math.abs(view.az.v - goal.az) + Math.abs(view.el.v - goal.el) > 1e-4
      || Math.abs(view.az.vel) + Math.abs(view.el.vel) > 1e-4;
    if (!moving) Object.assign(view, { az: { v: goal.az, vel: 0 }, el: { v: goal.el, vel: 0 } });
    frame();
    if (moving) requestAnimationFrame(tick); else animating = false;
  }
  function viewState() {
    const turned = Math.abs(nearest(HOME_AZ, goal.az) - goal.az) > 1e-6 || Math.abs(nearest(goal.az, HOME_AZ) - HOME_AZ) > 1e-6;
    const moved = zoom !== 1 || pan.x || pan.y;
    return { home: !turned && !moved && goal.el === EL.home, top: goal.el === EL.top, front: goal.el === EL.front, free };
  }
  function setView(action) {
    if (action === 'left') goal.az -= Math.PI / 2;
    else if (action === 'right') goal.az += Math.PI / 2;
    else if (action === 'home') { goal.az = nearest(HOME_AZ, goal.az); goal.el = EL.home; zoom = 1; pan = { x: 0, y: 0 }; }
    else if (action === 'top' || action === 'front') goal.el = goal.el === EL[action] ? EL.home : EL[action];
    if (!animating) { animating = true; lastT = performance.now(); requestAnimationFrame(tick); }
    return viewState();
  }
  let insets = { l: 0, r: 0, t: 0, b: 0 };

  /* Pose each arm from live joint positions; an arm that is absent or lost is a ghost at rest.
     Materials only change when an arm's liveness does. */
  function setStatus(st) {
    for (const [name, a] of Object.entries(arms)) {
      const s = st.arms[name];
      const live = !!(s && s.connected && !s.lost && s.pos);
      if (!a.arm || a.live !== live) {
        const role = name.startsWith('leader') ? 'leader' : 'follower';
        if (a.arm) a.holder.remove(a.arm);
        a.arm = buildArm(assets[role === 'leader' ? name : 'follower'], live ? mats[role] : mats.ghost);
        a.arm.traverse(o => { if (o.isMesh) o.castShadow = live; });
        a.holder.add(a.arm);
        a.live = live;
        if (!live) a.arm.userData.setPose([0, 0, 0, 0, 0, 0, 0]);
      }
      if (live) a.arm.userData.setPose(s.pos);
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
    const az = view.az.v, el = view.el.v;
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
    const s = Math.max((hi.x - lo.x) / fw, (hi.y - lo.y) / fh) * cfg.camera.padding * zoom;   // world per px
    spp = s;
    const cx = (lo.x + hi.x) / 2 + pan.x, cy = (lo.y + hi.y) / 2 + pan.y;
    cam.left = cx - (insets.l + fw / 2) * s; cam.right = cam.left + w * s;
    cam.top = cy + (insets.t + fh / 2) * s; cam.bottom = cam.top - h * s;
    cam.updateProjectionMatrix();
    render();
  }

  function render() { renderer.render(scene, cam); host.dispatchEvent(new Event('rendered')); }

  /* Free camera on the canvas: drag orbits, right- or shift-drag pans, wheel zooms, double-click
     returns home. Off by default so a stray drag can't swing the view. */
  let free = false, drag = null;
  const MIN_EL = 3 * D2R, MAX_EL = 88 * D2R;
  canvas.addEventListener('pointerdown', e => {
    if (!free) return;
    drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey };
    canvas.setPointerCapture(e.pointerId);
    canvas.style.cursor = drag.pan ? 'move' : 'grabbing';
  });
  canvas.addEventListener('pointermove', e => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    drag.x = e.clientX; drag.y = e.clientY;
    if (drag.pan) { pan.x -= dx * spp; pan.y += dy * spp; }
    else {
      goal.az -= dx * 0.006;
      goal.el = Math.min(MAX_EL, Math.max(MIN_EL, goal.el + dy * 0.006));
      view.az = { v: goal.az, vel: 0 }; view.el = { v: goal.el, vel: 0 };
    }
    frame();
    host.dispatchEvent(new Event('viewchange'));
  });
  const endDrag = () => { drag = null; canvas.style.cursor = free ? 'grab' : ''; };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);
  canvas.addEventListener('contextmenu', e => { if (free) e.preventDefault(); });
  canvas.addEventListener('wheel', e => {
    if (!free) return;
    e.preventDefault();
    zoom = Math.min(3, Math.max(0.15, zoom * Math.exp(e.deltaY * 0.0015)));
    frame();
    host.dispatchEvent(new Event('viewchange'));
  }, { passive: false });
  canvas.addEventListener('dblclick', () => { if (free) { setView('home'); host.dispatchEvent(new Event('viewchange')); } });
  function setFree(on) { free = on; canvas.style.cursor = on ? 'grab' : ''; return viewState(); }

  /* screen position (px, relative to host) just above an arm's highest point */
  function project(name) {
    const box = new THREE.Box3().setFromObject(arms[name].holder);
    const p = box.getCenter(new THREE.Vector3()).setY(box.max.y + 0.3).project(cam);
    return { x: (p.x + 1) / 2 * host.clientWidth, y: (1 - p.y) / 2 * host.clientHeight };
  }
  function projectPoint(xyz) {
    const p = new THREE.Vector3(...xyz).project(cam);
    return { x: (p.x + 1) / 2 * host.clientWidth, y: (1 - p.y) / 2 * host.clientHeight };
  }

  setStatus(status);
  setTheme(themeName);
  new ResizeObserver(frame).observe(host);
  return {
    setTheme, update: s => { setStatus(s); render(); }, project, projectPoint, axisTips, setView, viewState, setFree,
    setDrape: cal => { setDrape(cal); render(); }, setDrapeImage: b => { setDrapeImage(b); render(); },
    setCoordinates: doc => { setCoordinates(doc); render(); },
    /* overlays change the scene only; the next status frame (~30 Hz) draws them, so a fast
       topic stream can't add renders of its own */
    setCubes, setFrusta, cameraTips,
    setInsets(i) { insets = { ...insets, ...i }; frame(); },
    get arms() { return arms; },
  };
}
