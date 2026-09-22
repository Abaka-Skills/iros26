import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { STRINGS } from './i18n.js';
import { loadArms } from './arm.js';

/* ============================ 语言 ============================ */
const DEFAULT_LANG = 'en';                           // 首次访问的默认语言
let lang = localStorage.getItem('cs-lang') || DEFAULT_LANG;
let L = STRINGS[lang];

/* ============================ config ============================ */
let CFG;
try {
  CFG = await (await fetch('./config.json', { cache: 'no-store' })).json();
} catch (err) {
  for (const id of ['needTitle', 'needBody', 'needAlt']) document.getElementById(id).textContent = L[id];
  document.getElementById('needserver').hidden = false;
  throw err;
}
const G = CFG.grid, CUBES = CFG.cubes, IX = CFG.interaction, CAM = CFG.camera;

/* ============================ easing ============================ */
const clamp = (v, a, b) => v < a ? a : v > b ? b : v;
const easeOutCubic = k => 1 - Math.pow(1 - k, 3);
const easeOutBack  = k => { const c = 1.9; return 1 + (c + 1) * Math.pow(k - 1, 3) + c * Math.pow(k - 1, 2); };

/* 把目标角折算到离当前角最近的等价角，复位时就走最短那条路，而不是把转过的圈退回去 */
const TAU = Math.PI * 2;
const nearestAngle = (target, current) =>
  current + (((target - current + Math.PI) % TAU) + TAU) % TAU - Math.PI;

/* 临界阻尼弹簧：所有「灵动」的来源 */
function springStep(s, target, stiffness, dt) {
  const d = 2 * Math.sqrt(stiffness);
  s.vel += ((target - s.v) * stiffness - s.vel * d) * dt;
  s.v += s.vel * dt;
}
const S = (v = 0) => ({ v, vel: 0 });

/* 微型补间系统：fn(k) 收到 0..1 的原始进度，自己挑缓动 */
const tweens = [];
function tween(dur, fn, onDone) {
  const w = { t: 0, dur, fn, onDone };
  tweens.push(w);
  return w;
}
function cancelTween(w) {
  const i = w ? tweens.indexOf(w) : -1;
  if (i >= 0) tweens.splice(i, 1);
}
function stepTweens(dt) {
  for (let i = tweens.length - 1; i >= 0; i--) {
    const w = tweens[i];
    w.t += dt;
    const k = Math.min(w.t / w.dur, 1);
    w.fn(k);
    if (k >= 1) { tweens.splice(i, 1); w.onDone && w.onDone(); }
  }
}
const pulse = (mesh, amp = 0.07, dur = 0.32) =>          // 只给落点框用
  tween(dur, k => mesh.scale.setScalar(1 + amp * Math.sin(Math.PI * k) * (1 - k)));
/* 方块只在位置上弹一下，不做拉伸变形 */
const hop = (mesh, amp = 0.09, dur = 0.28) => {
  const y0 = mesh.userData.baseY ?? mesh.position.y;
  tween(dur, k => { mesh.position.y = y0 + Math.sin(Math.PI * k) * amp * (1 - k); });
};

/* ============================ 渲染器 / 场景 ============================ */
const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;

const scene = new THREE.Scene();
scene.environment = new THREE.PMREMGenerator(renderer)
  .fromScene(new RoomEnvironment(), 0.04).texture;

const keyLight = new THREE.DirectionalLight(0xffffff, 1);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.radius = 7;
keyLight.shadow.bias = -0.0009;
keyLight.shadow.normalBias = 0.035;
const hemi = new THREE.HemisphereLight(0xffffff, 0x000000, 1);
const fill = new THREE.DirectionalLight(0xffffff, 0.3);
fill.position.set(-14, 9, -10);
scene.add(keyLight, keyLight.target, hemi, fill);

/* 两台 ALOHA 的间距按实测摆：gapMeters 是两个底座相对边缘之间的净距，
   加上底座中心到该侧边缘的距离，才是中心距。缩放同样由方块边长推出来。 */
const R = CFG.robots;
const ARM_SCALE = 1 / G.cellMeters;                  // 1 米 = 多少格
const ARM_GAP = R ? (R.gapMeters + 2 * R.baseFrontEdge) / G.cellMeters : 0;   // 中心距（格）
const ARM_PLACES = R ? [
  { x: G.x / 2 - ARM_GAP / 2, z: R.zFrac * G.z, yaw: 0 },
  { x: G.x / 2 + ARM_GAP / 2, z: R.zFrac * G.z, yaw: Math.PI }
] : [];
/* 臂座下面那块方格挖空，跟着臂座一起走 */
const EXCLUDE = ARM_PLACES.map(a => ({
  x: Math.round(a.x - R.baseCells / 2), z: Math.round(a.z - R.baseCells / 2),
  w: R.baseCells, d: R.baseCells
}));

/* ---- 底板 + 网格线 ---- */
const B = CFG.board;
const slab = new THREE.Mesh(                        // 台面比网格大一圈，两侧留出放机械臂的地方
  new RoundedBoxGeometry(G.x + 2 * B.marginX, B.thickness, G.z + 2 * B.marginZ, 4, 0.24),
  new THREE.MeshPhysicalMaterial({ roughness: 0.98, metalness: 0, sheen: 0.4, sheenRoughness: 0.95 })
);
slab.position.set(G.x / 2, -B.thickness / 2, G.z / 2);
slab.receiveShadow = true;

/* 网格铺满台面，但抠掉两个臂座占的方块；按格子收集边，避免重复画 */
const blocked = (i, j) => EXCLUDE.some(e => i >= e.x && i < e.x + e.w && j >= e.z && j < e.z + e.d);
const edges = new Set();
for (let i = 0; i < G.x; i++) {
  for (let j = 0; j < G.z; j++) {
    if (blocked(i, j)) continue;
    edges.add(`h:${i}:${j}`); edges.add(`h:${i}:${j + 1}`);
    edges.add(`v:${i}:${j}`); edges.add(`v:${i + 1}:${j}`);
  }
}
const pts = [];
for (const key of edges) {
  const [dir, a, b] = key.split(':');
  const i = +a, j = +b;
  if (dir === 'h') pts.push(i, 0, j, i + 1, 0, j);
  else pts.push(i, 0, j, i, 0, j + 1);
}
const gridMat = new THREE.LineBasicMaterial({ transparent: true, depthWrite: false });
const gridLines = new THREE.LineSegments(
  new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(pts, 3)),
  gridMat
);
gridLines.position.y = 0.005;
scene.add(gridLines);

const armMat = new THREE.MeshPhysicalMaterial({
  roughness: 0.55, metalness: 0.18, sheen: 0.25, sheenRoughness: 0.9
});
const armObjects = [];
let armsShown = localStorage.getItem('cs-arms') !== 'off';
if (CFG.robots) {
  loadArms({ ...R, scale: ARM_SCALE, arms: ARM_PLACES }, armMat, slab.material)   // 垫块跟台面同色
    .then(arms => arms.forEach(a => {
      a.visible = armsShown;
      armObjects.push(a);
      scene.add(a);
    }))
    .catch(e => console.warn('WidowX 加载失败：', e));
}

const pickRoot = new THREE.Group();
const blocksGroup = new THREE.Group();
pickRoot.add(slab, blocksGroup);
scene.add(pickRoot);

/* ---- 方块 ---- */
const cubeGeo = new RoundedBoxGeometry(1, 1, 1, 4, CFG.style.bevel);
const cubeMats = CUBES.map(c => new THREE.MeshPhysicalMaterial({
  color: new THREE.Color(c.color),
  roughness: CFG.style.roughness, metalness: 0,
  sheen: CFG.style.sheen, sheenRoughness: 0.95, sheenColor: 0xffffff
}));

const blocks = new Map();
const k3 = (i, k, j) => i + ',' + k + ',' + j;
const blockAt = (i, k, j) => blocks.get(k3(i, k, j));
const inBounds = (i, k, j) =>
  i >= 0 && j >= 0 && k >= 0 && i < G.x && j < G.z && k < G.maxHeight && !blocked(i, j);

/* ---- 落点指示板（磁吸的视觉反馈）---- */
/* 圆点描边：沿 12 条棱等距撒点，用圆形贴图，所以是圆角点不是方块 dash */
function dotTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.beginPath();
  g.arc(32, 32, 27, 0, Math.PI * 2);
  g.fillStyle = '#fff';
  g.fill();
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
function boxDots(size, spacing) {
  const edge = new THREE.EdgesGeometry(new THREE.BoxGeometry(size, size, size)).attributes.position.array;
  const a = new THREE.Vector3(), b = new THREE.Vector3(), p = new THREE.Vector3(), out = [];
  for (let i = 0; i < edge.length; i += 6) {
    a.set(edge[i], edge[i + 1], edge[i + 2]);
    b.set(edge[i + 3], edge[i + 4], edge[i + 5]);
    const n = Math.max(1, Math.round(a.distanceTo(b) / spacing));
    for (let k = 0; k <= n; k++) {
      p.copy(a).lerp(b, k / n);
      out.push(p.x, p.y, p.z);
    }
  }
  return new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(out, 3));
}
const plate = new THREE.Points(
  boxDots(1.02, CFG.style.markerSpacing),
  new THREE.PointsMaterial({
    map: dotTexture(), size: CFG.style.markerDot, sizeAttenuation: true,
    transparent: true, opacity: 0.9, alphaTest: 0.45, depthWrite: false
  })
);
plate.visible = false;
scene.add(plate);
const plateS = { x: S(), y: S(), z: S() };
/* 半透明的下落框：落点格里的方块投影 */
const ghost = new THREE.Mesh(cubeGeo, new THREE.MeshBasicMaterial({
  transparent: true, opacity: CFG.style.ghostOpacity, depthWrite: false
}));
ghost.visible = false;
scene.add(ghost);

/* ============================ 相机 ============================ */
const DEG = Math.PI / 180;
const view = {
  azim: S(45 * DEG), azimT: 45 * DEG,
  elev: S(CAM.elevations[0] * DEG), elevIdx: 0,
  zoom: S(CAM.zoom.default), zoomT: CAM.zoom.default,
  target: new THREE.Vector3(G.x / 2, 1.5, G.z / 2)
};
const cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 500);

/* 默认缩放按台面大小算出来，换台面尺寸不用再手动调 */
function fitZoom() {
  const aspect = (canvas.clientWidth || innerWidth) / (canvas.clientHeight || innerHeight);
  const span = (G.x + G.z) * Math.SQRT1_2;          // 等距投影下台面的屏幕跨度
  const e = CAM.elevations[view.elevIdx] * DEG;
  const h = Math.max(span / (2 * aspect), span * Math.sin(e) / 2);
  return clamp(h * CAM.fitPadding, CAM.zoom.min, CAM.zoom.max);
}

/* 关掉 ALOHA 就不用给机械臂留地方，视角自动收近一点 */
let zoomBase = CAM.zoom.default;
function applyZoom() {
  const f = armsShown ? 1 : CAM.armsOffZoom;
  view.zoomT = clamp(zoomBase * f, CAM.zoom.min, CAM.zoom.max);
}

function updateCamera(dt) {
  springStep(view.azim, view.azimT, CAM.springStiffness, dt);
  springStep(view.elev, CAM.elevations[view.elevIdx] * DEG, CAM.springStiffness, dt);
  springStep(view.zoom, view.zoomT, CAM.springStiffness, dt);

  const e = view.elev.v, a = view.azim.v;
  const dir = new THREE.Vector3(Math.cos(e) * Math.sin(a), Math.sin(e), Math.cos(e) * Math.cos(a));
  cam.position.copy(view.target).addScaledVector(dir, 160);
  cam.lookAt(view.target);

  const h = view.zoom.v, aspect = canvas.clientWidth / canvas.clientHeight;
  cam.left = -h * aspect; cam.right = h * aspect; cam.top = h; cam.bottom = -h;
  cam.updateProjectionMatrix();

  /* 主光跟着视野走，阴影贴图才不会被 40×60 撑糊 */
  keyLight.position.copy(view.target).add(new THREE.Vector3(16, 30, 13));
  keyLight.target.position.copy(view.target);
  const ext = h * 1.5, sc = keyLight.shadow.camera;
  sc.left = -ext; sc.right = ext; sc.top = ext; sc.bottom = -ext;
  sc.near = 1; sc.far = 90;
  sc.updateProjectionMatrix();
}

/* ============================ 主题 ============================ */
let THEME_LIFT = 0.16;            // 落点色的明度补偿：深色底提亮，浅色底压暗
let themeName = localStorage.getItem('cs-theme') || CFG.defaultTheme || 'light';
function applyTheme(name) {
  themeName = name;
  const t = CFG.themes[name];
  document.documentElement.dataset.theme = name;
  scene.background = new THREE.Color(t.bg);
  keyLight.color.set(t.key); keyLight.intensity = t.keyIntensity;
  hemi.color.set(t.hemiSky); hemi.groundColor.set(t.hemiGround); hemi.intensity = t.hemiIntensity;
  fill.intensity = t.fill;
  scene.environmentIntensity = t.env;
  slab.material.color.set(t.board);
  armMat.color.set(t.robot);
  gridMat.color.set(t.gridLine); gridMat.opacity = t.gridOpacity;
  THEME_LIFT = t.markerLift ?? (name === 'dark' ? 0.16 : -0.1);
  document.documentElement.style.setProperty('--vignette', t.vignette);
  document.documentElement.style.setProperty('--grain', t.grain);
  localStorage.setItem('cs-theme', name);
}

/* ============================ 手上的方块 ============================ */
const held = { mesh: null, type: 0, source: null, origin: null };
const hs = { x: S(), y: S(), z: S() };
const state = { tool: 'build', selected: 0, target: null, magnetic: false, hover: null, painting: false, alt: false };

function heldMaterial(type) {
  const m = cubeMats[type].clone();
  m.transparent = true;
  m.depthTest = false;                 // 手上的方块不被场景挡住
  return m;
}
function armPalette() {
  disposeHeld();
  const m = new THREE.Mesh(cubeGeo, heldMaterial(state.selected));
  m.castShadow = true;
  m.renderOrder = 999;
  m.material.opacity = 0;                            // 淡入，不做弹性缩放
  scene.add(m);
  held.mesh = m; held.type = state.selected; held.source = 'palette'; held.origin = null;
}
function disposeHeld() {
  if (!held.mesh) return;
  scene.remove(held.mesh);
  held.mesh.material.dispose();
  held.mesh = null; held.source = null;
}
function pickUp(mesh) {
  const c = mesh.userData.cell;
  blocks.delete(k3(c.i, c.k, c.j));
  blocksGroup.remove(mesh);
  disposeHeld();
  mesh.material = heldMaterial(mesh.userData.type);
  mesh.renderOrder = 999;
  scene.add(mesh);
  held.mesh = mesh; held.type = mesh.userData.type; held.source = 'grid'; held.origin = c;
  hs.x.v = mesh.position.x; hs.y.v = mesh.position.y; hs.z.v = mesh.position.z;
  hs.x.vel = hs.z.vel = 0; hs.y.vel = 4.2;           // 抓起来「弹」一下
  rippleNeighbors(c);
}
function placeHeld() {
  const t = state.target;
  if (!t || !t.valid || !held.mesh) return;
  const mesh = held.mesh, from = mesh.position.clone();
  const rot = { x: mesh.rotation.x, z: mesh.rotation.z };

  mesh.material.dispose();
  mesh.material = cubeMats[held.type];
  mesh.renderOrder = 0;
  mesh.castShadow = mesh.receiveShadow = true;
  mesh.position.set(t.i + 0.5, t.k + 0.5, t.j + 0.5);
  mesh.userData = { cell: { i: t.i, k: t.k, j: t.j }, type: held.type, baseY: t.k + 0.5 };
  blocksGroup.add(mesh);
  blocks.set(k3(t.i, t.k, t.j), mesh);

  const to = mesh.position.clone();
  const drop = Math.max(0, from.y - to.y);
  const dur = clamp(0.16 + 0.05 * drop, 0.16, 0.5);
  tween(dur, k => {
    const e = easeOutCubic(k), g = k * k;             // 水平缓出，垂直加速 —— 像掉下去
    mesh.position.x = from.x + (to.x - from.x) * e;
    mesh.position.z = from.z + (to.z - from.z) * e;
    mesh.position.y = from.y + (to.y - from.y) * g;
    mesh.rotation.x = rot.x * (1 - e);
    mesh.rotation.z = rot.z * (1 - e);
  }, () => {
    mesh.position.copy(to);
    hop(mesh, 0.12, 0.3);                             // 落地弹一下
    rippleNeighbors(t);
  });

  held.mesh = null; held.source = null;
  pulse(plate, 0.3, 0.32);
  if (state.tool === 'build') armPalette();
  updateCount();
}
function returnHeld() {                                // Esc：抓着的方块回原位
  if (held.source !== 'grid' || !held.origin) return;
  const o = held.origin;
  state.target = { i: o.i, k: o.k, j: o.j, valid: !blockAt(o.i, o.k, o.j) };
  placeHeld();
}
function removeBlock(mesh) {
  const c = mesh.userData.cell;
  blocks.delete(k3(c.i, c.k, c.j));
  const y0 = mesh.position.y;
  tween(0.26, k => {
    mesh.scale.setScalar(Math.max(0.001, 1 - k * k));
    mesh.position.y = y0 + 0.45 * k;
  }, () => { blocksGroup.remove(mesh); updateCount(); });
  rippleNeighbors(c);
  settleColumn(c);
}
/* 抽掉下面一块，上面连着的那一摞整体落一格 */
function settleColumn(c) {
  const falling = [];
  for (let k = c.k + 1; k < G.maxHeight; k++) {
    const m = blockAt(c.i, k, c.j);
    if (!m) break;
    falling.push(m);
  }
  falling.forEach((m, n) => {
    blocks.delete(k3(c.i, m.userData.cell.k, c.j));
    m.userData.cell = { i: c.i, k: m.userData.cell.k - 1, j: c.j };
    m.userData.baseY = m.userData.cell.k + 0.5;
    const from = m.position.y, to = m.userData.baseY;
    tween(0.3 + n * 0.02, k => {
      m.position.y = from + (to - from) * (k * k);   // 加速下落
    }, () => { m.position.y = to; hop(m, 0.07, 0.26); });
  });
  falling.forEach(m => blocks.set(k3(m.userData.cell.i, m.userData.cell.k, m.userData.cell.j), m));
}
function rippleNeighbors(c) {
  for (const [dx, dk, dz] of [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]) {
    const m = blockAt(c.i + dx, c.k + dk, c.j + dz);
    if (m) hop(m, 0.05, 0.26);
  }
}

/* ============================ 指针 / 落点 ============================ */
const ray = new THREE.Raycaster();
const ptr = new THREE.Vector2(-9, -9);
let pointerOver = false;

const pickNdc = new THREE.Vector2();
function pick() {
  if (!pointerOver) return null;
  pickNdc.copy(ptr);
  if (held.mesh) {                    // 手上有方块时，方块挂在光标下方
    const px = IX.cursorLead * Math.cos(view.elev.v) * (innerHeight / (2 * view.zoom.v));
    pickNdc.y -= (2 * px) / innerHeight;
  }
  ray.setFromCamera(pickNdc, cam);
  const hit = ray.intersectObject(pickRoot, true)[0];
  return hit || null;
}
function computeTarget(hit) {
  if (!hit) return null;
  let i, j;
  if (hit.object === slab) {
    if (hit.face.normal.y < 0.5) return null;
    i = Math.floor(hit.point.x);
    j = Math.floor(hit.point.z);
  } else {
    const c = hit.object.userData.cell;            // 指到方块就取它所在那一列
    i = c.i; j = c.j;
  }
  if (!inBounds(i, 0, j)) return { i, k: 0, j, valid: false };
  let k = 0;
  while (k < G.maxHeight && blockAt(i, k, j)) k++; // 一路堆到这一列的顶上
  return { i, k, j, valid: k < G.maxHeight };
}
function neighbourCount(t) {
  let n = 0;
  for (const [dx, dk, dz] of [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])
    if (blockAt(t.i + dx, t.k + dk, t.j + dz)) n++;
  return n;
}
function refreshTarget() {
  const hit = pick();
  const isBlock = hit && hit.object !== slab;
  const grabbing = (state.alt || state.tool === 'grab') && held.source !== 'grid';

  setHover((grabbing || state.tool === 'mine') && isBlock ? hit.object : null);

  const prev = state.target;
  const t = computeTarget(hit);
  state.target = t;
  state.magnetic = !!(t && t.valid && neighbourCount(t) > 0);

  const noPlate = grabbing || state.tool === 'mine';   // 抓取 / 镐子模式不显示落点
  if (t && (!prev || prev.i !== t.i || prev.k !== t.k || prev.j !== t.j)) {
    plate.visible = !noPlate;
    if (state.magnetic) pulse(plate, 0.16, 0.26);       // 吸住的一下「咔」
  }
  if (!t || noPlate) plate.visible = false;
  updateCursor(isBlock, grabbing);
}
const HOVER_LIFT = 0.14;
function liftTo(mesh, to, dur, ease) {
  cancelTween(mesh.userData.hoverTween);             // 抬起和落回不能同时跑，否则会卡在半路
  const from = mesh.position.y - mesh.userData.baseY;
  mesh.userData.hoverTween = tween(dur, k => {
    mesh.position.y = mesh.userData.baseY + from + (to - from) * ease(k);
  }, () => { mesh.position.y = mesh.userData.baseY + to; });
}
function setHover(mesh) {
  if (state.hover === mesh) return;
  const old = state.hover;
  if (old && old.parent) liftTo(old, 0, 0.2, easeOutCubic);
  state.hover = mesh;
  if (mesh) liftTo(mesh, HOVER_LIFT, 0.24, easeOutBack);
}
function updateCursor(isBlock, grabbing) {
  let c = 'cur-none';
  if (panning) c = 'cur-pan';
  else if (held.source === 'grid') c = 'cur-grabbing';
  else if (grabbing) c = isBlock ? 'cur-grab' : 'cur-none';
  else if (state.tool === 'mine') c = 'cur-mine';   // 整个删除模式都用垃圾桶光标
  else if (state.target) c = state.target.valid ? 'cur-grabbing' : 'cur-deny';
  canvas.className = c;
}

/* ============================ 输入 ============================ */
let panning = false, panPrev = null, spaceDown = false;

canvas.addEventListener('pointerenter', () => { pointerOver = true; });
canvas.addEventListener('pointerleave', () => {
  pointerOver = false; state.target = null; plate.visible = false; setHover(null);
});
canvas.addEventListener('pointermove', e => {
  ptr.x = (e.clientX / innerWidth) * 2 - 1;
  ptr.y = -(e.clientY / innerHeight) * 2 + 1;
  pointerOver = true;

  if (panning && panPrev) {
    const px = e.clientX - panPrev.x, py = e.clientY - panPrev.y;
    const scale = (2 * view.zoom.v) / innerHeight;
    const a = view.azim.v;
    const right = new THREE.Vector3(Math.cos(a), 0, -Math.sin(a));
    const fwd = new THREE.Vector3(Math.sin(a), 0, Math.cos(a));
    view.target.addScaledVector(right, -px * scale).addScaledVector(fwd, -py * scale / Math.sin(view.elev.v));
    view.target.x = clamp(view.target.x, -B.marginX, G.x + B.marginX);
    view.target.z = clamp(view.target.z, -B.marginZ, G.z + B.marginZ);
    panPrev = { x: e.clientX, y: e.clientY };
    return;
  }
  refreshTarget();
  if (state.painting && state.target && state.target.valid) placeHeld();
});
canvas.addEventListener('pointerdown', e => {
  try { canvas.setPointerCapture(e.pointerId); } catch { /* 捕获失败不该挡住这次点击 */ }
  if (e.button === 1 || (e.button === 0 && spaceDown)) {
    panning = true; panPrev = { x: e.clientX, y: e.clientY };
    e.preventDefault(); return;
  }
  if (e.button === 2) {                                  // 右键删除
    const hit = pick();
    if (hit && hit.object !== slab) { setHover(null); removeBlock(hit.object); }
    return;
  }
  if (e.button !== 0) return;

  if (state.tool === 'mine' && !state.alt) {             // 镐子：敲掉单个方块
    const hit = pick();
    if (hit && hit.object !== slab) { setHover(null); removeBlock(hit.object); }
    return;
  }
  if ((state.alt || state.tool === 'grab') && held.source !== 'grid') {   // Alt / 手形工具 抓取
    const hit = pick();
    if (hit && hit.object !== slab) { setHover(null); pickUp(hit.object); refreshTarget(); }
    return;
  }
  placeHeld();
  if (e.shiftKey) state.painting = true;                 // Shift 连放
});
addEventListener('pointerup', () => { panning = false; panPrev = null; state.painting = false; });
canvas.addEventListener('contextmenu', e => e.preventDefault());
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  zoomBase = clamp(zoomBase * (1 + Math.sign(e.deltaY) * 0.12), CAM.zoom.min, CAM.zoom.max);
  applyZoom();
}, { passive: false });

addEventListener('keydown', e => {
  if (e.key === 'Alt') { state.alt = true; refreshTarget(); }
  if (e.code === 'Space') { spaceDown = true; e.preventDefault(); }
  if (e.key === 'q' || e.key === 'Q') rotate(+1);
  if (e.key === 'e' || e.key === 'E') rotate(-1);
  if (e.key === 'r' || e.key === 'R') toggleTilt();
  if (e.key === 'Escape') returnHeld();
  if (e.key === '0') setTool('grab');
  if (e.key === 'x' || e.key === 'X') setTool('mine');
  if (e.key === 'h' || e.key === 'H') resetView();
  const n = parseInt(e.key, 10);
  if (n >= 1 && n <= CUBES.length) selectCube(n - 1);
});
addEventListener('keyup', e => {
  if (e.key === 'Alt') { state.alt = false; setHover(null); refreshTarget(); }
  if (e.code === 'Space') spaceDown = false;
});

/* ============================ UI ============================ */
const paletteEl = document.getElementById('palette');
CUBES.forEach((c, idx) => {
  const b = document.createElement('button');
  b.className = 'sw';
  b.setAttribute('role', 'radio');
  b.setAttribute('aria-checked', idx === 0);
  b.title = L.colorTip(L.colors[idx] || c.label, idx + 1);
  b.innerHTML = `<i style="background:${c.color}"></i><em>${idx + 1}</em>`;
  b.onclick = () => selectCube(idx);
  paletteEl.appendChild(b);
});

const GRAB_ICON = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor"
  stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
  <path d="M9 11.5V5.6a1.5 1.5 0 0 1 3 0v5.9"/>
  <path d="M12 11.5V4.6a1.5 1.5 0 0 1 3 0v6.9"/>
  <path d="M15 11.5V6.6a1.5 1.5 0 0 1 3 0V14"/>
  <path d="M9 11.5V9.6a1.5 1.5 0 0 0-3 0v4.3c0 3.2 2.4 5.9 5.6 6.1h1.3c3 0 5.1-2.4 5.1-5.4"/>
</svg>`;
const MINE_ICON = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor"
  stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
  <path d="M3.5 6h17"/>
  <path d="M8.5 6V4.6A1.6 1.6 0 0 1 10 3h4a1.6 1.6 0 0 1 1.5 1.6V6"/>
  <path d="M6.2 6l.9 13.4A1.7 1.7 0 0 0 8.8 21h6.4a1.7 1.7 0 0 0 1.7-1.6L17.8 6"/>
  <path d="M10.2 10.5v6M13.8 10.5v6"/>
</svg>`;
const grabBtn = document.createElement('button');
grabBtn.className = 'sw tool';
grabBtn.setAttribute('role', 'radio');
grabBtn.setAttribute('aria-checked', 'false');
grabBtn.title = L.tipGrab;
grabBtn.innerHTML = `<i>${GRAB_ICON}</i><em>0</em>`;
grabBtn.onclick = () => setTool('grab');
paletteEl.appendChild(grabBtn);

const mineBtn = document.createElement('button');
mineBtn.className = 'sw tool';
mineBtn.setAttribute('role', 'radio');
mineBtn.setAttribute('aria-checked', 'false');
mineBtn.title = L.tipMine;
mineBtn.innerHTML = `<i>${MINE_ICON}</i><em>X</em>`;
mineBtn.onclick = () => setTool('mine');
paletteEl.appendChild(mineBtn);

const TOOL_BTN = { grab: () => grabBtn, mine: () => mineBtn };
function paintPalette() {
  [...paletteEl.children].forEach((el, i) => el.setAttribute('aria-checked',
    state.tool === 'build' ? i === state.selected : el === TOOL_BTN[state.tool]()));
}
function setTool(t) {
  state.tool = t;
  if (t !== 'build' && held.source !== 'grid') disposeHeld();
  paintPalette();
  refreshTarget();
}
function selectCube(idx) {
  state.selected = idx;
  state.tool = 'build';
  paintPalette();
  if (held.source !== 'grid') armPalette();
}
function rotate(sign) { view.azimT += sign * CAM.azimuthStep * DEG; }
function toggleTilt() {
  const before = CAM.elevations[view.elevIdx] * DEG;
  view.elevIdx = (view.elevIdx + 1) % CAM.elevations.length;
  const after = CAM.elevations[view.elevIdx] * DEG;
  /* 仰角越高，同样的深度在画面上铺得越长——按比例补偿缩放，俯视时底板才不会溢出 */
  zoomBase = clamp(zoomBase * (Math.sin(after) / Math.sin(before)), CAM.zoom.min, CAM.zoom.max);
  applyZoom();
  document.getElementById('tilt').setAttribute('aria-pressed', view.elevIdx > 0);
}
function resetView() {
  view.azimT = nearestAngle(45 * DEG, view.azim.v);
  view.elevIdx = 0;
  zoomBase = fitZoom();
  applyZoom();
  view.target.set(G.x / 2, 1.5, G.z / 2);
  document.getElementById('tilt').setAttribute('aria-pressed', 'false');
}
const alohaBtn = document.getElementById('aloha');
function applyArms(animate) {
  armObjects.forEach(a => {
    a.visible = armsShown;
    if (animate && armsShown) {                     // 出场：掉下来，落地弹一下
      const drop = CFG.robots.dropHeight, rest = a.userData.restY || 0;
      tween(0.2, k => { a.position.y = rest + drop * (1 - k * k); }, () => {
        a.position.y = rest;
        tween(0.22, k => { a.position.y = rest + Math.sin(Math.PI * k) * 0.2 * (1 - k); });
      });
    }
  });
  applyZoom();
  alohaBtn.setAttribute('aria-pressed', armsShown);
  localStorage.setItem('cs-arms', armsShown ? 'on' : 'off');
}
alohaBtn.onclick = () => { armsShown = !armsShown; applyArms(true); };

document.getElementById('reset').onclick = resetView;
document.getElementById('rotL').onclick = () => rotate(+1);
document.getElementById('rotR').onclick = () => rotate(-1);
document.getElementById('tilt').onclick = toggleTilt;
document.getElementById('theme').onclick = () => applyTheme(themeName === 'dark' ? 'light' : 'dark');
document.getElementById('gridlabel').textContent = `${G.x} × ${G.z}`;

const langEl = document.getElementById('lang');
function applyLang(next) {
  lang = next;
  L = STRINGS[lang];
  localStorage.setItem('cs-lang', lang);
  document.documentElement.lang = lang === 'en' ? 'en' : 'zh';
  langEl.dataset.lang = lang;
  langEl.title = L.tipLang;

  const set = (id, text, title) => {
    const el = document.getElementById(id);
    if (text !== null) el.textContent = text;
    if (title) el.title = title;
  };
  set('save', L.save, L.tipSave);
  set('clear', clearArmed ? L.resetConfirm : L.reset, L.tipReset);
  set('theme', null, L.tipTheme);
  set('aloha', null, L.tipArms);
  set('rotL', null, L.tipRotL);
  set('rotR', null, L.tipRotR);
  set('tilt', null, L.tipTilt);
  set('reset', null, L.tipView);
  grabBtn.title = L.tipGrab;
  mineBtn.title = L.tipMine;
  [...paletteEl.children].forEach((el, i) => {
    if (i < CUBES.length) el.title = L.colorTip(L.colors[i] || CUBES[i].label, i + 1);
  });

  const hint = document.getElementById('hint');
  hint.textContent = '';
  for (const line of L.hints) {
    const row = document.createElement('span');
    for (const tok of line) {
      if (typeof tok === 'string') row.append(tok);
      else {
        const kbd = document.createElement('kbd');
        kbd.textContent = tok.k;
        row.append(kbd);
      }
    }
    hint.append(row);
  }
  updateCount();
}
langEl.onclick = () => applyLang(lang === 'zh' ? 'en' : 'zh');

/* 两种语言里挑最宽的那个当下限，切语言（以及「重置 / 确认重置」）就不会把面板撑来撑去 */
function reserveWidths() {
  const measure = (el, samples) => {
    const keep = el.textContent;
    const prev = el.style.minWidth;
    el.style.minWidth = '0px';
    let max = 0;
    for (const text of samples) {
      el.textContent = text;
      max = Math.max(max, el.getBoundingClientRect().width);
    }
    el.textContent = keep;
    el.style.minWidth = Math.max(Math.ceil(max), parseFloat(prev) || 0) + 'px';
  };
  const both = key => ['zh', 'en'].map(lg => STRINGS[lg][key]);
  measure(document.getElementById('save'), both('save'));
  measure(document.getElementById('clear'), both('reset'));
  measure(document.getElementById('gridlabel'),
    ['zh', 'en'].map(lg => `${G.x} × ${G.z} · ${STRINGS[lg].blocks(88)}`));
}

const toastEl = document.getElementById('toast');
let toastTimer;
function toast(msg) {
  toastEl.textContent = msg;
  toastEl.classList.add('on');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toastEl.classList.remove('on'), 2000);
}
function updateCount() {
  document.getElementById('gridlabel').textContent = `${G.x} × ${G.z} · ${L.blocks(blocks.size)}`;
}
const clearBtn = document.getElementById('clear');
let clearArmed = false, clearTimer;
function disarmClear() {
  clearArmed = false;
  clearBtn.classList.remove('warn');
  clearBtn.textContent = L.reset;
}
clearBtn.onclick = () => {
  if (!clearArmed) {                                   // 第一次点只是确认
    clearArmed = true;
    clearBtn.classList.add('warn');
    clearBtn.textContent = L.resetConfirm;
    clearTimeout(clearTimer);
    clearTimer = setTimeout(disarmClear, 3000);
    return;
  }
  clearTimeout(clearTimer);
  disarmClear();
  const all = [...blocks.values()];
  blocks.clear();
  all.forEach((m, n) => {
    const y0 = m.position.y;
    tween(0.3 + (n % 8) * 0.025, k => {
      m.scale.setScalar(Math.max(0.001, 1 - k * k));
      m.position.y = y0 + 0.5 * k;
    }, () => blocksGroup.remove(m));
  });
  resetView();
  updateCount();
  toast(L.cleared(all.length));
};

document.getElementById('save').onclick = () => {
  const data = {
    version: 1,
    grid: G,
    exclude: EXCLUDE,
    palette: CUBES.map(c => ({ id: c.id, color: c.color })),
    blocks: [...blocks.values()].map(m => ({
      x: m.userData.cell.i, y: m.userData.cell.k, z: m.userData.cell.j,
      type: CUBES[m.userData.type].id
    }))
  };
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
  const a = document.createElement('a');
  a.href = url;
  a.download = `layout-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '')}.json`;
  a.click();
  URL.revokeObjectURL(url);
  toast(L.saved(data.blocks.length, a.download));
};

/* ============================ 主循环 ============================ */
function resize() {
  renderer.setSize(innerWidth, innerHeight, false);
}
addEventListener('resize', resize);
resize();
applyTheme(themeName);
applyLang(lang);
zoomBase = fitZoom();
applyArms();
view.zoom.v = view.zoomT;                            // 开局直接就位，不做一次缩放动画
reserveWidths();
armPalette();
paintPalette();
updateCount();

const clock = new THREE.Clock();
let time = 0;
renderer.setAnimationLoop(() => {
  const dt = Math.min(clock.getDelta(), 1 / 30);
  time += dt;

  stepTweens(dt);
  updateCamera(dt);

  const t = state.target;
  if (held.mesh && t) {
    const stiff = state.magnetic ? IX.springMagnetic : IX.springFree;
    springStep(hs.x, t.i + 0.5, stiff, dt);
    springStep(hs.z, t.j + 0.5, stiff, dt);
    springStep(hs.y, t.k + 0.5 + IX.hoverHeight, stiff * 0.8, dt);
    const m = held.mesh;
    m.position.set(hs.x.v, hs.y.v + Math.sin(time * 3.2) * IX.bobAmount, hs.z.v);
    m.rotation.z = clamp(-hs.x.vel * IX.tiltAmount, -0.26, 0.26);   // 往运动方向倾一点
    m.rotation.x = clamp(hs.z.vel * IX.tiltAmount, -0.26, 0.26);
    m.material.opacity += ((t.valid ? 1 : 0.45) - m.material.opacity) * Math.min(1, dt * 12);
    m.visible = true;
  } else if (held.mesh) {
    held.mesh.visible = false;
  }

  ghost.visible = plate.visible && !!t && t.valid;
  if (plate.visible && t) {
    const stiff = state.magnetic ? IX.springMagnetic : IX.springFree;
    springStep(plateS.x, t.i + 0.5, stiff, dt);
    springStep(plateS.z, t.j + 0.5, stiff, dt);
    springStep(plateS.y, t.k + 0.53, stiff, dt);   // 抬一点，底边不被底板吃掉
    plate.position.set(plateS.x.v, plateS.y.v, plateS.z.v);
    plate.material.color.set(t.valid ? CUBES[held.mesh ? held.type : state.selected].color : '#b23b2e');
    plate.material.color.offsetHSL(0, 0.06, THEME_LIFT);  // 只动明度/饱和，保住方块本来的颜色
    ghost.position.set(plateS.x.v, plateS.y.v, plateS.z.v);
    ghost.material.color.copy(plate.material.color);
    plate.material.opacity = (t.valid ? (state.magnetic ? 1 : 0.85) : 0.9);
  }

  renderer.render(scene, cam);
});

canvas.focus();
