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
/* 方块的竖直运动都走这一个槽，互相取消。两条补间同时写 position.y 会把方块留在半空 */
function yTween(mesh, dur, fn) {
  cancelTween(mesh.userData.yTween);                 // 新的一条接管，旧的立刻作废
  const w = tween(dur, fn, () => { mesh.position.y = mesh.userData.baseY; });
  mesh.userData.yTween = w;
  return w;                                          // 调用方可以改写 onDone
}
/* 方块只在位置上弹一下，不做拉伸变形 */
const hop = (mesh, amp = 0.09, dur = 0.28) =>
  yTween(mesh, dur, k => {
    mesh.position.y = mesh.userData.baseY + Math.sin(Math.PI * k) * amp * (1 - k);
  });

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
/* 可放置区只有两条高台「之间」：高台本身和它外侧一律挖空 */
const BELT_W = R ? (R.riserMeters / G.cellMeters) : 0;      // 高台宽度（格）
const PLAY = { x0: 0, x1: G.x - 1, z0: 0, z1: G.z - 1 };
if (R) {
  const inner = ARM_PLACES.map(a => a.x).sort((p, q) => p - q);
  PLAY.x0 = Math.ceil(inner[0] + BELT_W / 2);               // 左侧高台的内边
  PLAY.x1 = Math.floor(inner[1] - BELT_W / 2) - 1;          // 右侧高台的内边
}
/* 再往里收一圈：最外一圈格子不画网格也不允许放置 */
PLAY.x0 += 1; PLAY.x1 -= 1; PLAY.z0 += 1; PLAY.z1 -= 1;

/* ---- 底板 + 网格线 ---- */
const B = CFG.board;
const WM = CFG.watermark;
const slab = new THREE.Mesh(                        // 台面比网格大一圈，两侧留出放机械臂的地方
  new RoundedBoxGeometry(G.x + 2 * B.marginX, B.thickness, G.z + 2 * B.marginZ, 4, 0.24),
  new THREE.MeshPhysicalMaterial({ roughness: 0.98, metalness: 0, sheen: 0.4, sheenRoughness: 0.95 })
);
slab.position.set(G.x / 2, -B.thickness / 2, G.z / 2);
slab.receiveShadow = true;

/* 网格铺满台面，但抠掉两个臂座占的方块；按格子收集边，避免重复画 */
const blocked = (i, j) => i < PLAY.x0 || i > PLAY.x1 || j < PLAY.z0 || j > PLAY.z1;
const edges = new Set();
for (let i = 0; i < G.x; i++) {
  for (let j = 0; j < G.z; j++) {
    if (blocked(i, j)) continue;
    edges.add(`h:${i}:${j}`); edges.add(`h:${i}:${j + 1}`);
    edges.add(`v:${i}:${j}`); edges.add(`v:${i + 1}:${j}`);
  }
}
/* 两座高台：俯视是胶囊形的小台，坐在台面上。不碰台沿，也就没有剖面可言 */
const LIFT = R ? R.lift * ARM_SCALE : 0;
if (R && LIFT > 0) {
  const rw = (R.riserMeters * ARM_SCALE) / 2;              // 胶囊半宽
  const straight = R.riserLengthMeters * ARM_SCALE - 2 * rw;  // 中间直段
  const fil = Math.min(R.beltFillet ?? 0.25, LIFT / 4);   // 圆角不能吃掉整个高度，否则看着像个枕头
  const half = Math.max(straight, 0) / 2;

  /* 胶囊轮廓按点列出来，首尾不重合。路径若回到起点，那个重复顶点会让倒角算出错的
     偏移方向，那一侧就凹进去 */
  const outline = [];
  const N = 32;
  for (let i = 0; i <= N; i++) {                   // 上端半圆：(rw, half) → (-rw, half)
    const a = (i / N) * Math.PI;
    outline.push(new THREE.Vector2(rw * Math.cos(a), half + rw * Math.sin(a)));
  }
  for (let i = 0; i <= N; i++) {                   // 下端半圆：(-rw, -half) → (rw, -half)
    const a = Math.PI + (i / N) * Math.PI;
    outline.push(new THREE.Vector2(rw * Math.cos(a), -half + rw * Math.sin(a)));
  }
  const cap = new THREE.Shape(outline);            // 两条直边由相邻点自然连出

  /* 倒角同时做出顶面的圆边和落到台面的那圈圆滑过渡 */
  const riserGeo = new THREE.ExtrudeGeometry(cap, {
    depth: LIFT - 2 * fil, bevelEnabled: true,
    bevelThickness: fil, bevelSize: fil, bevelSegments: 4, curveSegments: 24
  });
  riserGeo.rotateX(-Math.PI / 2);                          // 挤出方向转成朝上
  riserGeo.computeBoundingBox();
  riserGeo.translate(0, -riserGeo.boundingBox.min.y, 0);   // 底面贴台面

  for (const a of ARM_PLACES) {
    const riser = new THREE.Mesh(riserGeo, slab.material);
    riser.position.set(a.x, 0, a.z);
    riser.castShadow = riser.receiveShadow = true;
    scene.add(riser);
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
  loadArms({ ...R, scale: ARM_SCALE, arms: ARM_PLACES }, armMat)
    .then(arms => {
      arms.forEach(a => { a.visible = false; armObjects.push(a); scene.add(a); });
      /* 首次进来让机械臂「掉」进场，但要等网格真的就绪：先编译着色器，
         再等一帧，否则掉落的头几帧会被首次上传 GPU 卡掉。 */
      renderer.compile(scene, cam);
      requestAnimationFrame(() => requestAnimationFrame(() => applyArms(true)));
    })
    .catch(e => console.warn('WidowX 加载失败：', e));
}

/* 桌下的地面，以及印在上面的水印。取 Abaka logo 的品牌橙与几何无衬线，压到刚好看得见 */
function watermarkTexture(color) {
  const lines = WM.lines;
  const c = document.createElement('canvas');
  c.width = 2048;
  c.height = 16;                                     // 先量字，量完才知道该多高
  const font = px => `800 ${px}px "Futura", "Avenir Next", "Century Gothic", ` +
                     `"Poppins", "Helvetica Neue", system-ui, sans-serif`;
  const setup = g => {
    g.textAlign = 'left';
    g.textBaseline = 'middle';
    g.fillStyle = color;
    try { g.letterSpacing = WM.tracking + 'px'; } catch {}
  };
  const pad = 20;                                    // 留边，否则末字母会被画布切掉
  const avail = c.width - pad * 2;

  let g = c.getContext('2d');
  setup(g);
  /* 每行各自缩放到同一宽度：短的字就大，整块像一个标志 */
  const sizes = lines.map(t => {
    g.font = font(400);
    const m = g.measureText(t);
    return Math.floor(400 * avail / Math.max(m.width, m.actualBoundingBoxRight || 0));
  });
  const rows = sizes.map(px => Math.round(px * WM.lineHeight));
  c.height = rows.reduce((a, b) => a + b, 0);        // 改尺寸会重置 context，要重新设一遍
  g = c.getContext('2d');
  setup(g);
  let y = 0;
  lines.forEach((t, i) => {
    g.font = font(sizes[i]);
    g.fillText(t, pad, y + rows[i] / 2);
    y += rows[i];
  });
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}
const floorMat = new THREE.MeshBasicMaterial();   // 不透明：否则会排在水印之后把它盖掉
const floor = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), floorMat);
floor.rotation.x = -Math.PI / 2;
scene.add(floor);

const markMat = new THREE.MeshBasicMaterial({
  map: watermarkTexture(WM.color), transparent: true,
  opacity: WM.opacity, depthWrite: false
});
const markW = (G.x + 2 * B.marginX) * WM.scale;
const markImg = markMat.map.image;
const markH = markW * markImg.height / markImg.width;
const mark = new THREE.Mesh(new THREE.PlaneGeometry(markW, markH), markMat);
mark.rotation.x = -Math.PI / 2;
mark.renderOrder = 1;
scene.add(mark);

const floorY = () => -B.thickness - WM.drop;
floor.scale.set((G.x + 2 * B.marginX) * 40, (G.z + 2 * B.marginZ) * 40, 1);  // 够大，正视图里看不到地面尽头
floor.position.set(G.x / 2, floorY(), G.z / 2);

/* 水印始终贴在背对镜头的那条长边外侧；镜头转到另一条长边时先淡出再淡入 */
let wmSide = 0, wmAlpha = 1, wmFade = null;
function placeMark(side) {
  const off = (G.z + 2 * B.marginZ) / 2 + markH / 2 + WM.offset;
  mark.position.set(G.x / 2, floorY() + 0.01, G.z / 2 - side * off);   // 相对台面居中
  mark.rotation.z = side > 0 ? 0 : Math.PI;          // 翻面，保证顺着屏幕从左读到右
}
function updateMarkSide() {
  const side = Math.cos(view.azimT) >= 0 ? 1 : -1;   // 镜头在哪一侧，水印就去对面那条长边
  if (side === wmSide) return;
  if (wmSide === 0) { wmSide = side; placeMark(side); return; }
  wmSide = side;
  cancelTween(wmFade);
  wmFade = tween(0.22, k => { wmAlpha = 1 - k; }, () => {
    placeMark(wmSide);
    wmFade = tween(0.45, k => { wmAlpha = k; });
  });
}
const pickRoot = new THREE.Group();
const blocksGroup = new THREE.Group();
pickRoot.add(slab, blocksGroup);
scene.add(pickRoot);

/* ---- 方块 ---- */
const CUBE = G.cubeMeters / G.cellMeters;            // 方块边长（格）：2.5cm 本体放进 3cm 的格
const cellY = k => k * CUBE + CUBE / 2;              // 层间距＝方块本身，堆叠不留缝；缝只在水平方向
const cubeGeo = new RoundedBoxGeometry(CUBE, CUBE, CUBE, 4, CFG.style.bevel * CUBE);
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
/* 落点指示：落点格里的半透明方块 */
const ghost = new THREE.Mesh(cubeGeo, new THREE.MeshBasicMaterial({
  transparent: true, opacity: CFG.style.ghostOpacity, depthWrite: false
}));
ghost.visible = false;
scene.add(ghost);
const ghostS = { x: S(), y: S(), z: S() };
let markerOn = false;
let ghostAlpha = 1, ghostFade = null;
function ghostDropFade() {                            // 放下时先让落点方块消失，别和真方块叠在一起
  cancelTween(ghostFade);
  ghostFade = tween(0.09, k => { ghostAlpha = 1 - k; }, () => {
    ghostFade = tween(0.3, k => { ghostAlpha = k; });
  });
}
function ghostFromHand() {                            // 影子从「手上这一块」起跳，而不是从上一块滑过来
  for (const [g, h] of [[ghostS.x, hs.x], [ghostS.y, hs.y], [ghostS.z, hs.z]]) {
    g.v = h.v; g.vel = 0;
  }
}

/* ============================ 相机 ============================ */
const DEG = Math.PI / 180;
const view = {
  azim: S(45 * DEG), azimT: 45 * DEG,
  elev: S(CAM.elevations[0] * DEG), elevT: CAM.elevations[0] * DEG,
  zoom: S(CAM.zoom.default), zoomT: CAM.zoom.default,
  target: new THREE.Vector3(G.x / 2, 1.5, G.z / 2)
};
const cam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 500);

/* 默认缩放按台面大小算出来，换台面尺寸不用再手动调 */
function fitZoom(e = view.elevT) {
  const aspect = (canvas.clientWidth || innerWidth) / (canvas.clientHeight || innerHeight);
  const span = (G.x + G.z) * Math.SQRT1_2;          // 等距投影下台面的屏幕跨度
  const tall = R ? CAM.sceneHeight * ARM_SCALE : 8; // 场景的竖直高度（米 -> 格）
  const h = Math.max(span / (2 * aspect),            // 取最大而不是相加：相加会把常规视角也撑飞
                     span * Math.sin(e) / 2,
                     tall * Math.cos(e) / 2);
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
  springStep(view.elev, view.elevT, CAM.springStiffness, dt);
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
  floorMat.color.set(t.floor);
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
  ghostFromHand();
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
  ghostFromHand();
  rippleNeighbors(c);
}
function placeHeld() {
  const t = state.target;
  if (!t || !t.valid || !held.mesh) return;
  if (blockAt(t.i, t.k, t.j)) return;                // 该格已被占：目标是旧的，覆盖会留下孤儿
  const mesh = held.mesh, from = mesh.position.clone();
  const rot = { x: mesh.rotation.x, z: mesh.rotation.z };

  mesh.material.dispose();
  mesh.material = cubeMats[held.type];
  mesh.renderOrder = 0;
  mesh.castShadow = mesh.receiveShadow = true;
  mesh.position.set(t.i + 0.5, cellY(t.k), t.j + 0.5);
  mesh.userData = { cell: { i: t.i, k: t.k, j: t.j }, type: held.type, baseY: cellY(t.k) };
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
  ghostDropFade();
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
    m.userData.baseY = cellY(m.userData.cell.k);
    const from = m.position.y;
    yTween(m, 0.3 + n * 0.02, k => {
      m.position.y = from + (m.userData.baseY - from) * (k * k);   // 加速下落
    }).onDone = () => { m.position.y = m.userData.baseY; hop(m, 0.07, 0.26); };
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
    markerOn = !noPlate;
    if (state.magnetic) pulse(ghost, 0.12, 0.26);       // 吸住的一下「咔」
  }
  if (!t || noPlate) markerOn = false;
  updateCursor(isBlock, grabbing);
}
const HOVER_LIFT = 0.14;
function liftTo(mesh, to, dur, ease) {
  const from = mesh.position.y - mesh.userData.baseY;
  yTween(mesh, dur, k => {
    mesh.position.y = mesh.userData.baseY + from + (to - from) * ease(k);
  }).onDone = () => { mesh.position.y = mesh.userData.baseY + to; };
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
  else if (grabbing) c = 'cur-grab';                 // 抓取模式下始终显示手
  else if (state.tool === 'mine') c = 'cur-mine';   // 整个删除模式都用垃圾桶光标
  else if (state.target) c = state.target.valid ? 'cur-grabbing' : 'cur-deny';
  canvas.className = c;
}

/* ============================ 输入 ============================ */
let panning = false, panPrev = null, spaceDown = false;

canvas.addEventListener('pointerenter', () => { pointerOver = true; });
canvas.addEventListener('pointerleave', () => {
  pointerOver = false; state.target = null; markerOn = false; setHover(null);
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
  refreshTarget();                                   // 上一次点击可能已经改变了这一列

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
  if (e.key === 'r' || e.key === 'R') topView();
  if (e.key === 'f' || e.key === 'F') faceView(true);
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
function rotate(sign) { view.azimT += sign * CAM.azimuthStep * DEG; updateViewButtons(); }
function setElevation(deg) {                       // 换仰角后重新取景，比例补偿在接近平视时会失效
  view.elevT = deg * DEG;
  zoomBase = fitZoom();
  applyZoom();
  updateViewButtons();
}
/* 当前是哪种视角，直接从相机姿态看出来，所以转完 90° 正视/侧视会自己对调 */
function viewMode() {
  const near = (a, b) => Math.abs(a - b) < 0.01;
  if (near(view.elevT, CAM.elevations[1] * DEG)) return 'top';
  if (near(view.elevT, CAM.frontElevation * DEG)) return 'front';
  return 'iso';
}
function updateViewButtons() {
  const m = viewMode();
  for (const [id, mode] of [['reset', 'iso'], ['tilt', 'top'], ['front', 'front']]) {
    document.getElementById(id).setAttribute('aria-pressed', m === mode);
  }
}
function topView() { setElevation(CAM.elevations[1]); }
function faceView(longEdge) {                        // 正视＝看长边（机械臂的侧面）；侧视＝看短边
  const q = Math.PI / 2;
  const k = Math.round((view.azim.v - (longEdge ? 0 : q)) / Math.PI);
  view.azimT = nearestAngle(k * Math.PI + (longEdge ? 0 : q), view.azim.v);
  setElevation(CAM.frontElevation);
}
function resetView() {
  view.azimT = nearestAngle(45 * DEG, view.azim.v);
  view.elevT = CAM.elevations[0] * DEG;
  zoomBase = fitZoom();
  applyZoom();
  view.target.set(G.x / 2, 1.5, G.z / 2);
  updateViewButtons();
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

document.getElementById('front').onclick = () => faceView(true);
document.getElementById('reset').onclick = resetView;
document.getElementById('rotL').onclick = () => rotate(+1);
document.getElementById('rotR').onclick = () => rotate(-1);
document.getElementById('tilt').onclick = topView;
document.getElementById('theme').onclick = () => applyTheme(themeName === 'dark' ? 'light' : 'dark');

/* 全屏：按钮只负责发起，图标跟着浏览器真实的全屏状态走（Esc 退出也能同步） */
const fsBtn = document.getElementById('fullscreen');
fsBtn.onclick = () => {
  if (document.fullscreenElement) document.exitFullscreen?.();
  else document.documentElement.requestFullscreen?.().catch(() => {});
};
document.addEventListener('fullscreenchange', () => {
  const on = !!document.fullscreenElement;
  fsBtn.setAttribute('aria-pressed', on);
  fsBtn.title = on ? L.tipFullExit : L.tipFull;
});
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
  set('fullscreen', null, document.fullscreenElement ? L.tipFullExit : L.tipFull);
  set('aloha', null, L.tipArms);
  set('rotL', null, L.tipRotL);
  set('rotR', null, L.tipRotR);
  set('tilt', null, L.tipTilt);
  set('front', null, L.tipFront);
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
    play: PLAY,
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
updateViewButtons();
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
  updateMarkSide();
  markMat.opacity = WM.opacity * wmAlpha;

  const t = state.target;
  if (held.mesh && t) {
    const stiff = state.magnetic ? IX.springMagnetic : IX.springFree;
    springStep(hs.x, t.i + 0.5, stiff, dt);
    springStep(hs.z, t.j + 0.5, stiff, dt);
    springStep(hs.y, cellY(t.k) + IX.hoverHeight, stiff * 0.8, dt);
    const m = held.mesh;
    m.position.set(hs.x.v, hs.y.v + Math.sin(time * 3.2) * IX.bobAmount, hs.z.v);
    m.rotation.z = clamp(-hs.x.vel * IX.tiltAmount, -0.26, 0.26);   // 往运动方向倾一点
    m.rotation.x = clamp(hs.z.vel * IX.tiltAmount, -0.26, 0.26);
    m.material.opacity += ((t.valid ? 1 : 0.45) - m.material.opacity) * Math.min(1, dt * 12);
    m.visible = true;
  } else if (held.mesh) {
    held.mesh.visible = false;
  }

  const ghostWasVisible = ghost.visible;
  ghost.visible = markerOn && !!t && t.valid;
  if (ghost.visible && !ghostWasVisible) ghostFromHand();
  if (ghost.visible) {
    const gs = state.magnetic ? IX.springMagnetic : IX.springFree;
    springStep(ghostS.x, t.i + 0.5, gs, dt);
    springStep(ghostS.y, cellY(t.k), gs, dt);
    springStep(ghostS.z, t.j + 0.5, gs, dt);
    ghost.position.set(ghostS.x.v, ghostS.y.v, ghostS.z.v);
    ghost.material.color.set(CUBES[held.mesh ? held.type : state.selected].color);
    ghost.material.color.offsetHSL(0, 0.06, THEME_LIFT);  // 只动明度/饱和，保住方块本来的颜色
    ghost.material.opacity = CFG.style.ghostOpacity * ghostAlpha;
  }

  renderer.render(scene, cam);
});

canvas.focus();
