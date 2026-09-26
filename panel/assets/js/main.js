/* The operator panel: the rig full-screen with frosted panels over it.
   Status comes from mock.js until the backend exists; the shapes already match it. */
import { STATUS } from './mock.js';
import * as W from './widgets.js';
import { loadArmAssets } from './arm.js';
import { makeScene } from './scene.js';

const $ = id => document.getElementById(id);

let CFG;
try {
  CFG = await (await fetch('./config.json', { cache: 'no-store' })).json();
} catch (err) {
  document.body.innerHTML = '<p class="needserver">config.json could not be loaded. Serve the repo over http: run <code>python3 -m http.server 8000</code> from the repo root, then open <code>http://localhost:8000/panel/</code>.</p>';
  throw err;
}

/* ---- UI state (per viewer) ---- */
const S = STATUS;
const ui = { selected: W.active(S)?.name ?? 'reset', query: '', cat: 'all', cams: {}, log: 'all' };
let theme = CFG.defaultTheme;
try { theme = localStorage.getItem('myaloha-theme') || theme; } catch {}
document.documentElement.dataset.theme = theme;

/* ---- static chrome ---- */
$('theme').innerHTML = W.icon('theme');
$('estop').innerHTML = `${W.icon('stop')}Stop`;
$('estop').disabled = !W.active(S);
$('search-icon').outerHTML = W.icon('search');
$('mfoot').textContent = W.modCount();

/* ---- rendering ---- */
function renderBrowser() {
  $('cats').innerHTML = W.modCats(ui);
  $('mlist').innerHTML = W.modList(S, ui);
}
function renderDock() {
  $('dock-body').innerHTML = W.dock(S, ui.selected);
  $('dock-foot').innerHTML = W.now(S) + W.actions(S, ui.selected);
}
function renderCams() {
  $('cams').innerHTML = W.CAM_ORDER.map(k => W.camTile(S, k, CFG.snapshots, ui.cams[k])).join('');
}
function renderLog() {
  $('log').innerHTML = W.logPanel(S, ui.log);
  const lines = $('log').querySelector('.loglines');
  lines.scrollTop = lines.scrollHeight;                 // follow the newest line
}
function renderAll() {
  $('brand').innerHTML = `<b class="brand-name">myaloha</b>${W.majorStrip(S)}`;
  renderBrowser(); renderDock(); renderCams(); renderLog();
}
renderAll();
/* tick the running state's clock; leave the body alone while another mod's params are edited */
setInterval(() => {
  if (W.active(S)?.name === ui.selected) $('dock-body').innerHTML = W.dock(S, ui.selected);
  $('dock-foot').innerHTML = W.now(S) + W.actions(S, ui.selected);
}, 1000);

/* ---- the rig: arm status lives on the arms themselves ---- */
const assets = await loadArmAssets(CFG.robots.dir, CFG.robots.spec);
const host = $('scene');
const scene = makeScene(host, CFG, assets, S, theme);
host.addEventListener('rendered', () => {
  $('tags').innerHTML = W.ARM_ORDER.map(n => {
    const a = S.arms[n], p = scene.project(n);
    return `<div class="tag ${a.connected ? '' : 'off'}" style="left:${p.x}px;top:${p.y}px">${W.dot(W.armKind(a))}${W.armLabel(n)}<span class="sub">${W.modeText(a)}</span></div>`;
  }).join('');
});
/* keep the rig in the gap between the panels */
scene.setInsets({ l: 316, r: 380, t: 210, b: 222 });

/* ---- interaction ---- */
$('q').addEventListener('input', e => { ui.query = e.target.value.trim(); renderBrowser(); });
document.addEventListener('keydown', e => {
  if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'k') { e.preventDefault(); $('q').focus(); $('q').select(); }
  if (e.key === 'Escape' && document.activeElement === $('q')) { $('q').value = ''; ui.query = ''; renderBrowser(); $('q').blur(); }
});
document.addEventListener('click', e => {
  const t = e.target;
  const cat = t.closest('[data-cat]');
  if (cat) { ui.cat = cat.dataset.cat; renderBrowser(); return; }
  const mod = t.closest('[data-mod]');
  if (mod) { ui.selected = mod.dataset.mod; renderBrowser(); renderDock(); return; }
  const cam = t.closest('[data-cam]');
  if (cam) { const k = cam.dataset.cam; ui.cams[k] = ui.cams[k] === 'depth' ? 'color' : 'depth'; renderCams(); return; }
  const tog = t.closest('button[data-param]');
  if (tog) {
    tog.classList.toggle('on');
    if (!tog.dataset.val) tog.textContent = tog.classList.contains('on') ? 'on' : 'off';
    return;
  }
  const lf = t.closest('[data-logf]');
  if (lf) { ui.log = lf.dataset.logf; renderLog(); return; }
  if (t.closest('#theme')) {
    theme = theme === 'light' ? 'dark' : 'light';
    document.documentElement.dataset.theme = theme;
    try { localStorage.setItem('myaloha-theme', theme); } catch {}
    scene.setTheme(theme);
  }
});
window.__ready = true;
