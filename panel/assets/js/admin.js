/* The operator panel: a visualization of the robot API (myaloha/server.py). Every number on
   screen comes from the /api/stream WebSocket; every button is one REST call. */
import * as api from './api.js';
import * as W from './widgets.js';
import { loadArmAssets } from './arm.js';
import { makeScene } from './scene.js';

const $ = id => document.getElementById(id);
const { ctx } = W;

/* ---- emergency stop first: armed before anything else loads, fires on press ---- */
let send = () => false;                        // the stream socket, once open
function emergencyStop() {
  api.emergencyStop(send).catch(e => { ui.error = `E-stop REST failed: ${e.message}`; });
}
$('estop').innerHTML = `${W.icon('octagon')}E-STOP<kbd>Space</kbd>`;
$('estop').addEventListener('pointerdown', e => { e.preventDefault(); emergencyStop(); });
document.addEventListener('keydown', e => {
  if ((e.code === 'Space' || e.key === ' ') && !e.repeat && !e.target.matches('input, textarea')) { e.preventDefault(); emergencyStop(); }
}, true);

const CFG = await (await fetch('./config.json', { cache: 'no-store' })).json();

/* ---- per-viewer UI state ---- */
const ui = { selected: null, query: '', cat: 'all', cams: {}, log: 'all', error: null, topics: {}, topicOpen: null };
let S = null, link = false, serverNow = Date.now() / 1000;
let theme = CFG.defaultTheme;
try { theme = localStorage.getItem('myaloha-theme') || theme; } catch {}
document.documentElement.dataset.theme = theme;

$('theme').innerHTML = W.icon('theme');
$('reconnect').innerHTML = `${W.icon('plug')}<span>Reconnect</span>`;
$('search-icon').outerHTML = W.icon('search');
$('fold').innerHTML = W.icon('fold');

/* mods panel: folded below 1280 px unless the viewer chose otherwise (remembered) */
let foldChoice = null;
try { foldChoice = localStorage.getItem('myaloha-fold'); } catch {}
const applyFold = () => {
  const folded = foldChoice ? foldChoice === 'folded' : innerWidth < 1280;
  document.body.classList.toggle('folded', folded);
  $('fold').title = folded ? 'Expand the mods panel' : 'Collapse the mods panel';
  $('q').placeholder = folded ? 'Search' : 'Search mods';
  scene?.setInsets({ l: folded ? 222 : 316 });
};
let scene = null;
applyFold();
addEventListener('resize', applyFold);

/* ---- rendering: each region redraws only when what it shows changed ---- */
const last = {};
const changed = (region, key) => (last[region] !== key ? ((last[region] = key), true) : false);

function renderBrand() {
  const m = W.active(S);
  if (changed('brand', JSON.stringify([link, S?.state, S?.mods?.map(x => [x.name, x.state])])))
    $('brand').innerHTML = `<b class="brand-name">myaloha</b>${W.majorStrip(S, link)}`;
  const absent = S ? [...Object.values(S.arms).filter(a => !a.connected || a.lost).map(a => a.name),
    ...Object.entries(S.cameras ?? {}).filter(([, c]) => !c.ok).map(([k]) => `${k} camera`)] : [];
  $('reconnect').hidden = !absent.length;
  $('reconnect').title = absent.length ? `Try to connect ${absent.join(', ')} (no motion; cameras also retry on their own)` : '';
  const estopped = S?.state === 'estop';
  $('estop').classList.toggle('on', estopped);
  $('estop-banner').hidden = !estopped;
  renderDrapeButton();
}

/* the table projection needs a finished localize run that registered the top camera: grey until
   there is one, and while localize is running (it replaces the calibration) */
function drapeBlocked() {
  if (S?.mods?.some(m => m.name === 'localize')) return 'Calibration running: wait for localize to finish';
  if (!ctx.coordinates) return 'No coordinate calibration yet: run localize first';
  if (!ctx.coordinates.cameras?.high) return 'The top camera was not registered in the last localize run; run it again';
  return '';
}
function renderDrapeButton() {
  const btn = document.querySelector('[data-view="drape"]');
  const why = drapeBlocked();
  btn.disabled = !!why;
  btn.title = why || "Project the top camera's pixels onto the table plane (T)";
  if (why && drapeOn) { drapeOn = false; scene.setDrape(null); btn.setAttribute('aria-pressed', 'false'); }
}

function renderBrowser(force) {
  const key = JSON.stringify([ui, S?.requirements, S?.mods?.map(x => [x.name, x.state])]);
  if (!changed('browser', key) && !force) return;
  $('cats').innerHTML = W.modCats(ui);
  $('mlist').innerHTML = W.modList(S, ui);
  $('mfoot').textContent = `${Object.keys(ctx.mods).length} mods`;
}

function renderDock(force) {
  const m = W.active(S);
  const running = m?.name === ui.selected;
  /* the body holds editable params: redraw it only when the selection or the running FSM moves */
  const key = JSON.stringify([ui.selected, running && m.history, running && m.state, S?.requirements?.[ui.selected], !!m]);
  if (changed('dock', key) || force) $('dock-body').innerHTML = W.dock(S, ui.selected, serverNow);
  else if (running) $('dock-body').querySelectorAll('.vstep.cur .vt').forEach(el => {
    el.textContent = `${(serverNow - m.since).toFixed(serverNow - m.since < 10 ? 1 : 0)} s ●`;
  });
  $('nowline').innerHTML = W.now(S, serverNow);
  if (changed('actions', JSON.stringify([link, !!S, !!m, S?.state, ui.selected, ui.error, S?.requirements?.[ui.selected]])) || force)
    $('acts').innerHTML = W.actions(link ? S : null, ui.selected, ui.error);
}

function renderCams(force) {
  const cams = S?.cameras ?? {};
  const key = JSON.stringify([Object.entries(cams).map(([k, c]) => [k, c.ok, c.error]), ui.cams]);
  if (changed('cams', key) || force) {
    $('cams').innerHTML = Object.entries(cams).map(([k, c]) => {
      return W.camTile(k, c, ui.cams[k] ?? 'color');
    }).join('');
  }
  for (const [k, c] of Object.entries(cams)) {  // fps only: the images keep updating in place
    const el = $('cams').querySelector(`[data-cam="${k}"] [data-fps]`);
    if (el && c.ok) el.textContent = `${c.fps.toFixed(0)} fps`;
  }
}

function renderLog(force) {
  if (!changed('loghead', JSON.stringify([ctx.log.length, ui.log, W.active(S)?.name])) && !force) return;
  $('loghead').innerHTML = W.logHead(S, ui.log);
}
function appendLog(records) {
  const lines = $('loglines');
  const follow = lines.scrollHeight - lines.scrollTop - lines.clientHeight < 24;
  lines.insertAdjacentHTML('beforeend', records.filter(W.logKeep(ui.log)).map(W.logLine).join(''));
  while (lines.children.length > 1000) lines.firstChild.remove();
  if (follow) lines.scrollTop = lines.scrollHeight;
}
function refilterLog() {
  $('loglines').innerHTML = ctx.log.filter(W.logKeep(ui.log)).map(W.logLine).join('');
  $('loglines').scrollTop = $('loglines').scrollHeight;
  renderLog(true);
}

/* ---- topics: controlled over REST; their results come straight from each topic's port ---- */
function renderTopics(force) {
  /* the rate changes every update: redraw on anything else, update the rate in place (a redraw
     under the pointer would swallow clicks) */
  const st = Object.values(S?.topics ?? {}).map(t => [t.name, t.state, t.url, t.waiting, t.code, t.requirements?.met]);
  const key = JSON.stringify([st, ui.topics, ui.topicOpen]);
  for (const [name, t] of Object.entries(S?.topics ?? {})) {
    const el = $('topics').querySelector(`[data-topic="${name}"] [data-thz]`);
    if (el && t.state === 'running') el.textContent = W.hzText(t.hz);
  }
  if (!changed('topics', key) && !force) return;
  $('topics').innerHTML = W.topicList(S, ui);
  const n = Object.values(S?.topics ?? {}).filter(W.topicLive).length;
  $('tcount').textContent = `${n} of ${Object.keys(ctx.topics).length} running`;
}
/* a subscription setting changed: remember it, redraw, resubscribe */
function topicUIChanged() {
  try { localStorage.setItem('myaloha-topics', JSON.stringify(ui.topics)); } catch {}
  renderTopics(true);
  syncTopics();
}

/* what each topic draws in the 3-D view, and how to clear it */
const OVERLAY = {
  locate_cubes: d => scene.setCubes(d?.cubes ?? null),
  cam_params: d => scene.setFrusta(d?.cameras ?? null),
};
const subs = {};
function syncTopics() {
  for (const name of Object.keys(ctx.topics)) {
    const t = S?.topics?.[name], u = ui.topics[name];
    const want = u.eye && W.topicLive(t) && t.url && OVERLAY[name] ? `${t.url} ${u.mode} ${u.period}` : null;
    if (subs[name]?.key === want) continue;
    if (subs[name]) { subs[name].close(); delete subs[name]; OVERLAY[name]?.(null); }
    if (want) subs[name] = { key: want, close: api.topic(t.url, u.mode, u.period, m => OVERLAY[name](m.data)) };
  }
}

/* yes / no in a small dialog; resolves true on yes */
function confirmBox(text, yes) {
  $('confirm-text').textContent = text;
  $('confirm-yes').textContent = yes;
  $('confirm').hidden = false;
  return new Promise(res => {
    const done = v => { $('confirm').hidden = true; $('confirm-yes').onclick = $('confirm-no').onclick = null; res(v); };
    $('confirm-yes').onclick = () => done(true);
    $('confirm-no').onclick = () => done(false);
  });
}

/* start / stop / restart; a 409 names the topics that must start or stop too, and the operator decides */
async function topicAction(act, name, query = '') {
  ui.error = null;
  try {
    await api.post(`/topics/${name}/${act}${query}`);
  } catch (e) {
    const missing = e.detail?.missing, dependents = e.detail?.dependents;
    if (e.status === 409 && act === 'start' && missing) {
      const s = missing.length > 1;
      if (await confirmBox(`${name} needs ${missing.join(' and ')}, which ${s ? 'are' : 'is'} stopped. Start ${s ? 'them' : 'it'} first?`, `Start ${missing.length + 1}`))
        return topicAction('start', name, '?with_deps=true');
    } else if (e.status === 409 && act === 'stop' && dependents) {
      if (await confirmBox(`${dependents.join(' and ')} depend${dependents.length > 1 ? '' : 's'} on ${name} and will stop too.`, `Stop ${dependents.length + 1}`))
        return topicAction('stop', name, '?cascade=true');
    } else ui.error = `${name}: ${e.message}`;
  }
  renderDock(true);
}

/* ---- data ---- */
let robot, mods, topics;
try {
  [robot, mods, topics] = await Promise.all([api.get('/robot'), api.get('/mods'), api.get('/topics')]);
} catch (err) {
  document.body.innerHTML = '<p class="needserver">The robot API is not reachable. Start it from the repo root with <code>uv run python -m myaloha panel</code> (or <code>--sim</code> without hardware); it serves this page at <code>http://127.0.0.1:8765/</code>.</p>';
  throw err;
}
ctx.robot = robot;
ctx.mods = Object.fromEntries(mods.map(m => [m.name, m]));
ctx.topics = Object.fromEntries(topics.map(t => [t.name, t]));
let savedTopics = {};
try { savedTopics = JSON.parse(localStorage.getItem('myaloha-topics') || '{}'); } catch {}
ui.topics = Object.fromEntries(topics.map(t => [t.name, { eye: true, mode: 'stream', period: 500, ...savedTopics[t.name] }]));
ui.selected = mods[0]?.name;
if (robot.sim) document.title = 'myaloha panel (sim)';

const geos = {}, assets = {};
for (const [model, file] of Object.entries(CFG.robots.specs)) assets[model] = await loadArmAssets(CFG.robots.dir, file, geos);
const host = $('scene');
scene = makeScene(host, CFG, assets, { arms: {} }, theme);
scene.setInsets({ r: 380, t: 210, b: 222 });   // keep the rig in the gap between the panels
applyFold();
/* param help: one popover for every (?), placed left of the dock beside the mark */
function showHelp(mark) {
  const pop = $('qpop');
  pop.innerHTML = W.esc(mark.dataset.help) + (mark.dataset.figure ? `<img src="assets/figures/${mark.dataset.figure}" alt="">` : '');
  pop.hidden = false;
  const r = mark.getBoundingClientRect(), dock = $('dock').getBoundingClientRect();
  pop.style.left = `${Math.max(8, dock.left - pop.offsetWidth - 12)}px`;
  pop.style.top = `${Math.min(innerHeight - pop.offsetHeight - 8, Math.max(8, r.top - 24))}px`;
}
document.addEventListener('mouseover', e => { const m = e.target.closest('.qhelp'); if (m) showHelp(m); });
document.addEventListener('focusin', e => { const m = e.target.closest('.qhelp'); if (m) showHelp(m); });
document.addEventListener('mouseout', e => { if (e.target.closest('.qhelp')) $('qpop').hidden = true; });
document.addEventListener('focusout', e => { if (e.target.closest('.qhelp')) $('qpop').hidden = true; });

/* 3-D view controls */
const showView = st => document.querySelectorAll('#views [data-view]').forEach(b => {
  const k = b.dataset.view;
  if (k !== 'drape') b.setAttribute('aria-pressed', String(k in st ? !!st[k] : false));
});
let freeCam = false;
try { freeCam = localStorage.getItem('myaloha-freecam') === 'on'; } catch {}
showView(scene.setFree(freeCam));
host.addEventListener('viewchange', () => showView(scene.viewState()));
/* top camera draped on the table (its pose from the latest localize run), live frames at ~6 Hz */
let drapeOn = false;
async function drapeLoop() {
  while (drapeOn) {
    try {
      const r = await fetch('/api/cameras/high/frame?kind=color', { cache: 'no-store' });
      if (r.ok && drapeOn) scene.setDrapeImage(await createImageBitmap(await r.blob()));
    } catch {}
    await new Promise(res => setTimeout(res, 160));
  }
}
function toggleDrape() {
  const btn = document.querySelector('[data-view="drape"]');
  if (drapeOn) { drapeOn = false; scene.setDrape(null); btn.setAttribute('aria-pressed', 'false'); return; }
  if (drapeBlocked()) return;
  drapeOn = true;
  btn.setAttribute('aria-pressed', 'true');
  scene.setDrape(ctx.coordinates.cameras.high);
  drapeLoop();
}

function viewAction(k) {
  if (k === 'drape') { toggleDrape(); return; }
  if (k === 'free') {
    freeCam = !freeCam;
    try { localStorage.setItem('myaloha-freecam', freeCam ? 'on' : 'off'); } catch {}
    showView(scene.setFree(freeCam));
  } else showView(scene.setView(k));
}
const VIEW_KEYS = { q: 'left', e: 'right', h: 'home', r: 'top', f: 'front', v: 'free', t: 'drape' };
document.addEventListener('keydown', e => {
  const k = VIEW_KEYS[e.key.toLowerCase()];
  if (k && !e.metaKey && !e.ctrlKey && !e.altKey && !e.target.matches('input, textarea')) viewAction(k);
});

host.addEventListener('rendered', () => {
  if (!S) return;
  const axisLabels = scene.axisTips().map(([name, xyz]) => {
    const p = scene.projectPoint(xyz);
    return `<div class="axis-tag ${name === 'W' ? 'origin' : name}" style="left:${p.x}px;top:${p.y + (name === 'W' ? 16 : 0)}px">${name}</div>`;
  }).join('');
  const camLabels = scene.cameraTips().map(([key, xyz]) => {
    const p = scene.projectPoint(xyz);
    return `<div class="cam-tag" style="left:${p.x}px;top:${p.y}px">${W.esc(S.cameras?.[key]?.label ?? key)}</div>`;
  }).join('');
  $('tags').innerHTML = axisLabels + camLabels + W.ARM_ORDER.filter(n => S.arms[n]).map(n => {
    const a = S.arms[n], p = scene.project(n);
    return `<div class="tag ${W.armLive(a) ? '' : 'off'}" style="left:${p.x}px;top:${p.y}px" title="${W.esc(a.error || a.ip)}">${W.dot(W.armKind(a))}${W.armLabel(n)}<span class="sub">${W.modeText(a)}</span></div>`;
  }).join('');
});

/* camera frames: one socket for all four; each frame replaces its tile's image in place */
const camURL = {};
const setCamKind = api.cameras((key, kind, blob) => {
  if ((ui.cams[key] ?? 'color') !== kind) return;           // a frame from before the switch
  const img = $('cams').querySelector(`img[data-feed="${key}"]`);
  if (!img) return;
  const url = URL.createObjectURL(blob);
  img.src = url;
  if (camURL[key]) URL.revokeObjectURL(camURL[key]);
  camURL[key] = url;
});

let frame = 0, coordId;
send = api.stream({
  link(up) { link = up; renderBrand(); renderDock(true); },
  state(msg) {
    const first = !S;
    S = msg; serverNow = msg.t;
    const cid = S.coordinates?.id ?? null;
    if (cid !== coordId) {
      coordId = cid;
      (cid ? api.get('/coordinates') : Promise.resolve(null)).then(doc => {
        ctx.coordinates = doc;
        scene.setCoordinates(doc);
        if (drapeOn) scene.setDrape(doc?.cameras?.high ?? null);
        renderDrapeButton();
        renderDock(true);
      }).catch(() => {});
    }
    if (first && W.active(S)) ui.selected = W.active(S).name;
    if (!frame) frame = requestAnimationFrame(() => {   // one paint per display frame at most
      frame = 0;
      scene.update(S);
      renderBrand(); renderBrowser(); renderDock(); renderCams(); renderLog(); renderTopics();
      syncTopics();
    });
  },
  log(msg) {
    ctx.log.push(...msg.records);
    if (ctx.log.length > 2000) ctx.log.splice(0, ctx.log.length - 2000);
    appendLog(msg.records);
    renderLog();
  },
});

/* ---- commands ---- */
async function call(fn) {
  ui.error = null;
  try { await fn(); } catch (e) { ui.error = e.message; }
  renderDock(true);
}

$('q').addEventListener('input', e => { ui.query = e.target.value.trim(); renderBrowser(true); });
$('topics').addEventListener('change', e => {
  const name = e.target.dataset.tperiod;
  if (!name) return;
  ui.topics[name].period = Math.max(50, Math.round(Number(e.target.value)) || 500);
  topicUIChanged();
});
document.addEventListener('keydown', e => {
  if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'k') { e.preventDefault(); $('q').focus(); $('q').select(); }
  if (e.key === 'Escape' && document.activeElement === $('q')) { $('q').value = ''; ui.query = ''; renderBrowser(true); $('q').blur(); }
});
document.addEventListener('click', e => {
  const t = e.target;
  const cat = t.closest('[data-cat]');
  if (cat) { ui.cat = cat.dataset.cat; renderBrowser(true); return; }
  const mod = t.closest('[data-mod]');
  if (mod) { ui.selected = mod.dataset.mod; ui.error = null; renderBrowser(true); renderDock(true); return; }
  const cam = t.closest('[data-cam]');
  if (cam && !cam.classList.contains('nofeed')) {
    const k = cam.dataset.cam;
    ui.cams[k] = ui.cams[k] === 'depth' ? 'color' : 'depth';
    setCamKind(k, ui.cams[k]);
    renderCams(true);
    return;
  }
  const tog = t.closest('button[data-param]');
  if (tog && !tog.disabled) {
    tog.classList.toggle('on');
    if ('bool' in tog.dataset) tog.textContent = tog.classList.contains('on') ? 'on' : 'off';
    return;
  }
  const teye = t.closest('[data-teye]');
  if (teye) {
    if (!teye.disabled) { ui.topics[teye.dataset.teye].eye = !ui.topics[teye.dataset.teye].eye; topicUIChanged(); }
    return;
  }
  const tact = t.closest('[data-tact]');
  if (tact) { if (!tact.disabled) topicAction(tact.dataset.tact, tact.dataset.topicName); return; }
  const tmode = t.closest('[data-tmode]');
  if (tmode) { ui.topics[tmode.dataset.topicName].mode = tmode.dataset.tmode; topicUIChanged(); return; }
  const thead = t.closest('[data-topic]');
  if (thead && !t.closest('input')) { ui.topicOpen = ui.topicOpen === thead.dataset.topic ? null : thead.dataset.topic; renderTopics(true); return; }
  const lf = t.closest('[data-logf]');
  if (lf) { ui.log = lf.dataset.logf; refilterLog(); return; }
  if (t.closest('#run')) {
    const params = W.readParams($('dock-body'), ctx.mods[ui.selected]);
    call(() => api.post(`/mods/${ui.selected}/run`, { params }));
    return;
  }
  const vb = t.closest('[data-view]');
  if (vb) { viewAction(vb.dataset.view); return; }
  if (t.closest('#fold')) {
    foldChoice = document.body.classList.contains('folded') ? 'open' : 'folded';
    try { localStorage.setItem('myaloha-fold', foldChoice); } catch {}
    applyFold();
    return;
  }
  if (t.closest('#stop')) { call(() => api.post('/stop')); return; }
  const reset = t.closest('#estop-reset');
  if (reset) {                                   // two clicks: releasing the latch is deliberate
    if (!reset.classList.contains('armed')) {
      reset.classList.add('armed'); reset.textContent = 'Click again to reset';
      setTimeout(() => { reset.classList.remove('armed'); reset.textContent = 'Reset'; }, 3000);
    } else {
      reset.classList.remove('armed'); reset.textContent = 'Reset';
      call(() => api.post('/estop/reset'));
    }
    return;
  }
  if (t.closest('#reconnect')) {
    const btn = $('reconnect');
    btn.disabled = true; btn.querySelector('span').textContent = 'Connecting…';
    call(() => Promise.all([api.post('/arms/connect'), api.post('/cameras/connect')]))
      .finally(() => { btn.disabled = false; btn.querySelector('span').textContent = 'Reconnect'; });
    return;
  }
  if (t.closest('#theme')) {
    theme = theme === 'light' ? 'dark' : 'light';
    document.documentElement.dataset.theme = theme;
    try { localStorage.setItem('myaloha-theme', theme); } catch {}
    scene.setTheme(theme);
  }
});
window.__ready = true;
