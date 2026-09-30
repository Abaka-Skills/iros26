/* pickandplay.html, the iros26 mod's page: click a cube, then where it goes (on the table, or on
   another cube to stack it); "Pick and place" sends the choice to the running mod
   (POST /api/mods/iros26/input), which carries it out with pick_place. Live state comes from the
   /api/stream WebSocket, the cubes from the locate_cubes topic; the page holds only the choice
   being made. With Auto on, a page no one has used (no mouse event) for 60 s asks the mod for an auto
   round (a random cube, handed over to the other arm's side), and another 20 s after each ends. */
import * as api from './api.js';
import * as W from './widgets.js';
import { loadArmAssets } from './arm.js';
import { makeScene } from './scene.js';

const $ = id => document.getElementById(id);

/* ---- emergency stop first: armed before anything else loads, fires on press ---- */
let send = () => false, scene = null, error = null;
function emergencyStop() {
  api.emergencyStop(send).catch(e => { error = `E-stop REST failed: ${e.message}`; draw(); });
}
$('estop').innerHTML = `${W.icon('octagon')}E-STOP<kbd>Space</kbd>`;
$('estop').addEventListener('pointerdown', e => { e.preventDefault(); emergencyStop(); });
document.addEventListener('keydown', e => {
  if ((e.code === 'Space' || e.key === ' ') && !e.repeat && !e.target.matches('input, textarea')) { e.preventDefault(); emergencyStop(); }
}, true);

const CFG = await (await fetch('./config.json', { cache: 'no-store' })).json();
const U = CFG.unitsPerMeter, SIZE = CFG.cubes.size;
let theme = CFG.defaultTheme;
try { theme = localStorage.getItem('myaloha-theme') || theme; } catch {}
document.documentElement.dataset.theme = theme;

/* ---- state ---- */
let S = null, link = false, cubes = [], note = null, sending = false;
const choice = { track: null, target: null };   // target: {x, y, on: the track of the cube it goes on, or null}
let sent = null;                                 // the choice being carried out: {track, color, from, target {x, y, z}, rounds}

const iros = () => S?.mods?.find(m => m.name === 'iros26') ?? null;
const child = () => S?.mods?.find(m => m.name === 'pick_place') ?? null;
const cubeOf = track => cubes.find(c => c.track === track);
/* what iros26 carries out: a choice made here, or an auto round's (pick_place's params, once it chose) */
function doing() {
  const m = iros()?.choice, p = child()?.params;
  return m?.auto ? (p?.track ? { auto: true, track: p.track, target_x: p.target_x, target_y: p.target_y } : { auto: true }) : m;
}
const choosable = c => !!c && !!c.color && c.level !== null;      // colour known, on the table or a stack
const xy = (x, y) => `(${x.toFixed(2)}, ${y.toFixed(2)})`;
const swatch = color => `<i class="sw" style="background:${CFG.cubes.colors[color] ?? CFG.cubes.hidden}"></i>`;

/* offline (no API), ended (iros26 not running), starting, choosing (iros26 waiting), moving */
function phase() {
  if (!link || !S) return 'offline';
  const m = iros();
  return !m ? 'ended' : { waiting: 'choosing', moving: 'moving' }[m.state] ?? 'starting';
}
/* where the cube's centre ends up: on the table, or on the cube it goes on */
function centre(t) {
  const under = t.on !== null ? cubeOf(t.on) : null;
  return { x: t.x, y: t.y, z: under ? under.z + SIZE : SIZE / 2 };
}
function clearChoice() { choice.track = null; choice.target = null; }

/* ---- the scene ---- */
const geos = {}, assets = {};
assets.follower = await loadArmAssets(CFG.robots.dir, CFG.robots.specs.follower, geos);   // no leaders here
const host = $('scene');
/* from the operator's side of the table, facing the arms, nearly straight down and square to
   the table: the cubes are small, and choosing is on the table's plane (never quite 90 deg: straight
   down, the view's roll is undefined) */
scene = makeScene(host, { ...CFG, camera: { ...CFG.camera, azimuth: 180, elevation: 68, topElevation: 86 } },
  assets, { arms: {} }, theme, { leaders: false, watermark: CFG.watermark });
scene.setInsets({ l: 0, r: 300, t: 70, b: 70 });

/* view controls, as in the operator panel: 90 deg turns, presets, free rotate (drag orbits) or free
   drag (drag pans), wheel zooms, right-drag pans; the free mode is remembered in this browser */
const showView = st => document.querySelectorAll('#views [data-view]').forEach(b =>
  b.setAttribute('aria-pressed', String(!!st[b.dataset.view])));
let freeCam = false;                               // false | 'rotate' | 'drag'
try { const v = localStorage.getItem('myaloha-pp-freecam'); freeCam = v === 'rotate' || v === 'drag' ? v : false; } catch {}
showView(scene.setFree(freeCam));
host.addEventListener('viewchange', () => showView(scene.viewState()));
function viewAction(k) {
  if (k !== 'rotate' && k !== 'drag') { showView(scene.setView(k)); return; }
  freeCam = freeCam === k ? false : k;
  try { localStorage.setItem('myaloha-pp-freecam', freeCam || 'off'); } catch {}
  showView(scene.setFree(freeCam));
}
const VIEW_KEYS = { q: 'left', e: 'right', h: 'home', r: 'top', f: 'front', v: 'rotate', g: 'drag' };
document.addEventListener('keydown', e => {
  const k = VIEW_KEYS[e.key.toLowerCase()];
  if (k && !e.metaKey && !e.ctrlKey && !e.altKey && !e.target.matches('input, textarea')) viewAction(k);
});
W.ctx.robot = await api.get('/robot').catch(() => null);

let marksKey = '';
function marks() {
  let track = choice.track, target = choice.target && { ...centre(choice.target), color: cubeOf(choice.track)?.color };
  let from = null;                                 // where the arrow starts once the cube is in a gripper (untracked)
  if (phase() === 'moving') {
    const m = doing();
    track = sent?.track ?? m?.track ?? null;
    from = sent?.from ?? null;
    target = sent ? { ...sent.target, color: sent.color }
      : m?.track ? { x: m.target_x, y: m.target_y, z: SIZE / 2, color: cubeOf(m.track)?.color } : null;
  }
  const key = JSON.stringify([track, target, from]);
  if (key !== marksKey) { marksKey = key; scene.setMarks({ track, target, from }); }
}

function onCubes(list) {
  cubes = list ?? [];
  scene.setCubes(cubes);
  if (choice.track !== null && !choosable(cubeOf(choice.track))) {
    clearChoice();
    note = 'The chosen cube is no longer seen on the table: choose again.';
  }
  marks();
  draw();
}
let sub = null;
function syncCubes() {
  const t = S?.topics?.locate_cubes, url = W.topicLive(t) && t.url ? t.url : null;
  if ((sub?.url ?? null) === url) return;
  sub?.close();
  sub = url && { url, close: api.topic(url, 'stream', 0, m => onCubes(m.data?.cubes)) };
  if (!url) onCubes([]);
}

let coordId;
function syncCoordinates() {                    // the calibration, the reach and the safety boundary it is clipped to
  const cid = S.coordinates?.id ?? null, key = JSON.stringify([cid, S.boundary?.box]);
  if (key === coordId) return;
  coordId = key;
  scene.setBoundary(S.boundary ?? null);
  Promise.all(cid ? [api.get('/coordinates'), api.get('/reach')] : [null, null]).then(([doc, reach]) => {
    scene.setCoordinates(doc);
    scene.setReach(reach);
  }).catch(() => { coordId = undefined; });     // again on the next state frame
}

/* ---- the card: three steps, then what happened to each choice ---- */
function step(n, state, title, text, what) {
  return `<li class="step ${state}"><i class="n">${n}</i><div><b>${title}</b>${text ? `<p>${text}</p>` : ''}${what ?? ''}</div></li>`;
}
function cubeLine(track, color, c, redo) {
  return `<div class="what">${swatch(color)}${W.esc(color ?? 'cube')} cube ${track}${c ? ` <span class="sub">${xy(c.x, c.y)}</span>` : ''}` +
    `${redo ? '<button class="lk" data-redo="cube">change</button>' : ''}</div>`;
}
function targetLine(t, redo) {
  const under = t.on !== null && cubeOf(t.on);
  return `<div class="what">${under ? `on ${swatch(under.color)}${W.esc(under.color)} cube ${t.on}` : 'on the table'} <span class="sub">${xy(t.x, t.y)}</span>` +
    `${redo ? '<button class="lk" data-redo="target">change</button>' : ''}</div>`;
}
function card() {
  const ph = phase(), m = iros();
  if (ph === 'offline') return '<h4>Pick and place</h4><div class="ended">The robot API is offline: reconnecting…</div>';
  if (ph === 'ended') return `<h4>Pick and place</h4><div class="ended">The <b>iros26</b> mod is not running, so nothing can be moved from here.
    Start it in the <a href="/">operator panel</a>; this page picks up again once it runs.</div>${rounds(S.last_run?.name === 'iros26' ? S.last_run.rounds : [])}`;
  const p = m.params;
  const opts = `<div class="sub opts">arm ${W.esc(p.arm)} · ${p.speed} m/s · ${p.tries} tries${p.handover ? ' · handover' : ''}${p.dry_run ? ' · <b>dry run: plans only</b>' : ''}</div>`;
  const cubesDown = !W.topicLive(S.topics?.locate_cubes);
  let body;
  if (ph === 'moving' || ph === 'starting') {
    const c = child(), d = doing(), t = sent?.target ?? (d?.track ? { x: d.target_x, y: d.target_y } : null);
    const track = sent?.track ?? d?.track;
    body = `<ol class="steps">
      ${step(1, 'done', d?.auto ? 'The cube <span class="sub">auto</span>' : 'The cube', d?.auto && !track ? 'Choosing one at random…' : '',
        track ? cubeLine(track, sent?.color ?? cubeOf(track)?.color) : '')}
      ${step(2, 'done', 'Where it goes', '', t ? `<div class="what">${xy(t.x, t.y)}</div>` : '')}
      ${step(3, 'cur', 'Pick and place', c ? `<b>${c.state}</b> — ${W.esc(c.states[c.state] ?? '')}` : 'starting…')}</ol>
      <div class="actions"><button class="btn stop big" id="stop" title="Stop the arms where they are and end iros26">${W.icon('stop')}Stop</button></div>`;
  } else {
    const c = choice.track !== null ? cubeOf(choice.track) : null, t = choice.target;
    const ready = c && t && !sending && !cubesDown;
    body = `<ol class="steps">
      ${step(1, c ? 'done' : 'cur', 'Choose a cube', c ? '' : 'Click a cube on the table.', c ? cubeLine(c.track, c.color, c, true) : '')}
      ${step(2, t ? 'done' : c ? 'cur' : '', 'Choose where it goes', t ? '' : 'Click the table, or a cube to stack on.', t ? targetLine(t, true) : '')}
      ${step(3, ready ? 'cur' : '', 'Pick and place', 'The arms move it; the cameras check every step.')}</ol>
      <div class="actions"><button class="btn big" id="clear" ${c || t ? '' : 'disabled'}>Clear</button>
        <button class="btn run big" id="go" ${ready ? '' : 'disabled'}>${W.icon('run')}<span class="lbl">${sending ? 'Sending…' : 'Pick and place'}<span class="auto-in"></span></span></button></div>`;
  }
  const hints = (cubesDown ? '<div class="hint err">locate_cubes is not running: start it in the operator panel to see the cubes.</div>' : '')
    + (error ? `<div class="hint err">${W.esc(error)}</div>` : '') + (note ? `<div class="hint">${W.esc(note)}</div>` : '');
  return `<h4>Pick and place <span class="sub">iros26 · ${m.state}</span></h4>${opts}${body}${hints}${rounds(m.rounds)}`;
}
function rounds(list) {
  if (!list?.length) return '';
  return `<div class="rounds"><h4>Done so far <span class="sub">${list.length}</span></h4>${[...list].reverse().map(r =>
    `<div class="round"><span class="r ${r.result}">${r.result === 'done' ? '✓' : '✗'}</span><span>${r.track ? `cube ${r.track} → ${xy(r.target_x, r.target_y)}` : 'no cube chosen'} <span class="sub">${r.auto ? 'auto · ' : ''}${r.result}</span></span>` +
    `${r.error ? `<span class="e">${W.esc(r.error)}</span>` : ''}</div>`).join('')}</div>`;
}

/* the auto countdown on the Pick and place button: a fill that grows until the auto round starts, and
   the seconds left (set by autoTick; painted again whenever the card is redrawn) */
const countdown = { on: false, frac: 0, secs: 0 };
function paintCountdown() {
  const go = $('go');
  if (!go) return;
  go.classList.toggle('counting', countdown.on);
  go.style.setProperty('--auto', countdown.on ? countdown.frac.toFixed(3) : '0');
  go.querySelector('.auto-in').textContent = countdown.on ? `auto in ${countdown.secs} s` : '';
}

/* each region redraws only when what it shows changed (a redraw under the pointer swallows clicks) */
const last = {};
const changed = (region, key) => (last[region] !== key ? ((last[region] = key), true) : false);
function draw() {
  if (!scene) return;
  if (changed('brand', JSON.stringify([link, S?.state, S?.mods?.map(x => [x.name, x.state])])))
    $('brand').innerHTML = `<b class="brand-name">Pick and place</b>${W.majorStrip(S, link)}`;
  const estopped = S?.state === 'estop';
  $('estop').classList.toggle('on', estopped);
  $('estop-banner').hidden = !estopped;
  const c = choice.track !== null ? cubeOf(choice.track) : null;
  const key = JSON.stringify([phase(), choice, c && [c.color, xy(c.x, c.y)], sent, sending, error, note, iros()?.rounds, doing(),
    iros()?.params, child()?.state, S?.last_run?.rounds, W.topicLive(S?.topics?.locate_cubes)]);
  if (changed('card', key)) { $('card').innerHTML = card(); paintCountdown(); }
}

/* labels: the cube under the pointer, and the target */
host.addEventListener('rendered', () => {
  const h = scene.hoveredCube(), c = h?.cube;
  const tip = c ? `<div class="tag" style="left:${h.x}px;top:${h.y}px">cube ${c.track} · ${W.esc(c.color ?? 'hidden')}` +
    `<span class="sub">${c.level === null ? 'in the air' : `level ${c.level}`} · ${xy(c.x, c.y)}</span></div>` : '';
  const t = phase() === 'choosing' ? choice.target && centre(choice.target) : phase() === 'moving' ? sent?.target : null;
  const p = t && scene.projectPoint([t.x * U, (t.z + SIZE) * U, -t.y * U]);
  $('tags').innerHTML = tip + (p ? `<div class="tag" style="left:${p.x}px;top:${p.y}px">target<span class="sub">${xy(t.x, t.y)}</span></div>` : '');
});

/* ---- choosing on the rig: a cube first, then where it goes ---- */
const canvas = host.querySelector('canvas');
host.addEventListener('pointermove', () => {
  const c = scene.hoveredCube()?.cube, choosing = phase() === 'choosing';
  canvas.classList.toggle('pointing', choosing && choosable(c));
  canvas.classList.toggle('aiming', choosing && !choosable(c) && choice.track !== null);
});
let downAt = [0, 0];
host.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; });
host.addEventListener('click', e => {
  if (phase() !== 'choosing' || sending) return;
  if (Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 5) return;   // a drag (free view), not a choice
  const hit = scene.pick(e.clientX, e.clientY);
  if (!hit) return;
  const c = hit.cube;
  note = error = null;
  if (c && !choosable(c)) note = c.color ? 'That cube is in the air (in a gripper).' : 'That cube is hidden under a stack.';
  else if (choice.track === null) {
    if (c) choice.track = c.track;
    else note = 'Choose a cube first: click one on the table.';
  } else if (c?.track === choice.track) clearChoice();                  // the chosen cube again: unchoose it
  else choice.target = c ? { x: c.x, y: c.y, on: c.track } : { x: hit.point[0], y: hit.point[1], on: null };
  marks();
  draw();
});

async function go() {
  const c = cubeOf(choice.track), t = choice.target;
  sending = true; error = note = null;
  draw();
  try {
    await api.post('/mods/iros26/input', { track: choice.track, target_x: t.x, target_y: t.y });
    sent = { track: choice.track, color: c?.color, from: c && { x: c.x, y: c.y, z: c.z }, target: centre(t),
             rounds: iros()?.rounds.length ?? 0 };
    clearChoice();
  } catch (e) { error = e.message; }
  sending = false;
  marks();
  draw();
}

document.addEventListener('click', e => {
  const t = e.target;
  if (t.closest('#go')) { if (!t.closest('#go').disabled) go(); return; }
  if (t.closest('#clear')) { clearChoice(); note = error = null; marks(); draw(); return; }
  const redo = t.closest('[data-redo]');
  if (redo) {
    if (redo.dataset.redo === 'cube') clearChoice(); else choice.target = null;
    marks(); draw();
    return;
  }
  const vb = t.closest('[data-view]');
  if (vb) { viewAction(vb.dataset.view); return; }
  if (t.closest('#stop')) { api.post('/stop').catch(err => { error = err.message; draw(); }); return; }
  const reset = t.closest('#estop-reset');
  if (reset) {                                   // two clicks: releasing the latch is deliberate
    if (!reset.classList.contains('armed')) {
      reset.classList.add('armed'); reset.textContent = 'Click again to reset';
      setTimeout(() => { reset.classList.remove('armed'); reset.textContent = 'Reset'; }, 3000);
    } else {
      reset.classList.remove('armed'); reset.textContent = 'Reset';
      api.post('/estop/reset').catch(err => { error = err.message; draw(); });
    }
  }
});
document.addEventListener('keydown', e => {
  if (e.key === 'Escape' && phase() === 'choosing') { clearChoice(); note = error = null; marks(); draw(); }
});

/* ---- auto: on, a round starts once no mouse event came for COLD, the next HOT after an auto round ends;
   a mouse event makes it COLD again (a round under way goes on: Stop ends it) ---- */
const COLD = 60e3, HOT = 20e3;
let autoOn = false, lastAct = Date.now(), lastEnd = 0, endedAuto = false, wasMoving = false;
try { autoOn = localStorage.getItem('myaloha-pp-auto') === 'on'; } catch {}
for (const ev of ['pointermove', 'pointerdown', 'wheel'])
  document.addEventListener(ev, () => { lastAct = Date.now(); }, { capture: true, passive: true });
async function goAuto() {
  sending = true; error = note = null;
  clearChoice(); marks(); draw();
  try { await api.post('/mods/iros26/input', { auto: true }); }
  catch (e) { error = `Auto: ${e.message}`; }
  lastEnd = Date.now(); endedAuto = true;          // HOT before another, sent or refused
  sending = false;
  draw();
}
function autoTick() {
  const ph = phase(), now = Date.now();
  if (wasMoving && ph !== 'moving') { lastEnd = now; endedAuto = !!iros()?.rounds.at(-1)?.auto; }
  wasMoving = ph === 'moving';
  const hot = endedAuto && lastEnd > lastAct;
  const left = (hot ? HOT : COLD) - (now - Math.max(lastAct, lastEnd));
  if (autoOn && ph === 'choosing' && !sending && left <= 0) goAuto();
  const span = hot ? HOT : COLD;
  Object.assign(countdown, { on: autoOn && ph === 'choosing' && !sending,
    frac: Math.min(1, Math.max(0, 1 - left / span)), secs: Math.max(0, Math.ceil(left / 1000)) });
  paintCountdown();
  $('auto').setAttribute('aria-pressed', String(autoOn));
  $('auto-note').textContent = !autoOn ? 'off' : ph === 'moving' ? (doing()?.auto ? 'moving a cube' : '')
    : ph !== 'choosing' ? '' : `${hot ? 'next' : 'starts'} in ${Math.max(0, Math.ceil(left / 1000))} s`;
}
$('auto').addEventListener('click', () => {
  autoOn = !autoOn;
  try { localStorage.setItem('myaloha-pp-auto', autoOn ? 'on' : 'off'); } catch {}
  autoTick();
});
setInterval(autoTick, 250);
autoTick();

/* ---- live ---- */
let frame = 0;
send = api.stream({
  link(up) { link = up; draw(); },
  state(msg) {
    S = msg;
    if (sent && phase() !== 'moving' && (!iros() || iros().rounds.length > sent.rounds)) sent = null;   // that one ended
    if (!frame) frame = requestAnimationFrame(() => {    // one paint per display frame at most
      frame = 0;
      syncCoordinates();
      syncCubes();
      scene.update(S);
      marks();
      draw();
    });
  },
});
window.__ready = true;
