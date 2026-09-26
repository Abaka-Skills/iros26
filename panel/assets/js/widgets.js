/* Panel building blocks as HTML strings. Every widget reads the status shape the backend
   will serve (Aloha.status() + cameras + timeline + log); nothing here is layout-specific. */
import { MODS, PLANNED, CATEGORIES, ROBOT_STATES, TIMELINE, LOG } from './mock.js';

export const ARM_ORDER = ['leader_left', 'follower_left', 'leader_right', 'follower_right'];
export const CAM_ORDER = ['high', 'low', 'wrist_a', 'wrist_b'];
const JOINTS = ['j0', 'j1', 'j2', 'j3', 'j4', 'j5', 'grip'];
const MODE = { position: 'position', external_effort: 'gravity comp', idle: 'braked', velocity: 'velocity', effort: 'effort' };

export const armLabel = n => n.replace('leader_', 'Leader ').replace('follower_', 'Follower ').replace('left', 'L').replace('right', 'R');
export const octet = ip => '.' + ip.split('.').pop();

/* A missing leader is expected and shown as absent; a missing follower is an error. */
export const armKind = a => a.connected ? 'ok' : a.role === 'leader' ? 'off' : 'err';
export const modeText = a => a.connected ? a.mode.map(m => MODE[m] || m).join(' / ') : 'absent';

export const dot = k => `<i class="dot ${k}"></i>`;
const ICON = {
  run: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5.5v13l10.5-6.5z"/></svg>',
  stop: '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6.5" y="6.5" width="11" height="11" rx="2"/></svg>',
  theme: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7"><circle cx="12" cy="12" r="7.5"/><path d="M12 4.5a7.5 7.5 0 0 1 0 15Z" fill="currentColor"/></svg>',
  search: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><circle cx="11" cy="11" r="6.5"/><path d="m16 16 4 4"/></svg>',
  chev: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="m7 10 5 5 5-5"/></svg>',
};
export const icon = k => ICON[k];

/* ---------------- running mod ---------------- */
export const active = S => S.mods[S.mods.length - 1] || null;
const NOW = () => Date.now() / 1000;
const secs = t => t < 10 ? t.toFixed(1) + ' s' : t < 90 ? t.toFixed(0) + ' s' : (t / 60).toFixed(1) + ' min';

/* the longest path from idle through the mod's own transitions, then done */
export function mainPath(tr) {
  const memo = {};
  const longest = s => memo[s] ??= [s, ...(tr[s] || []).map(longest).reduce((a, b) => (b.length > a.length ? b : a), [])];
  return [...longest('idle'), 'done'];
}

/* seconds spent in each visited state; the current one is still counting */
function durations(m) {
  const d = {};
  m.history.forEach(([s, t], i) => { d[s] = (m.history[i + 1] ? m.history[i + 1][1] : NOW()) - t; });
  return d;
}

/* what each mod needs from the hardware right now */
export function availability(name, S) {
  const A = S.arms, has = n => A[n].connected;
  if (name === 'reset') {
    const n = Object.values(A).filter(a => a.connected).length;
    return [[n ? 'ok' : 'err', `${n}/4 arms`]];
  }
  if (name === 'teleop') return ['left', 'right'].map(s => {
    const ok = has('leader_' + s) && has('follower_' + s);
    return [ok ? 'ok' : 'off', `${s} pair${ok ? '' : ': no leader'}`];
  });
  if (name === 'instruction') return ['left', 'right'].map(s => [has('follower_' + s) ? 'ok' : 'err', `follower ${s[0].toUpperCase()}`]);
  return [];
}

/* every mod the browser lists: real ones from Python, then planned examples */
export const ALL_MODS = () => [
  ...Object.entries(MODS).map(([name, m]) => ({ name, category: m.category, doc: m.doc })),
  ...PLANNED,
];
function modDot(S, m) {
  if (m.planned) return 'off';
  if (active(S) && active(S).name === m.name) return 'live';
  return availability(m.name, S).some(([k]) => k === 'ok') ? 'ok' : 'err';
}

/* ---------------- watchdog ---------------- */
export function watchdog(S, { title = true } = {}) {
  const arms = ARM_ORDER.map(n => S.arms[n]);
  const cams = CAM_ORDER.map(k => S.cameras[k]);
  return `
  <div class="wd-group">
    ${title ? `<h4>Arms <span class="sub">${arms.filter(a => a.connected).length}/4</span></h4>` : ''}
    ${arms.map(a => `<div class="wd-row">${dot(armKind(a))}<span class="k">${armLabel(a.name)}<span class="sub">${octet(a.ip)}</span></span><span class="mode ${a.connected ? '' : 'absent'}">${modeText(a)}</span></div>`).join('')}
  </div>
  <div class="wd-group">
    ${title ? `<h4>Cameras <span class="sub">${cams.filter(c => c.ok).length}/4</span></h4>` : ''}
    ${cams.map(c => `<div class="wd-row">${dot(c.ok ? 'ok' : 'err')}<span class="k">${c.label}<span class="sub">·${c.serial.slice(-4)}</span></span><span class="mode num">${c.fps.toFixed(0)} fps</span></div>`).join('')}
  </div>`;
}

export function leds(S) {
  return `<div class="leds">
    ${ARM_ORDER.map(n => S.arms[n]).map(a => `<span>${dot(armKind(a))}${armLabel(a.name)}</span>`).join('')}
    <span style="color:var(--faint)">|</span>
    ${CAM_ORDER.map(k => S.cameras[k]).map(c => `<span>${dot(c.ok ? 'ok' : 'err')}${c.label}</span>`).join('')}
  </div>`;
}

/* ---------------- arm cards ---------------- */
export function jointBars(pos) {
  return JOINTS.map((j, i) => {
    const v = pos[i], grip = i === 6;
    const f = grip ? Math.min(1, v / 0.044) : Math.max(-1, Math.min(1, v / Math.PI));
    const [l, w] = grip ? [0, f * 100] : f >= 0 ? [50, f * 50] : [50 + f * 50, -f * 50];
    return `<div class="jb"><span>${j}</span><span class="bar ${grip ? 'grip' : ''}"><i style="left:${l}%;width:${w}%"></i></span><span class="v">${grip ? (v * 1000).toFixed(0) + ' mm' : v.toFixed(2)}</span></div>`;
  }).join('');
}

export function armCard(a, { bars = true } = {}) {
  if (!a.connected) return `
    <div class="arm absent"><div class="arm-head">${dot(armKind(a))}<b>${armLabel(a.name)}</b><span class="sub">${octet(a.ip)}</span><span class="mode absent">absent</span></div>
    <div class="absent-note">${a.role === 'leader' ? 'Not connected. Teleop skips this side; reset and instruction are unaffected.' : a.error}</div></div>`;
  return `
    <div class="arm"><div class="arm-head">${dot(armKind(a))}<b>${armLabel(a.name)}</b><span class="sub">${octet(a.ip)}</span><span class="mode">${modeText(a)}</span></div>
    ${bars ? jointBars(a.pos) : ''}</div>`;
}

/* ---------------- cameras ---------------- */
export function camTile(S, key, snapDir, { fill = false, depth = false } = {}) {
  const c = S.cameras[key];
  return `<figure class="cam ${fill ? 'fill' : ''}">
    <img src="${snapDir}${c.serial}_color.png" alt="${c.label} camera">
    ${depth ? `<div class="depth"><img src="${snapDir}${c.serial}_depth.png" alt=""></div>` : ''}
    <figcaption>${dot(c.ok ? 'ok' : 'err')}${c.label}<span class="s">${c.note} · ${c.fps.toFixed(0)} fps</span></figcaption>
  </figure>`;
}

/* ---------------- mod browser ---------------- */
function modRow(S, m, selected) {
  const avail = m.planned ? '<span class="tagp">planned</span>'
    : active(S) && active(S).name === m.name ? `<span class="sub">${active(S).state}</span>`
    : `<span class="sub">${availability(m.name, S).some(([k]) => k === 'ok') ? 'ready' : 'blocked'}</span>`;
  return `<div class="mrow ${m.name === selected ? 'on' : ''} ${m.planned ? 'planned' : ''}">
    ${dot(modDot(S, m))}<span class="mname">${m.name}</span>${avail}
    <span class="mdoc">${m.doc}</span></div>`;
}

export function search(placeholder = 'Search mods', { kbd = true, value = '' } = {}) {
  return `<div class="search">${icon('search')}<span class="${value ? '' : 'ph'}">${value || placeholder}</span>${value ? '<i class="caret"></i>' : ''}${kbd ? '<kbd>⌘K</kbd>' : ''}</div>`;
}

/* Searchable, grouped by category. Scales to dozens of mods; empty categories are hidden. */
export function modBrowser(S, { selected = 'teleop', chips = true, docs = true } = {}) {
  const all = ALL_MODS();
  const ready = all.filter(m => !m.planned).length;
  return `<div class="browser ${docs ? '' : 'nodoc'}">
    ${search()}
    ${chips ? `<div class="cats"><span class="on">all ${all.length}</span>${CATEGORIES.map(c => `<span>${c}</span>`).join('')}</div>` : ''}
    <div class="mlist">${CATEGORIES.map(c => {
      const ms = all.filter(m => m.category === c);
      return ms.length ? `<div class="mcat">${c}<span>${ms.length}</span></div>${ms.map(m => modRow(S, m, selected)).join('')}` : '';
    }).join('')}</div>
    <div class="mfoot sub">${ready} available · ${all.length - ready} planned (examples)</div>
  </div>`;
}

/* Explorer tree (IDE style): folders per category, running mod highlighted */
export function modTree(S, { selected = 'teleop', open = ['motion', 'teleop', 'teach', 'data'] } = {}) {
  const all = ALL_MODS();
  return `<div class="tree">${CATEGORIES.map(c => {
    const ms = all.filter(m => m.category === c), isOpen = open.includes(c);
    return `<div class="tfold">${isOpen ? '▾' : '▸'} ${c}<span>${ms.length}</span></div>
      ${isOpen ? ms.map(m => `<div class="tleaf ${m.name === selected ? 'on' : ''} ${m.planned ? 'planned' : ''}">${dot(modDot(S, m))}${m.name}${m.planned ? '<span class="tagp">planned</span>' : ''}</div>`).join('') : ''}`;
  }).join('')}</div>`;
}

/* Combobox for a toolbar: shows the selected mod; opens the palette */
export function modCombo(S, selected = 'teleop') {
  const m = ALL_MODS().find(x => x.name === selected);
  return `<div class="combo">${dot(modDot(S, m))}<b>${m.name}</b><span class="sub">${m.category}</span>${icon('chev')}<kbd>⌘K</kbd></div>`;
}

/* the palette open, with a query typed */
export function palette(S, query = 're') {
  const hits = ALL_MODS().filter(m => m.name.includes(query));
  const hl = s => s.replace(query, `<mark>${query}</mark>`);
  return `<div class="palette panel solid">${search('', { value: query, kbd: false })}
    <div class="phits">${hits.map((m, i) => `<div class="mrow ${i === 0 ? 'on' : ''} ${m.planned ? 'planned' : ''}">${dot(modDot(S, m))}<span class="mname">${hl(m.name)}</span>
      <span class="sub">${m.planned ? '<span class="tagp">planned</span>' : m.category}</span><span class="mdoc">${m.doc}</span></div>`).join('')}</div>
    <div class="mfoot sub">↑↓ select · ↵ open params · ⌘↵ run with defaults</div></div>`;
}

/* ---------------- params ---------------- */
export function params(S, name = 'teleop', { inline = false, head = false } = {}) {
  const run = active(S) && active(S).name === name;
  const P = run ? active(S).params : MODS[name].params;
  const unit = { duration_s: 's', align_time: 's', goal_time: 's', sample_hz: 'Hz' };
  const fields = Object.entries(P).map(([k, v]) => {
    let input;
    if (k === 'sides') {
      input = `<span class="toggles">${['left', 'right'].map(s => {
        const ok = name === 'teleop' ? S.arms['leader_' + s].connected : S.arms['follower_' + s].connected;
        return `<span class="${!ok ? 'x' : v.includes(s) ? 'on' : ''}">${s}</span>`;
      }).join('')}</span>`;
    } else if (typeof v === 'boolean') input = `<span class="toggles"><span class="${v ? 'on' : ''}">${v ? 'on' : 'off'}</span></span>`;
    else input = `<span class="in">${v === 0 && k === 'duration_s' ? '∞' : v}<u>${unit[k] || ''}</u></span>`;
    return `<div class="field"><label>${k}</label>${input}</div>`;
  }).join('');
  const skip = name === 'teleop' && !S.arms.leader_right.connected
    ? `<div class="hint">● leader R absent: the right side will be skipped</div>` : '';
  const top = head ? `<div class="phead"><div><b>${name}</b><span class="sub">${MODS[name].category}</span></div><p>${MODS[name].doc}</p></div>` : '';
  return `${top}<div class="params ${inline ? 'inline' : ''}">${fields}${inline ? '' : skip}</div>`;
}

export function actions(S, { big = false } = {}) {
  const busy = !!active(S);
  return `<div class="actions">
    <button class="btn run ${big ? 'big' : ''}" ${busy ? 'disabled' : ''}>${icon('run')}Run</button>
    <button class="btn stop ${big ? 'big' : ''}">${icon('stop')}Stop</button></div>`;
}

/* ================= FSM: two levels ================= */
/* Level 1: the robot's major states, with the running mod (and its parents) as a breadcrumb. */
export function majorStrip(S) {
  const order = ['offline', 'idle', 'running'];
  const m = active(S);
  const crumbs = S.mods.map(x => `<span class="crumb">${x.name}</span>`).join('<span class="sep">›</span>');
  return `<div class="major">${order.map(s => `<span class="mst ${s === S.state ? 'cur' : ''}" title="${ROBOT_STATES[s]}">${s}</span>`).join('<i class="marr"></i>')}
    ${m ? `<span class="sep">›</span>${crumbs}<span class="sep">›</span><span class="crumb cur">${m.state}</span>` : ''}</div>`;
}

/* Level 1 over time: major-state segments for the last `span` seconds. */
export function timeline(S, { width = 700, span = 90, h = 46 } = {}) {
  const now = NOW(), x = t => Math.max(0, (t - (now - span)) / span * width);
  const segs = TIMELINE.map((s, i) => ({ ...s, t1: TIMELINE[i + 1] ? TIMELINE[i + 1].t0 : now }));
  const bar = segs.map(s => {
    const x0 = x(s.t0), w = Math.max(0, x(s.t1) - x0 - 2);
    if (w <= 0) return '';
    const live = s.state === 'running' && !s.result;
    const fill = s.state === 'running' ? (live ? 'var(--accent)' : 'var(--hi2)') : s.state === 'idle' ? 'var(--hi)' : 'url(#hatch)';
    const label = s.state === 'running' ? `${s.mod}${s.result ? (s.result === 'done' ? ' ✓' : ' ■') : ''}` : s.state;
    return `<rect x="${x0}" y="4" width="${w}" height="24" rx="7" fill="${fill}"/>
      ${w > 36 ? `<text x="${x0 + 8}" y="20" fill="${live ? '#fff' : s.state === 'running' ? 'var(--ink)' : 'var(--muted)'}" font-size="11" font-weight="${s.state === 'running' ? 600 : 400}">${label}</text>` : ''}`;
  }).join('');
  const ticks = [0, 30, 60, 90].filter(v => v <= span).map(v => `<text x="${width - v / span * width}" y="${h - 2}" text-anchor="${v === 0 ? 'end' : v === span ? 'start' : 'middle'}" fill="var(--faint)" font-size="10">${v ? '−' + v + ' s' : 'now'}</text>`).join('');
  return `<svg class="graph" viewBox="0 0 ${width} ${h}" width="100%">
    <defs><pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><rect width="6" height="6" fill="var(--hi)"/><line x1="0" y1="0" x2="0" y2="6" stroke="var(--faint)" stroke-width="1.2"/></pattern></defs>
    ${bar}${ticks}</svg>`;
}

function stateClass(s, m) {
  if (!m) return '';
  if (s === m.state) return 'cur';
  return m.history.some(([h]) => h === s) ? 'done' : '';
}

/* Level 2: the running mod's own FSM as a graph. Main path in a row, skip edges arc under it,
   the shared terminals (stopped / failed) sit at the end, visited states show time spent. */
export function fsmGraph(m, { width = 760, descs = true } = {}) {
  const name = m.name, tr = MODS[name].transitions, states = MODS[name].states, path = mainPath(tr);
  const d = durations(m);
  const n = path.length, nodeH = 32, y = 26, gap = (width - 96) / n;
  const nodeW = Math.min(gap - 22, 116), cx = i => gap * i + gap / 2;
  const id = 'ah' + name + width;
  let edges = '';
  for (const [a, outs] of Object.entries(tr)) for (const b of outs) {
    const i = path.indexOf(a), j = path.indexOf(b);
    if (j === i + 1) edges += `<line x1="${cx(i) + nodeW / 2 + 2}" y1="${y + nodeH / 2}" x2="${cx(j) - nodeW / 2 - 4}" y2="${y + nodeH / 2}" stroke="var(--faint)" stroke-width="1.5" marker-end="url(#${id})"/>`;
    else if (j > i) {
      edges += `<path d="M${cx(i) + 10} ${y - 2} C ${cx(i) + 10} ${y - 26}, ${cx(j) - 10} ${y - 26}, ${cx(j) - 10} ${y - 5}" fill="none" stroke="var(--faint)" stroke-width="1.5" stroke-dasharray="4 4" marker-end="url(#${id})"/>`;
    }
  }
  const nodes = path.map((s, i) => {
    const c = stateClass(s, m);
    const fillC = c === 'cur' ? 'var(--accent)' : c === 'done' ? 'var(--hi2)' : 'none';
    const text = c === 'cur' ? '#fff' : c === 'done' ? 'var(--ink)' : 'var(--faint)';
    const t = d[s] !== undefined && s !== 'idle' ? `<text x="${cx(i)}" y="${y + nodeH + 17}" text-anchor="middle" fill="${c === 'cur' ? 'var(--accent)' : 'var(--muted)'}" font-size="11" font-weight="${c === 'cur' ? 650 : 400}">${secs(d[s])}${c === 'cur' ? ' ●' : ''}</text>` : '';
    return `<rect x="${cx(i) - nodeW / 2}" y="${y}" width="${nodeW}" height="${nodeH}" rx="${nodeH / 2}" fill="${fillC}" stroke="${c ? 'none' : 'var(--faint)'}" stroke-width="1.5"/>
      <text x="${cx(i)}" y="${y + nodeH / 2 + 4}" text-anchor="middle" fill="${text}" font-size="12.5" font-weight="${c === 'cur' ? 650 : 500}">${s}</text>${t}`;
  }).join('');
  const tx = width - 44;
  const terms = `<line x1="${tx - 50}" y1="${y}" x2="${tx - 50}" y2="${y + 52}" stroke="var(--panel-b)" stroke-width="1.5"/>`
    + ['stopped', 'failed'].map((s, i) => `<rect x="${tx - 38}" y="${y + i * 30}" width="76" height="24" rx="12" fill="none" stroke="var(${s === 'failed' ? '--err' : '--faint'})" stroke-width="1.3" stroke-dasharray="3 3"/>
    <text x="${tx}" y="${y + i * 30 + 16}" text-anchor="middle" fill="var(${s === 'failed' ? '--err' : '--muted'})" font-size="11">${s}</text>`).join('')
    + `<text x="${tx}" y="${y + 72}" text-anchor="middle" fill="var(--faint)" font-size="10">from any state</text>`;
  const descH = descs ? 50 : 0;
  const desc = descs ? path.map((s, i) => `<foreignObject x="${cx(i) - gap / 2 + 3}" y="${y + nodeH + 30}" width="${gap - 6}" height="${descH}"><div xmlns="http://www.w3.org/1999/xhtml" class="fdesc ${stateClass(s, m)}">${states[s]}</div></foreignObject>`).join('') : '';
  return `<svg class="graph" viewBox="0 0 ${width} ${y + nodeH + (descs ? 30 + descH : 30)}" width="100%">
    <defs><marker id="${id}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M0 0L10 5L0 10z" fill="var(--faint)"/></marker></defs>
    ${edges}${nodes}${terms}${desc}</svg>`;
}

/* Level 2, vertical: each state with the time spent in it; the current one explains itself */
export function fsmTimeline(m) {
  const path = mainPath(m.transitions), d = durations(m);
  return `<div class="vsteps">${path.map(s => {
    const c = stateClass(s, m);
    return `<div class="vstep ${c}"><i class="n"></i><div><div class="vrow"><b>${s}</b>${d[s] !== undefined && s !== 'idle' ? `<span class="vt">${secs(d[s])}${c === 'cur' ? ' ●' : ''}</span>` : ''}</div><p>${m.states[s]}</p></div></div>`;
  }).join('')}</div>
    <div class="term">from any state → stopped · <span class="f">failed</span></div>`;
}

/* one-line progress for a timed state */
export function now(m) {
  if (!m) return '<div class="now">No mod running</div>';
  const t = NOW() - m.since, dur = m.params.duration_s;
  return `<div class="now"><b>${m.name} · ${m.state}</b> — ${m.states[m.state]}</div>
    ${m.state === 'following' && dur ? `<div style="display:flex;gap:10px;align-items:center;margin-top:8px"><div class="progress" style="flex:1"><i style="width:${(t / dur) * 100}%"></i></div><span class="sub">${t.toFixed(1)} / ${dur} s</span></div>` : ''}`;
}

/* ---------------- log (always on screen) ---------------- */
export function logPanel({ title = true, wrap = false, filters = true } = {}) {
  const n = k => LOG.filter(l => l.level === k).length;
  const ts = t => new Date(t * 1000).toTimeString().slice(0, 8);
  return `<div class="logp ${wrap ? 'wrap' : ''}">
    <div class="loghead">${title ? '<b>Log</b>' : ''}${filters ? `<span class="lf on">all ${LOG.length}</span><span class="lf">warn ${n('warn')}</span><span class="lf">error ${n('error')}</span><span class="lf">teleop</span>` : ''}<span class="lfollow">${dot('live')}follow</span></div>
    <div class="loglines">${LOG.map(l => `<div class="ll ${l.level}"><span class="lt">${ts(l.t)}</span><span class="lv">${l.level}</span><span class="ls">${l.src}</span><span class="lm">${l.msg}</span></div>`).join('')}</div>
  </div>`;
}
