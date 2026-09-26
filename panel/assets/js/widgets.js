/* Panel building blocks as HTML strings. They read the status shape the backend will serve
   (Aloha.status() + cameras + log) and the panel's own UI state; they hold no state. */
import { MODS, PLANNED, CATEGORIES, ROBOT_STATES, LOG } from './mock.js';

export const ARM_ORDER = ['leader_left', 'follower_left', 'leader_right', 'follower_right'];
export const CAM_ORDER = ['high', 'low', 'wrist_a', 'wrist_b'];
const MODE = { position: 'position', external_effort: 'gravity comp', idle: 'braked', velocity: 'velocity', effort: 'effort' };
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);

export const armLabel = n => n.replace('leader_', 'Leader ').replace('follower_', 'Follower ').replace('left', 'L').replace('right', 'R');

/* A missing leader is expected and shown as absent; a missing follower is an error. */
export const armKind = a => a.connected ? 'ok' : a.role === 'leader' ? 'off' : 'err';
export const modeText = a => a.connected ? a.mode.map(m => MODE[m] || m).join(' / ') : 'absent';

export const dot = k => `<i class="dot ${k}"></i>`;
const ICON = {
  run: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5.5v13l10.5-6.5z"/></svg>',
  stop: '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6.5" y="6.5" width="11" height="11" rx="2"/></svg>',
  theme: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7"><circle cx="12" cy="12" r="7.5"/><path d="M12 4.5a7.5 7.5 0 0 1 0 15Z" fill="currentColor"/></svg>',
  search: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><circle cx="11" cy="11" r="6.5"/><path d="m16 16 4 4"/></svg>',
};
export const icon = k => ICON[k];

/* ---------------- mods ---------------- */
export const active = S => S.mods[S.mods.length - 1] || null;
const NOW = () => Date.now() / 1000;
const secs = t => t < 10 ? t.toFixed(1) + ' s' : t < 90 ? t.toFixed(0) + ' s' : (t / 60).toFixed(1) + ' min';

/* every mod the browser lists: real ones from Python, then planned examples */
export const ALL_MODS = [
  ...Object.entries(MODS).map(([name, m]) => ({ name, category: m.category, doc: m.doc })),
  ...PLANNED,
];

/* the longest path from idle through the mod's own transitions, then done */
function mainPath(tr) {
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

function modDot(S, m) {
  if (m.planned) return 'off';
  if (active(S) && active(S).name === m.name) return 'live';
  return availability(m.name, S).some(([k]) => k === 'ok') ? 'ok' : 'err';
}

/* ---------------- mod browser ---------------- */
const matches = (m, { query, cat }) =>
  (cat === 'all' || m.category === cat) &&
  (!query || (m.name + ' ' + m.doc).toLowerCase().includes(query.toLowerCase()));

export function modCats(ui) {
  const n = c => ALL_MODS.filter(m => m.category === c).length;
  return `<span data-cat="all" class="${ui.cat === 'all' ? 'on' : ''}">all ${ALL_MODS.length}</span>` +
    CATEGORIES.filter(n).map(c => `<span data-cat="${c}" class="${ui.cat === c ? 'on' : ''}">${c}</span>`).join('');
}

export function modList(S, ui) {
  const hits = ALL_MODS.filter(m => matches(m, ui));
  if (!hits.length) return `<div class="empty">No mod matches “${esc(ui.query)}”.</div>`;
  return CATEGORIES.map(c => {
    const ms = hits.filter(m => m.category === c);
    return ms.length ? `<div class="mcat">${c}<span>${ms.length}</span></div>${ms.map(m => {
      const run = active(S) && active(S).name === m.name;
      const tail = m.planned ? '<span class="tagp">planned</span>'
        : `<span class="sub">${run ? active(S).state : availability(m.name, S).some(([k]) => k === 'ok') ? 'ready' : 'blocked'}</span>`;
      return `<div class="mrow ${m.name === ui.selected ? 'on' : ''} ${m.planned ? 'planned' : ''}" data-mod="${m.name}">
        ${dot(modDot(S, m))}<span class="mname">${m.name}</span>${tail}<span class="mdoc">${m.doc}</span></div>`;
    }).join('')}` : '';
  }).join('');
}

export function modCount() {
  const ready = ALL_MODS.filter(m => !m.planned).length;
  return `${ready} available · ${ALL_MODS.length - ready} planned (examples)`;
}

/* ---------------- dock: the selected mod ---------------- */
function paramFields(S, name, locked) {
  const run = active(S) && active(S).name === name;
  const P = run ? active(S).params : MODS[name].params;
  const unit = { duration_s: 's', align_time: 's', goal_time: 's', sample_hz: 'Hz' };
  const dis = locked ? 'disabled' : '';
  return Object.entries(P).map(([k, v]) => {
    let input;
    if (k === 'sides') {
      input = `<span class="toggles">${['left', 'right'].map(s => {
        const ok = name === 'teleop' ? S.arms['leader_' + s].connected && S.arms['follower_' + s].connected : S.arms['follower_' + s].connected;
        return `<button class="${!ok ? 'x' : v.includes(s) ? 'on' : ''}" ${dis || !ok ? 'disabled' : ''} data-param="${k}" data-val="${s}">${s}</button>`;
      }).join('')}</span>`;
    } else if (typeof v === 'boolean') {
      input = `<span class="toggles"><button class="${v ? 'on' : ''}" ${dis} data-param="${k}">${v ? 'on' : 'off'}</button></span>`;
    } else {
      input = `<label class="in"><input value="${v}" inputmode="decimal" ${dis} data-param="${k}"><u>${unit[k] || ''}</u></label>`;
    }
    return `<div class="field"><span>${k}</span>${input}</div>`;
  }).join('');
}

export function dock(S, name) {
  const m = ALL_MODS.find(x => x.name === name);
  const run = active(S) && active(S).name === name;
  const head = `<div class="phead"><div><b>${m.name}</b><span class="sub">${m.category}</span></div><p>${m.doc}</p>
    ${m.planned ? '' : `<div class="need">${availability(name, S).map(([k, t]) => `<span>${dot(k)}${t}</span>`).join('')}</div>`}</div>`;
  if (m.planned) return `${head}<div class="planned-note">Planned: not implemented yet. Listed to show how the browser scales.</div>`;
  const skip = name === 'teleop' && !S.arms.leader_right.connected
    ? '<div class="hint">● leader R absent: the right side will be skipped</div>' : '';
  const fsm = run ? active(S) : { name, state: null, history: [], ...MODS[name] };
  return `${head}
    <div class="params">${paramFields(S, name, run)}${skip}</div>
    <div><h4>State machine <span class="sub">${run ? 'running' : 'not running'}</span></h4>${fsmTimeline(fsm)}</div>`;
}

export function actions(S, name) {
  const m = ALL_MODS.find(x => x.name === name);
  const busy = !!active(S);
  return `<div class="actions">
    <button class="btn run big" ${busy || m.planned ? 'disabled' : ''} title="${busy ? 'a mod is running' : ''}">${icon('run')}Run ${m.planned ? '' : name}</button>
    <button class="btn stop big" ${busy ? '' : 'disabled'}>${icon('stop')}Stop</button></div>`;
}

/* ================= FSM: two levels ================= */
/* Level 1: the robot's major states, then the running mod (and its parents) and its state. */
export function majorStrip(S) {
  const m = active(S);
  const crumbs = S.mods.map(x => `<span class="crumb">${x.name}</span>`).join('<span class="sep">›</span>');
  return `<div class="major">${['offline', 'idle', 'running'].map(s => `<span class="mst ${s === S.state ? 'cur' : ''}" title="${ROBOT_STATES[s]}">${s}</span>`).join('<i class="marr"></i>')}
    ${m ? `<span class="sep">›</span>${crumbs}<span class="sep">›</span><span class="crumb cur">${m.state}</span>` : ''}</div>`;
}

function stateClass(s, m) {
  if (s === m.state) return 'cur';
  return m.history.some(([h]) => h === s) ? 'done' : '';
}

/* Level 2: each state of the mod's own machine, time spent in it, and what the arms do there. */
function fsmTimeline(m) {
  const path = mainPath(m.transitions), d = durations(m);
  return `<div class="vsteps">${path.map(s => {
    const c = stateClass(s, m);
    return `<div class="vstep ${c}"><i class="n"></i><div><div class="vrow"><b>${s}</b>${d[s] !== undefined && s !== 'idle' ? `<span class="vt">${secs(d[s])}${c === 'cur' ? ' ●' : ''}</span>` : ''}</div><p>${m.states[s]}</p></div></div>`;
  }).join('')}</div>
    <div class="term">from any state → stopped · <span class="f">failed</span></div>`;
}

/* the running mod in one line, with progress for a timed state */
export function now(S) {
  const m = active(S);
  if (!m) return '<div class="now">No mod running</div>';
  const t = NOW() - m.since, dur = m.params.duration_s;
  return `<div class="now"><b>${m.name} · ${m.state}</b> — ${m.states[m.state]}</div>
    ${m.state === 'following' && dur ? `<div class="prow"><div class="progress"><i style="width:${Math.min(100, (t / dur) * 100)}%"></i></div><span class="sub">${t.toFixed(1)} / ${dur} s</span></div>` : ''}`;
}

/* ---------------- cameras: click a tile to switch RGB / depth ---------------- */
export function camTile(S, key, snapDir, mode = 'color') {
  const c = S.cameras[key];
  return `<figure class="cam" data-cam="${key}" title="Click for ${mode === 'color' ? 'depth' : 'RGB'}">
    <img src="${snapDir}${c.serial}_${mode}.png" alt="${c.label} ${mode === 'color' ? 'RGB' : 'depth'}" onerror="this.parentNode.classList.add('nofeed')">
    <figcaption>${dot(c.ok ? 'ok' : 'err')}${c.label}<span class="s">${c.fps.toFixed(0)} fps</span></figcaption>
    <span class="cmode">${mode === 'color' ? 'RGB' : 'Depth'}</span>
  </figure>`;
}

/* ---------------- log (always on screen) ---------------- */
export function logPanel(S, filter = 'all') {
  const m = active(S);
  const n = k => LOG.filter(l => l.level === k).length;
  const keep = l => filter === 'all' || l.level === filter || l.src === filter;
  const ts = t => new Date(t * 1000).toTimeString().slice(0, 8);
  const chip = (f, label) => `<span class="lf ${filter === f ? 'on' : ''}" data-logf="${f}">${label}</span>`;
  return `<div class="loghead"><b>Log</b>${chip('all', `all ${LOG.length}`)}${chip('warn', `warn ${n('warn')}`)}${chip('error', `error ${n('error')}`)}${m ? chip(m.name, m.name) : ''}<span class="lfollow">${dot('live')}follow</span></div>
    <div class="loglines">${LOG.filter(keep).map(l => `<div class="ll ${l.level}"><span class="lt">${ts(l.t)}</span><span class="lv">${l.level}</span><span class="ls">${l.src}</span><span class="lm">${esc(l.msg)}</span></div>`).join('')}</div>`;
}
