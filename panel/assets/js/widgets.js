/* Panel building blocks as HTML strings. They read the live status pushed by the API and the
   catalog it serves (both kept in `ctx` by admin.js); they hold no state of their own. */

export const ctx = {
  mods: {},       // name -> catalog entry (GET /api/mods)
  robot: null,    // GET /api/robot: major states, arm and camera specs
  log: [],        // records from the stream, oldest first
  coordinates: null,  // GET /api/coordinates (latest localize result)
  topics: {},     // name -> catalog entry (GET /api/topics): doc, depends, output
};

export const ARM_ORDER = ['leader_left', 'follower_left', 'leader_right', 'follower_right'];
const MODE = { position: 'position', external_effort: 'gravity comp', idle: 'braked', velocity: 'velocity', effort: 'effort' };
export const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);

export const armLabel = n => n.replace('leader_', 'Leader ').replace('follower_', 'Follower ').replace('left', 'L').replace('right', 'R');

/* A missing leader is expected and shown as absent; a missing follower, or any arm whose output
   stopped, is an error. */
export const armLive = a => a.connected && !a.lost && a.pos;
export const armKind = a => armLive(a) ? 'ok' : (!a.connected || a.lost) && a.role === 'leader' ? 'off' : 'err';
export const modeText = a => a.lost ? (a.role === 'leader' ? 'offline' : 'no output') : a.connected ? (a.mode || []).map(m => MODE[m] || m).join(' / ') : 'absent';

export const dot = k => `<i class="dot ${k}"></i>`;
const ICON = {
  run: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5.5v13l10.5-6.5z"/></svg>',
  stop: '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6.5" y="6.5" width="11" height="11" rx="2"/></svg>',
  theme: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7"><circle cx="12" cy="12" r="7.5"/><path d="M12 4.5a7.5 7.5 0 0 1 0 15Z" fill="currentColor"/></svg>',
  fold: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="m14.5 6-6 6 6 6"/></svg>',
  search: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><circle cx="11" cy="11" r="6.5"/><path d="m16 16 4 4"/></svg>',
  octagon: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8.2 2.5h7.6l5.7 5.7v7.6l-5.7 5.7H8.2l-5.7-5.7V8.2z"/><path d="M7 12h10" stroke="#fff" stroke-width="2.4" stroke-linecap="round"/></svg>',
  eye: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M2.5 12S6 5.5 12 5.5 21.5 12 21.5 12 18 18.5 12 18.5 2.5 12 2.5 12Z"/><circle cx="12" cy="12" r="2.8"/></svg>',
  restart: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M19.5 12a7.5 7.5 0 1 1-2.2-5.3"/><path d="M19.5 4.5v4h-4"/></svg>',
  fence: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><rect x="2.5" y="5" width="19" height="14" rx="2"/><rect x="6.5" y="8.5" width="11" height="7" rx="1" stroke-dasharray="2.2 2.4"/></svg>',
  plug: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M9 3v5M15 3v5M6 8h12v3a6 6 0 0 1-12 0zM12 17v4"/></svg>',
};
export const icon = k => ICON[k];

/* ---------------- mods ---------------- */
export const active = S => S?.mods?.[S.mods.length - 1] || null;
const secs = t => t < 10 ? t.toFixed(1) + ' s' : t < 90 ? t.toFixed(0) + ' s' : (t / 60).toFixed(1) + ' min';
const categories = () => [...new Set(Object.values(ctx.mods).map(m => m.category))];

/* the longest path from idle through the mod's own transitions, then done. A transition back to an
   earlier state is a retry loop: its state is left out of the line and listed in `loops` */
function mainPath(tr) {
  const back = [], seen = {};
  const visit = s => {
    seen[s] = 'open';
    for (const n of tr[s] || []) {
      if (seen[n] === 'open') back.push([s, n]);
      else if (!seen[n]) visit(n);
    }
    seen[s] = 'done';
  };
  visit('idle');
  const retry = new Set(back.map(([s]) => s)), memo = {};
  const longest = s => memo[s] ??= [s, ...(tr[s] || []).filter(n => !retry.has(n))
    .map(longest).reduce((a, b) => (b.length > a.length ? b : a), [])];
  return { path: [...longest('idle'), 'done'], loops: back };
}

/* seconds spent in each visited state; the current one is still counting (server clock `now`) */
function durations(m, now) {
  const d = {};
  m.history.forEach(([s, t], i) => { d[s] = (m.history[i + 1] ? m.history[i + 1][1] : now) - t; });
  return d;
}

/* the server's check of a mod's REQUIRES against the connected hardware */
const req = (S, name) => S?.requirements?.[name] ?? { met: true, needs: [] };
function modDot(S, name) {
  if (active(S)?.name === name) return 'live';
  return req(S, name).met ? 'ok' : 'err';
}
/* why a mod cannot run, in one line */
export function blockedBy(S, name) {
  const unmet = req(S, name).needs.filter(n => !n.met);
  return unmet.map(n => `needs ${n.text} (${n.items.filter(i => !i.ok).map(i => `${i.label}: no ${i.missing.join(', ')}`).join('; ')})`).join('; ');
}

/* ---------------- mod browser ---------------- */
const matches = (m, { query, cat }) =>
  (cat === 'all' || m.category === cat) &&
  (!query || (m.name + ' ' + m.doc).toLowerCase().includes(query.toLowerCase()));

export function modCats(ui) {
  const all = Object.keys(ctx.mods).length;
  return `<span data-cat="all" class="${ui.cat === 'all' ? 'on' : ''}">all ${all}</span>` +
    categories().map(c => `<span data-cat="${c}" class="${ui.cat === c ? 'on' : ''}">${c}</span>`).join('');
}

export function modList(S, ui) {
  const hits = Object.values(ctx.mods).filter(m => matches(m, ui));
  if (!hits.length) return `<div class="empty">No mod matches “${esc(ui.query)}”.</div>`;
  return categories().map(c => {
    const ms = hits.filter(m => m.category === c);
    return ms.length ? `<div class="mcat">${c}<span>${ms.length}</span></div>${ms.map(m => {
      const run = active(S)?.name === m.name;
      const tail = `<span class="sub">${run ? active(S).state : modDot(S, m.name) === 'ok' ? 'ready' : 'blocked'}</span>`;
      return `<div class="mrow ${m.name === ui.selected ? 'on' : ''}" data-mod="${m.name}" title="${m.name}">
        ${dot(modDot(S, m.name))}<span class="mname">${m.name}</span>${tail}<span class="mdoc">${esc(m.doc)}</span></div>`;
    }).join('')}` : '';
  }).join('');
}

/* ---------------- dock: the selected mod ---------------- */
const UNIT = { duration_s: 's', stage_time: 's', goal_time: 's', sample_hz: 'Hz', target_x: 'm', target_y: 'm',
  speed: 'm/s', lost_s: 's' };

function paramFields(S, m, running, last) {
  const values = running ? active(S).params : { ...Object.fromEntries(m.params.map(p => [p.name, p.default])), ...last };
  const dis = running ? 'disabled' : '';
  return m.params.map(p => {
    const v = values[p.name];
    let input;
    if (p.type === 'tuple' && p.name === 'sides') {
      const can = Object.fromEntries(req(S, m.name).needs.flatMap(n => n.items).filter(i => i.side).map(i => [i.side, i.ok]));
      input = `<span class="toggles">${['left', 'right'].map(s => {
        const ok = can[s] ?? true;
        return `<button class="${!ok ? 'x' : v.includes(s) ? 'on' : ''}" ${dis || !ok ? 'disabled' : ''} data-param="${p.name}" data-val="${s}">${s}</button>`;
      }).join('')}</span>`;
    } else if (p.type === 'bool') {
      input = `<span class="toggles"><button class="${v ? 'on' : ''}" ${dis} data-param="${p.name}" data-bool>${v ? 'on' : 'off'}</button></span>`;
    } else {
      input = `<label class="in"><input value="${esc(v)}" ${p.type === 'str' ? '' : 'inputmode="decimal"'} ${dis} data-param="${p.name}" data-type="${p.type}"><u>${UNIT[p.name] || ''}</u></label>`;
    }
    const help = p.help ? `<span class="qhelp" tabindex="0" aria-label="What is ${p.name}?" data-help="${esc(p.help)}" data-figure="${p.figure ?? ''}">?</span>` : '';
    return `<div class="field"><span>${p.name}${help}</span>${input}</div>`;
  }).join('');
}

/* read the dock's inputs back into a params object for POST /api/mods/{name}/run */
export function readParams(root, m) {
  const out = {};
  for (const p of m.params) {
    if (p.type === 'tuple') {
      out[p.name] = [...root.querySelectorAll(`button[data-param="${p.name}"].on`)].map(b => b.dataset.val);
    } else if (p.type === 'bool') {
      out[p.name] = root.querySelector(`[data-param="${p.name}"]`).classList.contains('on');
    } else if (p.type === 'str') {
      out[p.name] = root.querySelector(`[data-param="${p.name}"]`).value.trim();
    } else {
      out[p.name] = Number(root.querySelector(`[data-param="${p.name}"]`).value);
    }
  }
  return out;
}

/* `last`: the params of this mod's last run, shown instead of the defaults */
export function dock(S, name, now, last = null) {
  const m = ctx.mods[name];
  if (!m) return '<div class="now">Waiting for the robot API…</div>';
  const running = active(S)?.name === name;
  const r = req(S, name);
  /* an option that is down while its Need is still met is only skipped; otherwise it blocks */
  const skipped = r.needs.filter(n => n.met).flatMap(n => n.items.filter(i => !i.ok).map(i => i.label));
  const fsm = running ? active(S) : { ...m, state: null, history: [] };
  const needs = r.needs.map(n => `<div class="need"><span class="ntext">${dot(n.met ? 'ok' : 'err')}needs ${esc(n.text)}</span>
      ${n.items.map(i => `<span>${dot(i.ok ? 'ok' : n.met ? 'off' : 'err')}${esc(i.label)}</span>`).join('')}</div>`).join('');
  return `<div class="phead"><div><b>${m.name}</b><span class="sub">${m.category}</span></div><p>${esc(m.doc)}</p>${needs}</div>
    <div class="params">${paramFields(S, m, running, last)}${skipped.length && !running ? `<div class="hint">● ${skipped.join('; ')}: skipped</div>` : ''}</div>
    ${name === 'localize' ? coordinatesCard() : ''}
    <div><h4>State machine <span class="sub">${running ? 'running' : 'not running'}</span></h4>${fsmTimeline(fsm, now)}</div>`;
}

/* the latest registration, with the numbers that say how far to trust it */
function coordinatesCard() {
  const c = ctx.coordinates;
  if (!c) return '<div class="result"><h4>Latest result</h4><div class="sub">none yet</div></div>';
  const r = c.report, g = r.geometry;
  const row = (k, v) => `<div class="kv"><span>${k}</span><b>${v}</b></div>`;
  return `<div class="result"><h4>Latest result <span class="sub">${esc(c.created)}</span></h4>
    ${row('bases apart', `${g.base_distance_mm} mm`)}
    ${row('left yaw / lateral', `${g.left_yaw_vs_facing_deg}° / ${g.left_lateral_in_right_mm} mm`)}
    ${row('base heights L / R', `${g.base_height_mm.left} / ${g.base_height_mm.right} mm`)}
    ${row('constraint', r.constraint?.startsWith('parallel') ? 'parallel rails' : 'free fit')}
    ${r.parallel_fit ? row('parallel fit rms', `${r.parallel_fit.rms_mm} mm`) : ''}
    ${r.free_fit_left_yaw_vs_facing_deg !== undefined ? row('free-fit yaw', `${r.free_fit_left_yaw_vs_facing_deg}°`) : ''}
    ${g.apparent_tilt_deg ? row('apparent tilt L / R (sag)', `${g.apparent_tilt_deg.left} / ${g.apparent_tilt_deg.right}°`) : ''}
    ${row('fit rms (inliers)', `${r.fit_residual_mm.rms} mm (${r.inliers}/${r.cross_matches})`)}
    ${r.split_half ? row('split-half', `${r.split_half.translation_mm} mm / ${r.split_half.rotation_deg}°`) : ''}
    ${row('view consistency L / R', `${r.view_consistency.left.median_mm} / ${r.view_consistency.right.median_mm} mm`)}
    ${row('table agreement', `${r.table_agreement.normal_angle_deg}° / ${r.table_agreement.height_gap_at_origin_mm} mm`)}
    ${r.vs_reference ? row('vs cube registration', `${r.vs_reference.base_distance_mm} mm / ${r.vs_reference.left_yaw_deg}° / ${r.vs_reference.left_lateral_mm} mm`) : ''}
    <a class="sub" href="/api/coordinates/matches.jpg" target="_blank">SIFT matches ↗</a></div>`;
}

export function actions(S, name, error) {
  const why = !S ? 'robot API offline' : S.state === 'estop' ? 'emergency stop is active'
    : active(S) ? 'a mod is running' : blockedBy(S, name);
  return `${error ? `<div class="hint err">${esc(error)}</div>` : !why || !S || active(S) || S.state === 'estop' ? '' : `<div class="hint err">${esc(why)}</div>`}<div class="actions">
    <button class="btn run big" id="run" ${why ? 'disabled' : ''} title="${esc(why || `Run ${name}`)}">${icon('run')}Run</button>
    <button class="btn stop big" id="stop" ${active(S) ? '' : 'disabled'} title="Stop the mod gracefully (its arms hold)">${icon('stop')}Stop</button></div>`;
}

/* ================= FSM: two levels ================= */
/* Level 1: the robot's major states, then the running mod (and its parents) and its state. */
export function majorStrip(S, link) {
  if (!link) return '<div class="major"><span class="mst down">backend offline</span><span class="sub">reconnecting…</span></div>';
  if (!S) return '<div class="major"><span class="sub">connecting…</span></div>';   // linked, no state frame yet
  const m = active(S);
  const crumbs = S.mods.map(x => `<span class="crumb">${x.name}</span>`).join('<span class="sep">›</span>');
  const states = ctx.robot?.states ?? {};
  return `<div class="major">${Object.keys(states).map(s => `<span class="mst ${s === S.state ? 'cur' : ''}" title="${esc(states[s])}">${s}</span>`).join('<i class="marr"></i>')}
    ${m ? `<span class="sep">›</span>${crumbs}<span class="sep">›</span><span class="crumb cur">${m.state}</span>` : ''}</div>`;
}

function stateClass(s, m) {
  if (s === m.state) return 'cur';
  return m.history.some(([h]) => h === s) ? 'done' : '';
}

/* Level 2: each state of the mod's own machine, time spent in it, and what the arms do there. */
function fsmTimeline(m, now) {
  const { path, loops } = mainPath(m.transitions), d = durations(m, now);
  const terminal = ['stopped', 'failed'].includes(m.state) ? m.state : null;
  return `<div class="vsteps">${path.map(s => {
    const c = stateClass(s, m);
    return `<div class="vstep ${c}"><i class="n"></i><div><div class="vrow"><b>${s}</b>${d[s] !== undefined && s !== 'idle' ? `<span class="vt">${secs(d[s])}${c === 'cur' ? ' ●' : ''}</span>` : ''}</div><p>${esc(m.states[s])}</p></div></div>`;
  }).join('')}</div>
    ${loops.map(([s, to]) => `<div class="term">a try that fails → <span class="${m.state === s ? 'hit' : ''}">${s}</span> → ${to} again</div>`).join('')}
    <div class="term">from any state → <span class="${terminal === 'stopped' ? 'hit' : ''}">stopped</span> · <span class="f">failed</span></div>`;
}

/* the running mod in one line, with progress for a timed state */
export function now(S, t) {
  const m = active(S);
  if (!m) return '<div class="now">No mod running</div>';
  const el = t - m.since, dur = m.params.duration_s;
  return `<div class="now"><b>${m.name} · ${m.state}</b> — ${esc(m.states[m.state])}</div>
    ${m.state === 'following' && dur ? `<div class="prow"><div class="progress"><i style="width:${Math.min(100, (el / dur) * 100)}%"></i></div><span class="sub">${el.toFixed(1)} / ${dur} s</span></div>` : ''}`;
}

/* ---------------- cameras: click a tile to switch RGB / depth ---------------- */
export function camTile(key, c, kind) {
  return `<figure class="cam ${c.ok ? '' : 'nofeed'}" data-cam="${key}" title="${c.ok ? `Click for ${kind === 'color' ? 'depth' : 'RGB'}` : esc(c.error || 'no frames')}">
    ${c.ok ? `<img data-feed="${key}" alt="${c.label} ${kind === 'color' ? 'RGB' : 'depth'}">` : `<span class="why">${esc(c.error || 'no frames')}</span>`}
    <figcaption>${dot(c.ok ? 'ok' : 'err')}${c.label}<span class="s" data-fps>${c.ok ? c.fps.toFixed(0) + ' fps' : 'offline'}</span></figcaption>
    ${c.ok ? `<span class="cmode">${kind === 'color' ? 'RGB' : 'Depth'}</span>` : ''}
  </figure>`;
}

/* ---------------- log (always on screen) ---------------- */
export const logHead = (S, filter) => {
  const n = k => ctx.log.filter(l => l.level === k).length;
  const m = active(S);
  const chip = (f, label) => `<span class="lf ${filter === f ? 'on' : ''}" data-logf="${f}">${label}</span>`;
  return `<b>Log</b>${chip('all', `all ${ctx.log.length}`)}${chip('warning', `warn ${n('warning')}`)}${chip('error', `error ${n('error') + n('critical')}`)}${m ? chip(m.name, m.name) : ''}<span class="lfollow">${dot('live')}follow</span>`;
};
export const logKeep = filter => l => filter === 'all' || l.level === filter || l.src === filter
  || (filter === 'error' && l.level === 'critical');
export const logLine = l => `<div class="ll ${l.level}" data-seq="${l.seq}"><span class="lt">${new Date(l.t * 1000).toTimeString().slice(0, 8)}</span><span class="lv">${{ warning: 'warn', critical: 'crit' }[l.level] ?? l.level}</span><span class="ls">${esc(l.src)}</span><span class="lm">${esc(l.msg)}</span></div>`;

/* the whole record, for the hover card: date and time to the ms, level, full logger, message, traceback */
export const logDetail = l => {
  const d = new Date(l.t * 1000), ms = String(d.getMilliseconds()).padStart(3, '0');
  return `<div class="ldh"><span>${d.toLocaleDateString()} ${d.toTimeString().slice(0, 8)}.${ms}</span><span class="lv ${l.level}">${l.level}</span><span>${esc(l.logger ?? l.src)}</span></div>`
    + `<pre class="ldm">${esc(l.msg)}</pre>${l.exc ? `<pre class="ldx">${esc(l.exc)}</pre>` : ''}`;
};

/* ---------------- topics: perception processes the backend runs ---------------- */
export const hzText = hz => `${hz.toFixed(hz < 10 ? 1 : 0)} Hz`;
const TOPIC_DOT = { running: 'ok', waiting: 'warn', starting: 'live', crashed: 'err', stopped: 'off' };
export const topicLive = t => t && ['running', 'waiting'].includes(t.state);

/* one row per topic: state, eye (show its results in the 3-D view), start/stop, restart; the
   open row adds what it computes, why it waits or crashed, and how the panel subscribes */
export function topicList(S, ui) {
  const st = S?.topics ?? {};
  return Object.values(ctx.topics).map(c => {
    const t = st[c.name] ?? { state: 'stopped' }, live = topicLive(t), u = ui.topics[c.name];
    const busy = t.state === 'starting';
    const unmet = t.requirements && !t.requirements.met
      ? t.requirements.needs.filter(n => !n.met).map(n => n.text).join('; ') : '';
    const sub = t.state === 'running' ? hzText(t.hz) : t.state === 'waiting' ? 'waiting' : t.state;
    const eye = `<button class="tb ${u.eye && live ? 'on' : ''}" data-teye="${c.name}" ${live ? '' : 'disabled'} title="${live ? (u.eye ? 'Hide' : 'Show') + ' in the 3-D view' : 'Start the topic to show it'}">${ICON.eye}</button>`;
    const go = live || busy
      ? `<button class="tb" data-tact="stop" data-topic-name="${c.name}" title="Stop ${c.name}">${ICON.stop}</button>`
      : `<button class="tb" data-tact="start" data-topic-name="${c.name}" ${unmet ? 'disabled' : ''} title="${esc(unmet ? 'Needs ' + unmet : 'Start ' + c.name)}">${ICON.run}</button>`;
    const again = `<button class="tb" data-tact="restart" data-topic-name="${c.name}" ${live ? '' : 'disabled'} title="Restart ${c.name}">${ICON.restart}</button>`;
    const open = ui.topicOpen === c.name;
    return `<div class="trow ${open ? 'open' : ''}">
      <div class="thead" data-topic="${c.name}">${dot(TOPIC_DOT[t.state] ?? 'off')}<span class="tname">${c.name}</span><span class="sub" data-thz>${esc(sub)}</span>${eye}${go}${again}</div>
      ${open ? topicDetail(c, t, u) : ''}</div>`;
  }).join('');
}

function topicDetail(c, t, u) {
  const why = t.state === 'waiting' ? `<div class="hint">${esc(t.waiting)}</div>`
    : t.state === 'crashed' ? `<div class="hint err">exit ${t.code}: ${esc((t.tail ?? []).slice(-2).join(' | '))}</div>` : '';
  const needs = c.depends.length ? `<div class="sub">needs ${c.depends.join(', ')}</div>` : '';
  const mode = m => `<button class="${u.mode === m ? 'on' : ''}" data-tmode="${m}" data-topic-name="${c.name}">${m}</button>`;
  return `<div class="tbody"><p>${esc(c.doc)}</p>${needs}${why}
    <div class="field"><span>subscribe</span><span class="toggles">${mode('stream')}${mode('poll')}</span></div>
    ${u.mode === 'poll' ? `<div class="field"><span>every</span><label class="in"><input value="${u.period}" inputmode="numeric" data-tperiod="${c.name}"><u>ms</u></label></div>` : ''}
    ${t.url ? `<a class="sub" href="${t.url}/latest" target="_blank">${t.url.replace('http://', '')}/latest ↗</a>` : ''}</div>`;
}
