/* Six panel layouts over the same widgets, the same status and the same rig. Each is designed
   on a 1440×900 frame; the board scales it down, ?l=N opens it at window size.
   Every layout carries: a mod browser that scales past a handful of mods, the two-level FSM
   (robot major state, then the running mod's machine), and an always-visible log. */
import * as W from './widgets.js';

const themeBtn = `<button class="btn icon" data-theme-btn title="Light / dark">${W.icon('theme')}</button>`;
const brand = S => `<b class="brand-name">myaloha</b>${W.majorStrip(S)}`;
const scene = (style = '') => `<div class="scene" data-scene style="${style}"></div>`;
const col = (style, inner) => `<div style="display:flex;flex-direction:column;gap:14px;min-height:0;${style}">${inner}</div>`;

export const LAYOUTS = [
  {
    num: '01', name: 'Stage', family: 'rig first',
    note: 'IROS26 layout language: the rig fills the screen and frosted panels float over it. Mod browser and watchdog on the left, the selected mod with its FSM timeline on the right, cameras above and the log below the rig.',
    insets: { l: 316, r: 380, t: 210, b: 222 },
    html: (S, snap) => `
      ${scene('position:absolute;inset:0')}
      <div class="panel chip float" style="top:18px;left:18px">${brand(S)}</div>
      <div class="panel chip float" style="top:18px;right:18px;padding:0 8px">${themeBtn}<button class="btn stop">${W.icon('stop')}Stop</button></div>
      <div class="float" style="top:88px;left:18px;bottom:18px;width:290px;display:flex;flex-direction:column;gap:10px">
        <div class="panel" style="flex:1;display:flex;min-height:0">${W.modBrowser(S)}</div>
        <div class="panel">${W.watchdog(S)}</div>
      </div>
      <div class="panel float" style="top:88px;right:18px;bottom:18px;width:350px;display:flex;flex-direction:column;gap:14px">
        ${W.params(S, 'teleop', { head: true })}
        <div><h4>State</h4>${W.fsmTimeline(W.active(S))}</div>
        <div style="margin-top:auto">${W.now(W.active(S))}<div style="margin-top:12px">${W.actions(S, { big: true })}</div></div>
      </div>
      <div class="float" style="top:88px;left:326px;right:386px;height:112px;display:grid;grid-template-columns:repeat(4,1fr);gap:10px">
        ${W.CAM_ORDER.map(k => W.camTile(S, k, snap, { fill: true })).join('')}
      </div>
      <div class="panel float" style="left:326px;right:386px;bottom:18px;height:196px">${W.logPanel()}</div>`,
  },
  {
    num: '02', name: 'Rails', family: 'rig first',
    note: 'Three columns. Left: the mod library, searchable and grouped by category. Middle: the robot\'s state history, the rig, the cameras and the running mod\'s FSM graph. Right: the selected mod\'s params, the watchdog and the log.',
    html: (S, snap) => `
      <div style="display:grid;grid-template-columns:290px 1fr 340px;gap:14px;padding:16px;height:100%">
        <div class="panel solid" style="display:flex;flex-direction:column;gap:12px;min-height:0">
          <div style="display:flex;align-items:center;height:36px"><b class="brand-name">myaloha</b><span style="margin-left:auto">${themeBtn}</span></div>
          ${W.modBrowser(S)}
        </div>
        ${col('', `
          <div class="panel solid" style="display:grid;gap:8px">${W.majorStrip(S)}${W.timeline(S, { width: 740 })}</div>
          ${scene('border-radius:16px;flex:1')}
          <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:10px;height:112px">${W.CAM_ORDER.map(k => W.camTile(S, k, snap, { fill: true })).join('')}</div>
          <div class="panel solid">${W.fsmGraph(W.active(S), { width: 740, descs: false })}${W.now(W.active(S))}</div>`)}
        ${col('', `
          <div class="panel solid">${W.params(S, 'teleop', { head: true })}<div style="margin-top:12px">${W.actions(S)}</div></div>
          <div class="panel solid">${W.watchdog(S)}</div>
          <div class="panel solid" style="flex:1;min-height:0">${W.logPanel({ wrap: true, filters: false })}</div>`)}
      </div>`,
  },
  {
    num: '03', name: 'Mirror', family: 'rig first',
    note: 'Built around the bimanual pairs: each side column shows its leader, follower and wrist camera, and an absent leader is a dashed card on its own side. The mod picker is a ⌘K palette in the toolbar (shown open), so the list can grow without taking space. FSM graph and log run along the bottom.',
    html: (S, snap) => `
      <div style="position:relative;display:grid;grid-template-columns:300px 1fr 300px;grid-template-rows:var(--bar-h) 1fr 218px;gap:14px;padding:16px;height:100%">
        <div style="grid-column:1/-1;display:flex;gap:14px;align-items:center">
          <div class="panel chip solid">${brand(S)}</div>
          <div class="panel chip solid" style="margin-left:auto;padding:0 8px">${W.modCombo(S)}${W.actions(S)}</div>
          <div class="panel chip solid" style="padding:0 8px">${themeBtn}</div>
        </div>
        ${col('', `<div class="panel solid" style="display:flex;flex-direction:column;gap:10px;flex:1">
          <h4>Left side <span class="sub">pair ready</span></h4>
          ${W.armCard(S.arms.leader_left)}${W.armCard(S.arms.follower_left)}${W.camTile(S, 'wrist_a', snap)}</div>`)}
        <div style="display:grid;grid-template-rows:150px 1fr;gap:14px;min-height:0">
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;min-height:0">${W.camTile(S, 'high', snap, { fill: true })}${W.camTile(S, 'low', snap, { fill: true })}</div>
          ${scene('border-radius:16px')}
        </div>
        ${col('', `<div class="panel solid" style="display:flex;flex-direction:column;gap:10px;flex:1">
          <h4>Right side <span class="sub">skipped by teleop</span></h4>
          ${W.armCard(S.arms.leader_right)}${W.armCard(S.arms.follower_right)}${W.camTile(S, 'wrist_b', snap)}</div>`)}
        <div style="grid-column:1/-1;display:grid;grid-template-columns:1.35fr 1fr;gap:14px;min-height:0">
          <div class="panel solid" style="display:grid;gap:10px;align-content:start">${W.params(S, 'teleop', { inline: true })}${W.fsmGraph(W.active(S), { width: 780, descs: false })}${W.now(W.active(S))}</div>
          <div class="panel solid" style="min-height:0">${W.logPanel()}</div>
        </div>
        <div style="position:absolute;z-index:6;top:74px;right:104px;width:420px">${W.palette(S)}</div>
      </div>`,
  },
  {
    num: '04', name: 'Cockpit', family: 'something else first',
    note: 'Cameras lead: a 2×2 grid with depth insets takes half the screen. The rig on the right carries arm status as tags, with the running mod\'s FSM graph under it. Along the bottom: a wide log, the selected mod\'s params, and hardware LEDs in the status bar.',
    html: (S, snap) => `
      <div style="display:grid;grid-template-columns:1fr 1fr;grid-template-rows:var(--bar-h) 1fr 196px 32px;gap:14px;padding:16px 16px 0;height:100%">
        <div style="grid-column:1/-1;display:flex;gap:14px;align-items:center">
          <div class="panel chip solid">${brand(S)}</div>
          <div class="panel chip solid" style="margin-left:auto;padding:0 8px">${W.modCombo(S)}${W.actions(S)}</div>
          <div class="panel chip solid" style="padding:0 8px">${themeBtn}</div>
        </div>
        <div style="display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:10px;min-height:0">
          ${W.CAM_ORDER.map(k => W.camTile(S, k, snap, { fill: true, depth: true })).join('')}
        </div>
        ${col('', `${scene('border-radius:16px;flex:1')}
          <div class="panel solid">${W.fsmGraph(W.active(S), { width: 680, descs: false })}${W.now(W.active(S))}</div>`)}
        <div class="panel solid" style="min-height:0">${W.logPanel()}</div>
        <div class="panel solid" style="display:grid;grid-template-columns:1fr 1.2fr;gap:22px;align-items:start">${W.params(S, 'teleop', { head: true })}</div>
        <div class="statusbar" style="grid-column:1/-1;margin:0 -16px">${W.leds(S)}<span style="margin-left:auto">loop 30.0 Hz</span><span>backend: mock</span></div>
      </div>`,
  },
  {
    num: '05', name: 'State machine', family: 'something else first',
    note: 'FSM first. The robot band shows the major states and their history (every mod run as a segment). Below it, the running mod\'s full machine with each state\'s meaning and time spent. The mod library runs down the left; rig, cameras and log fill the rest.',
    html: (S, snap) => `
      <div style="display:grid;grid-template-columns:300px 1fr;grid-template-rows:var(--bar-h) auto auto 1fr 150px 32px;gap:14px;padding:16px 16px 0;height:100%">
        <div style="grid-column:1/-1;display:flex;gap:14px;align-items:center">
          <div class="panel chip solid"><b class="brand-name">myaloha</b></div>
          <div class="panel chip solid" style="margin-left:auto;padding:0 8px">${themeBtn}${W.actions(S)}</div>
        </div>
        ${col('grid-row:2/6', `<div class="panel solid" style="flex:1;display:flex;min-height:0">${W.modBrowser(S, { docs: false })}</div>
          <div class="panel solid">${W.params(S, 'teleop', { head: true })}</div>`)}
        <div class="panel solid" style="display:grid;grid-template-columns:auto 1fr;gap:22px;align-items:center">
          <div><h4>Robot</h4>${W.majorStrip(S)}</div>${W.timeline(S, { width: 700 })}</div>
        <div class="panel solid"><h4>${W.active(S).name} <span class="sub">${W.active(S).category} · started ${(Date.now() / 1000 - W.active(S).history[0][1]).toFixed(1)} s ago</span></h4>${W.fsmGraph(W.active(S), { width: 1060 })}</div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:14px;min-height:0">
          ${scene('border-radius:16px')}
          <div style="display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:10px;min-height:0">${W.CAM_ORDER.map(k => W.camTile(S, k, snap, { fill: true })).join('')}</div>
        </div>
        <div class="panel solid" style="min-height:0">${W.logPanel()}</div>
        <div class="statusbar" style="grid-column:1/-1;margin:0 -16px">${W.leds(S)}<span style="margin-left:auto">loop 30.0 Hz</span><span>backend: mock</span></div>
      </div>`,
  },
  {
    num: '06', name: 'Console', family: 'something else first',
    note: 'An IDE-style workbench, edge to edge: a mod explorer tree grouped by category, the rig in the middle, an inspector with params, the FSM timeline and the watchdog, and a dock whose main tab is the log, with cameras beside it.',
    insets: { t: 90 },
    html: (S, snap) => `
      <div style="display:grid;grid-template-columns:250px 1fr 330px;grid-template-rows:56px 1fr 250px 30px;height:100%">
        <div style="grid-column:1/-1;display:flex;gap:16px;align-items:center;padding:0 14px;background:var(--panel-solid);border-bottom:1px solid var(--panel-b)">
          <b class="brand-name">myaloha</b>${W.majorStrip(S)}
          <span style="margin-left:auto;display:flex;gap:8px">${themeBtn}${W.actions(S)}</span>
        </div>
        <div style="grid-column:1;grid-row:2/4;background:var(--panel-solid);border-right:1px solid var(--panel-b);padding:12px;display:flex;flex-direction:column;gap:10px;min-height:0">
          ${W.search()}<h4 style="margin:4px 0 0">Mods <span class="sub">13</span></h4>${W.modTree(S)}
        </div>
        <div style="grid-column:2;grid-row:2;position:relative;min-height:0">
          ${scene('position:absolute;inset:0')}
          <div class="panel float" style="top:14px;left:14px;right:14px">${W.timeline(S, { width: 820, h: 44 })}</div>
        </div>
        <div style="grid-column:3;grid-row:2/4;background:var(--panel-solid);border-left:1px solid var(--panel-b);padding:14px;display:flex;flex-direction:column;gap:14px;min-height:0">
          ${W.params(S, 'teleop', { head: true })}
          <div><h4>State</h4>${W.fsmTimeline(W.active(S))}</div>
          <div style="margin-top:auto">${W.watchdog(S, { title: false })}</div>
        </div>
        <div style="grid-column:2;grid-row:3;display:grid;grid-template-columns:1fr 290px;border-top:1px solid var(--panel-b);background:var(--panel-solid);min-height:0">
          <div style="padding:10px 14px;min-height:0">${W.logPanel()}</div>
          <div style="padding:10px 14px;border-left:1px solid var(--panel-b);display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:8px;min-height:0">${W.CAM_ORDER.map(k => W.camTile(S, k, snap, { fill: true })).join('')}</div>
        </div>
        <div class="statusbar" style="grid-column:1/-1">${W.leds(S)}<span style="margin-left:auto">loop 30.0 Hz</span><span>backend: mock</span></div>
      </div>`,
  },
];
