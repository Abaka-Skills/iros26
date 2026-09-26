/* Mock of what the panel will poll. Shapes match Aloha.status() and Mod.status().
   MODS is dumped from the Python classes (_claude_tmp/dump_mods.py), not hand-written. */
export const MODS = {
 "reset": {
  "category": "motion",
  "doc": "Move every connected arm to the all-zero pose. Absent arms are skipped.",
  "params": {
   "goal_time": 3.0,
   "via_staged": true
  },
  "states": {
   "idle": "not started",
   "checking": "finding the connected arms",
   "staging": "connected arms move to the staged pose (position mode)",
   "zeroing": "connected arms move to all-zero (position mode)",
   "done": "finished normally",
   "stopped": "ended early on request; arms left safe",
   "failed": "ended on an error; see error"
  },
  "transitions": {
   "idle": [
    "checking"
   ],
   "checking": [
    "staging",
    "zeroing"
   ],
   "staging": [
    "zeroing"
   ]
  }
 },
 "teleop": {
  "category": "teleop",
  "doc": "Followers mirror their leaders' joints for a fixed duration.",
  "params": {
   "duration_s": 30.0,
   "align_time": 2.0,
   "sides": [
    "left",
    "right"
   ]
  },
  "states": {
   "idle": "not started",
   "pairing": "matching each side's leader and follower; incomplete sides are skipped",
   "aligning": "leaders hold still (position); followers move to the leaders' pose",
   "following": "leaders float under the operator's hand (gravity comp); followers track them (position)",
   "releasing": "leaders and followers hold their last pose (position)",
   "done": "finished normally",
   "stopped": "ended early on request; arms left safe",
   "failed": "ended on an error; see error"
  },
  "transitions": {
   "idle": [
    "pairing"
   ],
   "pairing": [
    "aligning"
   ],
   "aligning": [
    "following",
    "releasing"
   ],
   "following": [
    "releasing"
   ]
  }
 },
 "instruction": {
  "category": "teach",
  "doc": "Float the followers so they can be moved by hand, and measure them while they are.",
  "params": {
   "duration_s": 0.0,
   "sample_hz": 30.0,
   "sides": [
    "left",
    "right"
   ]
  },
  "states": {
   "idle": "not started",
   "entering": "selected followers switch to gravity comp (external effort 0)",
   "free": "followers float and are moved by hand; joint state sampled",
   "exiting": "followers hold their current pose (position)",
   "done": "finished normally",
   "stopped": "ended early on request; arms left safe",
   "failed": "ended on an error; see error"
  },
  "transitions": {
   "idle": [
    "entering"
   ],
   "entering": [
    "exiting",
    "free"
   ],
   "free": [
    "exiting"
   ]
  }
 }
};

/* Illustrative future mods, only to show how the browser scales. Not implemented. */
export const PLANNED = [
  ['sleep', 'motion', 'Fold every arm to its sleep pose and brake.'],
  ['go_to_pose', 'motion', 'Move chosen arms to a saved named pose.'],
  ['home_grippers', 'motion', 'Open and close each gripper once to re-seat it.'],
  ['teach_waypoints', 'teach', 'Hand-guide the followers and save waypoints.'],
  ['record_episode', 'data', 'Teleop while writing a LeRobot dataset episode.'],
  ['replay_episode', 'data', 'Play a recorded episode back on the followers.'],
  ['run_policy', 'policy', 'Drive the followers from a trained policy checkpoint.'],
  ['calibrate_cameras', 'calibration', 'Solve camera extrinsics from the ArUco board.'],
  ['joint_sweep', 'diagnostics', 'Sweep each joint through its range and log effort.'],
  ['latency_probe', 'diagnostics', 'Measure command-to-motion latency per arm.'],
].map(([name, category, doc]) => ({ name, category, doc, planned: true }));

export const CATEGORIES = ['motion', 'teleop', 'teach', 'data', 'policy', 'calibration', 'diagnostics'];

export const ROBOT_STATES = {
  offline: 'no arm connected',
  idle: 'arms connected, no mod running',
  running: 'a mod holds the robot',
};

const T = Date.now() / 1000;
const GRASP = [0.32, 1.12, 0.62, 0.48, -0.18, 0.4, 0.021];

/* Scenario: teleop running on the left pair; the right leader is unplugged. */
export const STATUS = {
  state: 'running',
  arms: {
    leader_left:    { name: 'leader_left',    ip: '192.168.10.2', role: 'leader',   side: 'left',  connected: true,  error: null, mode: ['external_effort'], pos: GRASP },
    leader_right:   { name: 'leader_right',   ip: '192.168.10.3', role: 'leader',   side: 'right', connected: false, error: 'timed out after 5.0 s', mode: null, pos: null },
    follower_left:  { name: 'follower_left',  ip: '192.168.10.4', role: 'follower', side: 'left',  connected: true,  error: null, mode: ['position'], pos: GRASP },
    follower_right: { name: 'follower_right', ip: '192.168.10.5', role: 'follower', side: 'right', connected: true,  error: null, mode: ['position'], pos: [0, 0, 0, 0, 0, 0, 0] },
  },
  cameras: {
    high:    { serial: '218622271652', label: 'High',    note: 'top-down',        ok: true, fps: 30.0 },
    low:     { serial: '230322270466', label: 'Low',     note: 'table level',     ok: true, fps: 29.9 },
    wrist_a: { serial: '218622271803', label: 'Wrist A', note: 'side not mapped', ok: true, fps: 30.0 },
    wrist_b: { serial: '218622272313', label: 'Wrist B', note: 'side not mapped', ok: true, fps: 29.8 },
  },
  mods: [{
    name: 'teleop', category: 'teleop', parent: null, state: 'following', since: T - 12.4, error: null,
    history: [['idle', T - 14.5], ['pairing', T - 14.5], ['aligning', T - 14.4], ['following', T - 12.4]],
    params: MODS.teleop.params,
    states: MODS.teleop.states, transitions: MODS.teleop.transitions,
    skipped: { right: 'leader_right not connected' },
  }],
};

/* Major-state history the backend will keep: one segment per state, running ones name their mod. */
export const TIMELINE = [
  { state: 'offline', t0: T - 90 },
  { state: 'idle', t0: T - 73 },
  { state: 'running', mod: 'reset', result: 'done', t0: T - 52 },
  { state: 'idle', t0: T - 43.8 },
  { state: 'running', mod: 'instruction', result: 'stopped', t0: T - 38 },
  { state: 'idle', t0: T - 21 },
  { state: 'running', mod: 'teleop', t0: T - 14.5 },
];

export const LOG = [
  [-78.0, 'info', 'robot', 'connect: 4 arms, 4 cameras requested'],
  [-73.1, 'warn', 'robot', 'leader_right (192.168.10.3) timed out after 5.0 s; marked absent'],
  [-73.0, 'info', 'robot', 'connected 3/4 arms: leader_left, follower_left, follower_right'],
  [-72.4, 'info', 'camera', '4/4 streaming at 30 fps: high, low, wrist_a, wrist_b'],
  [-52.0, 'info', 'reset', 'idle → checking; skip leader_right'],
  [-51.9, 'info', 'reset', 'checking → staging (3 s)'],
  [-48.9, 'info', 'reset', 'staging → zeroing (3 s)'],
  [-43.8, 'info', 'reset', 'zeroing → done'],
  [-38.0, 'info', 'instruction', 'idle → entering → free: follower_left, follower_right'],
  [-21.2, 'info', 'instruction', 'stop requested'],
  [-21.0, 'info', 'instruction', 'free → exiting → stopped; j1 range L 0.02…1.31'],
  [-14.5, 'info', 'teleop', 'idle → pairing'],
  [-14.5, 'warn', 'teleop', 'skip right: leader_right not connected'],
  [-14.4, 'info', 'teleop', 'pairing → aligning (2 s)'],
  [-12.4, 'info', 'teleop', 'aligning → following (30 s)'],
  [-3.1, 'warn', 'watchdog', 'follower_left joint_2 tracking error 0.09 rad'],
].map(([dt, level, src, msg]) => ({ t: T + dt, level, src, msg }));
