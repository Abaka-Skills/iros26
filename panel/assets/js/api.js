/* The panel's only data source: the robot API served by myaloha/server.py (see /docs). */

/* A failed call throws an Error whose message is the API's reason; `status` and `detail` (which
   may be an object, e.g. a topic's missing dependencies) ride along. */
async function ok(r) {
  const data = await r.json().catch(() => null);
  if (!r.ok) {
    const d = data?.detail;
    throw Object.assign(new Error(typeof d === 'string' ? d : d?.message ?? `${r.status} ${r.statusText}`), { status: r.status, detail: d });
  }
  return data;
}

export const get = path => fetch('/api' + path, { cache: 'no-store' }).then(ok);
export const post = (path, body) => fetch('/api' + path, {
  method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body ?? {}),
}).then(ok);

/* Every camera over one socket: onFrame(key, kind, blob). Returns setKind(key, kind). */
export function cameras(onFrame, fps = 15) {
  let ws = null, wait = 250;
  const kinds = {};
  const open = () => {
    ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/cameras/ws?fps=${fps}`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => { wait = 250; for (const [key, kind] of Object.entries(kinds)) ws.send(JSON.stringify({ key, kind })); };
    ws.onmessage = e => {
      const b = new Uint8Array(e.data), n = b[0];
      const key = new TextDecoder().decode(b.subarray(1, 1 + n));
      onFrame(key, b[1 + n] ? 'depth' : 'color', new Blob([b.subarray(2 + n)], { type: 'image/jpeg' }));
    };
    ws.onclose = () => { setTimeout(open, wait); wait = Math.min(wait * 2, 4000); };
  };
  open();
  return (key, kind) => {
    kinds[key] = kind;
    if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ key, kind }));
  };
}

/* Live stream: calls on.state / on.log / on.timeline per message, on.link(true|false) when the
   socket opens or drops, and reconnects with backoff. Returns send(msg) for messages to the robot. */
export function stream(on, hz = 30) {
  let wait = 250, ws = null;
  const open = () => {
    ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/stream?hz=${hz}`);
    ws.onopen = () => { wait = 250; on.link?.(true); };
    ws.onmessage = e => { const m = JSON.parse(e.data); on[m.type]?.(m); };
    ws.onclose = () => { on.link?.(false); setTimeout(open, wait); wait = Math.min(wait * 2, 4000); };
  };
  open();
  return msg => ws?.readyState === WebSocket.OPEN && (ws.send(JSON.stringify(msg)), true);
}

/* Emergency stop over every path at once: the open socket (fastest) and REST (independent). */
export function emergencyStop(send) {
  send({ type: 'estop' });
  return post('/estop');
}

/* A running topic's results, straight from its own port: its /stream socket, or /latest every
   `period` ms. onData(envelope {seq, t, wall, data}). Returns close(). */
export function topic(url, mode, period, onData) {
  let closed = false, ws = null, timer = null, seq = -1;
  const deliver = m => { if (!closed && m?.data && m.seq !== seq) { seq = m.seq; onData(m); } };
  if (mode === 'stream') {
    const open = () => {
      if (closed) return;
      ws = new WebSocket(url.replace(/^http/, 'ws') + '/stream');
      ws.onmessage = e => deliver(JSON.parse(e.data));
      ws.onclose = () => { if (!closed) setTimeout(open, 1000); };
    };
    open();
  } else {
    const poll = async () => {
      try { deliver(await (await fetch(url + '/latest', { cache: 'no-store' })).json()); } catch {}
      if (!closed) timer = setTimeout(poll, period);
    };
    poll();
  }
  return () => { closed = true; ws?.close(); clearTimeout(timer); };
}
