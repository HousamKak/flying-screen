// Thin wrapper over the engine HTTP surface.

let busyCount = 0;
const busyEl = () => document.getElementById('busy');
const busyText = () => document.getElementById('busy-text');

export function busy(on, text) {
  busyCount += on ? 1 : -1;
  busyCount = Math.max(busyCount, 0);
  const el = busyEl();
  if (!el) return;
  if (text && on) busyText().textContent = text;
  el.classList.toggle('on', busyCount > 0);
}

export async function post(path, body, label) {
  busy(true, label);
  try {
    const r = await fetch('/api/' + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    if (!r.ok) throw new Error(`${path}: ${r.status} ${await r.text()}`);
    return await r.json();
  } finally {
    busy(false);
  }
}

export async function get(path) {
  const r = await fetch('/api/' + path);
  if (!r.ok) throw new Error(`${path}: ${r.status}`);
  return await r.json();
}
