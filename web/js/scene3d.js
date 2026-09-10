// A small flat-shaded 3D renderer on canvas 2D.
//
// Everything in this tool is drawn from the engine's own numbers, and the
// assembly drawing should be no different: no CDN, no scene format, just
// polygons built from the dimensions the closure produced. Painter's
// algorithm is enough because the parts are convex and well separated.

const V = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
  norm: (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
};
export { V };

/** An orthonormal basis whose z axis is `axis`. */
function basis(axis) {
  const w = V.norm(axis);
  const ref = Math.abs(w[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  const u = V.norm(V.cross(ref, w));
  return [u, V.cross(w, u), w];
}

// ---------------------------------------------------------------------------
// Primitives. Each returns an array of faces; a face is {pts, color}.
// ---------------------------------------------------------------------------
export function box(centre, size, color, rotZ = 0) {
  const [w, d, h] = size.map((x) => x / 2);
  const c = Math.cos(rotZ), s = Math.sin(rotZ);
  const corner = (sx, sy, sz) => {
    const x = sx * w, y = sy * d;
    return [centre[0] + x * c - y * s, centre[1] + x * s + y * c, centre[2] + sz * h];
  };
  const p = {};
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    p[`${sx}${sy}${sz}`] = corner(sx, sy, sz);
  }
  const q = (a, b, cc, dd) => ({ pts: [p[a], p[b], p[cc], p[dd]], color });
  return [
    q('-1-11', '1-11', '111', '-111'),   // top
    q('-1-1-1', '-11-1', '11-1', '1-1-1'), // bottom
    q('-1-1-1', '1-1-1', '1-11', '-1-11'), // front
    q('-11-1', '-111', '111', '11-1'),   // back
    q('-1-1-1', '-1-11', '-111', '-11-1'), // left
    q('1-1-1', '11-1', '111', '1-11'),   // right
  ];
}

export function tube(p0, p1, r, color, seg = 12, capped = true) {
  const [u, v, w] = basis(V.sub(p1, p0));
  const ring = (o) => {
    const out = [];
    for (let i = 0; i < seg; i++) {
      const a = (2 * Math.PI * i) / seg;
      out.push(V.add(o, V.add(V.scale(u, r * Math.cos(a)), V.scale(v, r * Math.sin(a)))));
    }
    return out;
  };
  const A = ring(p0), B = ring(p1), faces = [];
  for (let i = 0; i < seg; i++) {
    const j = (i + 1) % seg;
    faces.push({ pts: [A[i], A[j], B[j], B[i]], color });
  }
  if (capped) { faces.push({ pts: A.slice().reverse(), color }); faces.push({ pts: B, color }); }
  return faces;
}

export function torus(centre, R, r, color, segMajor = 22, segMinor = 5) {
  const faces = [];
  const pt = (i, j) => {
    const a = (2 * Math.PI * i) / segMajor, b = (2 * Math.PI * j) / segMinor;
    const rr = R + r * Math.cos(b);
    return [centre[0] + rr * Math.cos(a), centre[1] + rr * Math.sin(a), centre[2] + r * Math.sin(b)];
  };
  for (let i = 0; i < segMajor; i++) for (let j = 0; j < segMinor; j++) {
    faces.push({ pts: [pt(i, j), pt(i + 1, j), pt(i + 1, j + 1), pt(i, j + 1)], color });
  }
  return faces;
}

/** A flat disc in the xy plane, used for the swept rotor area. */
export function disc(centre, R, color, seg = 28, alpha = 1) {
  const pts = [];
  for (let i = 0; i < seg; i++) {
    const a = (2 * Math.PI * i) / seg;
    pts.push([centre[0] + R * Math.cos(a), centre[1] + R * Math.sin(a), centre[2]]);
  }
  return [{ pts, color, alpha, flat: true }];
}

/** A propeller blade: a twisted, tapered plate from hub to tip. */
export function blade(hub, angle, R, chord, color, pitch = 0.22) {
  const faces = [];
  const N = 5;
  const at = (t, side) => {
    const rr = 0.14 * R + t * (R - 0.14 * R);
    const c = chord * (1.0 - 0.45 * t * t);
    const tw = pitch * (1 - 0.6 * t);
    const off = side * c / 2;
    const x = rr * Math.cos(angle) - off * Math.sin(angle);
    const y = rr * Math.sin(angle) + off * Math.cos(angle);
    return [hub[0] + x, hub[1] + y, hub[2] + off * tw];
  };
  for (let i = 0; i < N; i++) {
    const t0 = i / N, t1 = (i + 1) / N;
    faces.push({ pts: [at(t0, -1), at(t1, -1), at(t1, 1), at(t0, 1)], color });
  }
  return faces;
}

// ---------------------------------------------------------------------------
// Part: a named group of faces with a label anchor
// ---------------------------------------------------------------------------
export class Part {
  constructor(name, faces, anchor, lines, opts = {}) {
    this.name = name;
    this.faces = faces;
    this.anchor = anchor;
    this.lines = lines || [];
    this.side = opts.side || 'auto';
    this.key = opts.key || name;
    this.explode = opts.explode || [0, 0, 0];
  }
}

// ---------------------------------------------------------------------------
// Renderer
// ---------------------------------------------------------------------------
export class Scene {
  constructor(canvas) {
    this.canvas = canvas;
    this.parts = [];
    this.az = -0.62;
    this.el = 0.34;
    this.dist = 3.0;
    this.explode = 0;
    this.target = [0, 0, 0];
    this.hover = null;
    this.showLabels = true;
    this._bind();
  }

  _bind() {
    const c = this.canvas;
    let drag = null;
    c.style.touchAction = 'none';
    c.addEventListener('pointerdown', (e) => {
      drag = { x: e.clientX, y: e.clientY, az: this.az, el: this.el };
      c.setPointerCapture(e.pointerId);
    });
    c.addEventListener('pointermove', (e) => {
      if (!drag) return;
      this.az = drag.az + (e.clientX - drag.x) * 0.008;
      this.el = Math.max(-1.35, Math.min(1.35, drag.el + (e.clientY - drag.y) * 0.008));
      this.draw();
    });
    const stop = () => { drag = null; };
    c.addEventListener('pointerup', stop);
    c.addEventListener('pointercancel', stop);
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.dist = Math.max(1.1, Math.min(9, this.dist * (1 + Math.sign(e.deltaY) * 0.12)));
      this.draw();
    }, { passive: false });
  }

  setView(name) {
    const views = {
      iso: [-0.62, 0.34], front: [-Math.PI / 2, 0.02], side: [0, 0.02],
      top: [-Math.PI / 2, 1.34], three_quarter: [-1.05, 0.22],
    };
    const v = views[name] || views.iso;
    this.az = v[0]; this.el = v[1];
    this.draw();
  }

  _camera() {
    const ce = Math.cos(this.el), se = Math.sin(this.el);
    const ca = Math.cos(this.az), sa = Math.sin(this.az);
    const eye = [this.dist * ce * ca, this.dist * ce * sa, this.dist * se];
    const fwd = V.norm(V.sub(this.target, eye));
    const right = V.norm(V.cross(fwd, [0, 0, 1]));
    const up = V.cross(right, fwd);
    return { eye, fwd, right, up };
  }

  draw() {
    const cv = this.canvas;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth || 800, H = cv.clientHeight || 520;
    if (cv.width !== Math.round(W * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    const g = cv.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const css = getComputedStyle(document.documentElement);
    const col = (n, f) => (css.getPropertyValue(n) || f).trim();
    g.clearRect(0, 0, W, H);
    g.fillStyle = col('--bg', '#0e1116');
    g.fillRect(0, 0, W, H);

    const cam = this._camera();
    const f = Math.min(W, H) * 0.95;
    const project = (p) => {
      const d = V.sub(p, cam.eye);
      const z = V.dot(d, cam.fwd);
      if (z <= 0.02) return null;
      return [W / 2 + (f * V.dot(d, cam.right)) / z, H / 2 - (f * V.dot(d, cam.up)) / z, z];
    };

    const light = V.norm([0.4, -0.7, 0.9]);
    const items = [];
    for (const part of this.parts) {
      const off = V.scale(part.explode, this.explode);
      for (const face of part.faces) {
        const pts = face.pts.map((p) => V.add(p, off));
        const proj = pts.map(project);
        if (proj.some((q) => q === null)) continue;
        let z = 0;
        for (const q of proj) z += q[2];
        z /= proj.length;
        let shade = 1;
        if (!face.flat && pts.length >= 3) {
          const n = V.norm(V.cross(V.sub(pts[1], pts[0]), V.sub(pts[2], pts[0])));
          shade = 0.42 + 0.58 * Math.abs(V.dot(n, light));
        } else shade = 0.8;
        items.push({ proj, color: face.color, shade, z, alpha: face.alpha, part });
      }
    }
    items.sort((a, b) => b.z - a.z);

    for (const it of items) {
      const dim = this.hover && it.part.key !== this.hover ? 0.42 : 1;
      g.beginPath();
      g.moveTo(it.proj[0][0], it.proj[0][1]);
      for (let i = 1; i < it.proj.length; i++) g.lineTo(it.proj[i][0], it.proj[i][1]);
      g.closePath();
      g.globalAlpha = (it.alpha === undefined ? 1 : it.alpha) * dim;
      g.fillStyle = shadeHex(it.color, it.shade);
      g.fill();
      g.globalAlpha = 0.55 * dim;
      g.strokeStyle = shadeHex(it.color, it.shade * 0.55);
      g.lineWidth = 0.6;
      g.stroke();
    }
    g.globalAlpha = 1;

    if (this.showLabels) this._labels(g, W, H, project, col);
    return { project, W, H };
  }

  _labels(g, W, H, project, col) {
    const cands = [];
    for (const part of this.parts) {
      if (!part.lines.length) continue;
      const off = V.scale(part.explode, this.explode);
      const q = project(V.add(part.anchor, off));
      if (!q) continue;
      cands.push({ part, x: q[0], y: q[1] });
    }
    const left = cands.filter((c) => c.x < W / 2).sort((a, b) => a.y - b.y);
    const right = cands.filter((c) => c.x >= W / 2).sort((a, b) => a.y - b.y);

    g.font = '11px ui-monospace, SFMono-Regular, Menlo, monospace';
    const rowH = 30, pad = 12;
    const place = (list, side) => {
      const n = list.length;
      const total = n * rowH;
      let y = Math.max(pad + 10, Math.min(H / 2 - total / 2, H - total - pad));
      for (const c of list) {
        const lx = side < 0 ? pad : W - pad;
        const ly = y + rowH / 2;
        const dim = this.hover && c.part.key !== this.hover ? 0.35 : 1;
        g.globalAlpha = dim;
        g.strokeStyle = col('--line', '#39424f');
        g.lineWidth = 1;
        g.beginPath();
        const elbow = side < 0 ? lx + 92 : lx - 92;
        g.moveTo(elbow, ly);
        g.lineTo(side < 0 ? elbow + 14 : elbow - 14, ly);
        g.lineTo(c.x, c.y);
        g.stroke();
        g.fillStyle = col('--accent', '#5ec8f2');
        g.beginPath(); g.arc(c.x, c.y, 2.6, 0, 7); g.fill();
        g.textAlign = side < 0 ? 'left' : 'right';
        g.textBaseline = 'alphabetic';
        g.fillStyle = col('--fg', '#d7dee7');
        g.fillText(c.part.lines[0], lx, ly - 1);
        g.fillStyle = col('--muted', '#7f8b9a');
        g.fillText(c.part.lines[1] || '', lx, ly + 12);
        g.globalAlpha = 1;
        y += rowH;
      }
    };
    place(left, -1);
    place(right, 1);
  }
}

function shadeHex(hex, s) {
  const h = hex.replace('#', '');
  const n = parseInt(h.length === 3 ? h.split('').map((c) => c + c).join('') : h, 16);
  const r = Math.min(255, Math.round(((n >> 16) & 255) * s));
  const gg = Math.min(255, Math.round(((n >> 8) & 255) * s));
  const b = Math.min(255, Math.round((n & 255) * s));
  return `rgb(${r},${gg},${b})`;
}
