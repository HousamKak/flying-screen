// Minimal canvas plotting: line charts, heatmaps, bar charts.
// Written here rather than pulled in so the whole tool runs offline and the
// visual language stays consistent with the rest of the instrument panel.

const CSS = getComputedStyle(document.documentElement);
const col = (name, fallback) => (CSS.getPropertyValue(name) || fallback).trim();

export const PALETTE = ['#5ec8f2', '#f2a65e', '#8ce99a', '#f28b82',
  '#c39bf5', '#ffd76e', '#7fd1c1', '#f58fc2'];

function dpiFit(canvas) {
  const r = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 600;
  const h = canvas.clientHeight || 300;
  if (canvas.width !== Math.round(w * r) || canvas.height !== Math.round(h * r)) {
    canvas.width = Math.round(w * r);
    canvas.height = Math.round(h * r);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(r, 0, 0, r, 0, 0);
  return { ctx, w, h };
}

const isNum = (v) => typeof v === 'number' && isFinite(v);

function niceTicks(lo, hi, count, log) {
  if (log) {
    const a = Math.log10(Math.max(lo, 1e-12));
    const b = Math.log10(Math.max(hi, 1e-11));
    const out = [];
    const step = Math.max(1, Math.ceil((b - a) / Math.max(count, 1)));
    for (let e = Math.ceil(a); e <= Math.floor(b); e += step) out.push(Math.pow(10, e));
    if (out.length < 2) {
      for (let i = 0; i <= count; i++) out.push(Math.pow(10, a + (b - a) * i / count));
    }
    return out;
  }
  const span = hi - lo;
  if (!(span > 0)) return [lo];
  const raw = span / Math.max(count, 1);
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-9; v += step) out.push(v);
  return out;
}

export function fmt(v, digits = 3) {
  if (!isNum(v)) return '--';
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a >= 1e5 || a < 1e-3) return v.toExponential(1).replace('e+', 'e');
  return String(Number(v.toPrecision(digits)));
}

// ---------------------------------------------------------------------------
// Line / scatter chart
// ---------------------------------------------------------------------------
// spec = { series:[{x:[],y:[],label,color,width,dash,fill,type:'line'|'dots'|'area'}],
//          xlabel, ylabel, xlog, ylog, xlim, ylim, vlines:[{x,label,color,dash}],
//          hlines:[...], bands:[{x0,x1,color,label}], legend:true, markers:[{x,y,label,color}] }
export function linechart(canvas, spec) {
  const { ctx, w, h } = dpiFit(canvas);
  ctx.clearRect(0, 0, w, h);
  const S = spec.series || [];
  const pad = { l: 62, r: spec.padRight ?? 14, t: 14, b: 40 };
  const iw = Math.max(w - pad.l - pad.r, 10);
  const ih = Math.max(h - pad.t - pad.b, 10);

  let xs = [], ys = [];
  for (const s of S) {
    for (let i = 0; i < s.x.length; i++) {
      const X = s.x[i], Y = s.y[i];
      if (isNum(X) && (!spec.xlog || X > 0)) xs.push(X);
      if (isNum(Y) && (!spec.ylog || Y > 0)) ys.push(Y);
    }
  }
  for (const m of spec.markers || []) { if (isNum(m.x)) xs.push(m.x); if (isNum(m.y)) ys.push(m.y); }
  for (const v of spec.vlines || []) if (isNum(v.x)) xs.push(v.x);
  for (const v of spec.hlines || []) if (isNum(v.y)) ys.push(v.y);
  if (!xs.length || !ys.length) {
    ctx.fillStyle = col('--muted', '#888');
    ctx.font = '12px ui-monospace, monospace';
    ctx.fillText('no data', pad.l, pad.t + ih / 2);
    return;
  }
  let x0 = spec.xlim ? spec.xlim[0] : Math.min(...xs);
  let x1 = spec.xlim ? spec.xlim[1] : Math.max(...xs);
  let y0 = spec.ylim ? spec.ylim[0] : Math.min(...ys);
  let y1 = spec.ylim ? spec.ylim[1] : Math.max(...ys);
  if (x1 === x0) { x1 = x0 + 1; }
  if (y1 === y0) { y1 = y0 + Math.abs(y0 || 1) * 0.1; }
  if (!spec.ylim && !spec.ylog) { const m = (y1 - y0) * 0.08; y0 -= m; y1 += m; }
  if (spec.ylog) { y0 = Math.max(y0, 1e-12); }

  const px = (v) => spec.xlog
    ? pad.l + iw * (Math.log10(v) - Math.log10(x0)) / (Math.log10(x1) - Math.log10(x0))
    : pad.l + iw * (v - x0) / (x1 - x0);
  const py = (v) => spec.ylog
    ? pad.t + ih - ih * (Math.log10(v) - Math.log10(y0)) / (Math.log10(y1) - Math.log10(y0))
    : pad.t + ih - ih * (v - y0) / (y1 - y0);

  // bands
  for (const b of spec.bands || []) {
    ctx.fillStyle = b.color || 'rgba(242,139,130,0.10)';
    const a = px(Math.max(b.x0, x0)), c = px(Math.min(b.x1, x1));
    ctx.fillRect(Math.min(a, c), pad.t, Math.abs(c - a), ih);
  }

  // grid + axes
  ctx.strokeStyle = col('--grid', '#232a33');
  ctx.fillStyle = col('--muted', '#7d8794');
  ctx.lineWidth = 1;
  ctx.font = '11px ui-monospace, monospace';
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (const t of niceTicks(y0, y1, 5, spec.ylog)) {
    const Y = Math.round(py(t)) + 0.5;
    if (Y < pad.t - 1 || Y > pad.t + ih + 1) continue;
    ctx.beginPath(); ctx.moveTo(pad.l, Y); ctx.lineTo(pad.l + iw, Y); ctx.stroke();
    ctx.fillText(fmt(t), pad.l - 8, Y);
  }
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (const t of niceTicks(x0, x1, 6, spec.xlog)) {
    const X = Math.round(px(t)) + 0.5;
    if (X < pad.l - 1 || X > pad.l + iw + 1) continue;
    ctx.beginPath(); ctx.moveTo(X, pad.t); ctx.lineTo(X, pad.t + ih); ctx.stroke();
    ctx.fillText(spec.xfmt ? spec.xfmt(t) : fmt(t), X, pad.t + ih + 7);
  }
  ctx.strokeStyle = col('--line', '#39424f');
  ctx.beginPath();
  ctx.moveTo(pad.l + 0.5, pad.t); ctx.lineTo(pad.l + 0.5, pad.t + ih);
  ctx.lineTo(pad.l + iw, pad.t + ih); ctx.stroke();

  if (spec.xlabel) {
    ctx.fillStyle = col('--muted', '#7d8794');
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    ctx.fillText(spec.xlabel, pad.l + iw / 2, h - 4);
  }
  if (spec.ylabel) {
    ctx.save(); ctx.translate(12, pad.t + ih / 2); ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    ctx.fillStyle = col('--muted', '#7d8794');
    ctx.fillText(spec.ylabel, 0, 0); ctx.restore();
  }

  // series
  S.forEach((s, i) => {
    const c = s.color || PALETTE[i % PALETTE.length];
    if (s.type === 'area') {
      ctx.fillStyle = s.fill || (c + '22');
      ctx.beginPath(); let started = false;
      for (let k = 0; k < s.x.length; k++) {
        const X = s.x[k], Y = s.y[k];
        if (!isNum(X) || !isNum(Y)) continue;
        if (!started) { ctx.moveTo(px(X), py(Math.max(y0, 0))); started = true; }
        ctx.lineTo(px(X), py(Y));
      }
      if (started) {
        const lx = s.x[s.x.length - 1];
        ctx.lineTo(px(lx), py(Math.max(y0, 0))); ctx.closePath(); ctx.fill();
      }
    }
    ctx.strokeStyle = c; ctx.fillStyle = c;
    ctx.lineWidth = s.width || 1.8;
    ctx.setLineDash(s.dash || []);
    if (s.type === 'dots') {
      for (let k = 0; k < s.x.length; k++) {
        if (!isNum(s.x[k]) || !isNum(s.y[k])) continue;
        ctx.beginPath(); ctx.arc(px(s.x[k]), py(s.y[k]), s.r || 2.6, 0, 7); ctx.fill();
      }
    } else {
      ctx.beginPath(); let pen = false;
      for (let k = 0; k < s.x.length; k++) {
        const X = s.x[k], Y = s.y[k];
        const bad = !isNum(X) || !isNum(Y) || (spec.ylog && Y <= 0) || (spec.xlog && X <= 0);
        if (bad) { pen = false; continue; }
        const a = px(X), b = py(Y);
        if (!pen) { ctx.moveTo(a, b); pen = true; } else ctx.lineTo(a, b);
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);
  });

  for (const v of spec.vlines || []) {
    if (!isNum(v.x)) continue;
    ctx.strokeStyle = v.color || col('--accent2', '#f2a65e');
    ctx.setLineDash(v.dash || [4, 4]); ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(px(v.x), pad.t); ctx.lineTo(px(v.x), pad.t + ih); ctx.stroke();
    ctx.setLineDash([]);
    if (v.label) {
      ctx.fillStyle = v.color || col('--accent2', '#f2a65e');
      ctx.textAlign = 'left'; ctx.textBaseline = 'top'; ctx.font = '10px ui-monospace, monospace';
      ctx.fillText(v.label, Math.min(px(v.x) + 4, pad.l + iw - 60), pad.t + (v.row || 0) * 12 + 2);
    }
  }
  for (const v of spec.hlines || []) {
    if (!isNum(v.y)) continue;
    ctx.strokeStyle = v.color || col('--accent2', '#f2a65e');
    ctx.setLineDash(v.dash || [4, 4]); ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(pad.l, py(v.y)); ctx.lineTo(pad.l + iw, py(v.y)); ctx.stroke();
    ctx.setLineDash([]);
    if (v.label) {
      ctx.fillStyle = v.color || col('--accent2', '#f2a65e');
      ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'; ctx.font = '10px ui-monospace, monospace';
      ctx.fillText(v.label, pad.l + 4, py(v.y) - 3);
    }
  }
  for (const m of spec.markers || []) {
    if (!isNum(m.x) || !isNum(m.y)) continue;
    ctx.fillStyle = m.color || col('--accent', '#5ec8f2');
    ctx.beginPath(); ctx.arc(px(m.x), py(m.y), m.r || 4.5, 0, 7); ctx.fill();
    ctx.strokeStyle = col('--bg', '#11151b'); ctx.lineWidth = 1.5; ctx.stroke();
    if (m.label) {
      ctx.fillStyle = m.color || col('--accent', '#5ec8f2');
      ctx.font = '10px ui-monospace, monospace';
      ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
      ctx.fillText(m.label, px(m.x) + 7, py(m.y) - 5);
    }
  }

  if (spec.legend !== false && S.some((s) => s.label)) {
    ctx.font = '11px ui-monospace, monospace';
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    let yy = pad.t + 8;
    S.forEach((s, i) => {
      if (!s.label) return;
      const c = s.color || PALETTE[i % PALETTE.length];
      ctx.strokeStyle = c; ctx.lineWidth = 2.4;
      ctx.setLineDash(s.dash || []);
      ctx.beginPath(); ctx.moveTo(pad.l + iw - 130, yy); ctx.lineTo(pad.l + iw - 112, yy); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = col('--fg', '#d5dce4');
      ctx.fillText(s.label, pad.l + iw - 106, yy);
      yy += 14;
    });
  }
}

// ---------------------------------------------------------------------------
// Heatmap with an infeasible mask
// ---------------------------------------------------------------------------
const RAMP = [[13, 25, 42], [22, 72, 110], [24, 128, 140], [92, 178, 122],
  [220, 205, 100], [244, 150, 80], [238, 96, 92]];

export function colormap(t) {
  t = Math.max(0, Math.min(1, t));
  const s = t * (RAMP.length - 1);
  const i = Math.min(Math.floor(s), RAMP.length - 2);
  const f = s - i;
  const a = RAMP[i], b = RAMP[i + 1];
  return `rgb(${Math.round(a[0] + (b[0] - a[0]) * f)},${Math.round(a[1] + (b[1] - a[1]) * f)},${Math.round(a[2] + (b[2] - a[2]) * f)})`;
}

// spec = { x, y, z (rows over y), feasible, xlabel, ylabel, xlog, ylog, log,
//          overlay:{x:[],y:[],label}, point:{x,y}, onHover }
export function heatmap(canvas, spec) {
  const { ctx, w, h } = dpiFit(canvas);
  ctx.clearRect(0, 0, w, h);
  const pad = { l: 66, r: 74, t: 14, b: 42 };
  const iw = Math.max(w - pad.l - pad.r, 10);
  const ih = Math.max(h - pad.t - pad.b, 10);
  const X = spec.x, Y = spec.y, Z = spec.z, F = spec.feasible;
  if (!X || !Y || !Z) return;

  let lo = Infinity, hi = -Infinity;
  for (const row of Z) for (const v of row) if (isNum(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  const useLog = spec.log && lo > 0 && hi / lo > 40;
  const norm = (v) => {
    if (!isNum(v)) return null;
    if (useLog) return (Math.log10(v) - Math.log10(lo)) / (Math.log10(hi) - Math.log10(lo) || 1);
    return (v - lo) / ((hi - lo) || 1);
  };

  const cw = iw / X.length, chh = ih / Y.length;
  for (let j = 0; j < Y.length; j++) {
    for (let i = 0; i < X.length; i++) {
      const feas = F ? F[j][i] : 1;
      const t = norm(Z[j][i]);
      ctx.fillStyle = (!feas || t === null) ? col('--infeasible', '#1a1015') : colormap(t);
      ctx.fillRect(pad.l + i * cw, pad.t + ih - (j + 1) * chh, Math.ceil(cw) + 0.5, Math.ceil(chh) + 0.5);
    }
  }
  // hatch the infeasible region so it reads as excluded, not merely dark
  if (F) {
    ctx.save();
    ctx.beginPath(); ctx.rect(pad.l, pad.t, iw, ih); ctx.clip();
    ctx.strokeStyle = 'rgba(242,139,130,0.25)'; ctx.lineWidth = 1;
    for (let j = 0; j < Y.length; j++) for (let i = 0; i < X.length; i++) {
      if (F[j][i]) continue;
      const x = pad.l + i * cw, y = pad.t + ih - (j + 1) * chh;
      ctx.beginPath(); ctx.moveTo(x, y + chh); ctx.lineTo(x + cw, y); ctx.stroke();
    }
    ctx.restore();
  }

  const pxv = (v) => spec.xlog
    ? pad.l + iw * (Math.log10(v) - Math.log10(X[0])) / (Math.log10(X[X.length - 1]) - Math.log10(X[0]))
    : pad.l + iw * (v - X[0]) / (X[X.length - 1] - X[0]);
  const pyv = (v) => spec.ylog
    ? pad.t + ih - ih * (Math.log10(v) - Math.log10(Y[0])) / (Math.log10(Y[Y.length - 1]) - Math.log10(Y[0]))
    : pad.t + ih - ih * (v - Y[0]) / (Y[Y.length - 1] - Y[0]);

  if (spec.overlay && spec.overlay.x) {
    ctx.save();
    ctx.beginPath(); ctx.rect(pad.l, pad.t, iw, ih); ctx.clip();
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
    ctx.beginPath(); let pen = false;
    for (let i = 0; i < spec.overlay.x.length; i++) {
      const a = spec.overlay.x[i], b = spec.overlay.y[i];
      if (!isNum(a) || !isNum(b)) { pen = false; continue; }
      const px = pxv(a), py = pyv(b);
      if (!pen) { ctx.moveTo(px, py); pen = true; } else ctx.lineTo(px, py);
    }
    ctx.stroke(); ctx.setLineDash([]); ctx.restore();
  }
  if (spec.point && isNum(spec.point.x) && isNum(spec.point.y)) {
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
    const px = pxv(spec.point.x), py = pyv(spec.point.y);
    ctx.beginPath(); ctx.arc(px, py, 6, 0, 7); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(px - 10, py); ctx.lineTo(px + 10, py);
    ctx.moveTo(px, py - 10); ctx.lineTo(px, py + 10); ctx.stroke();
  }

  ctx.strokeStyle = col('--line', '#39424f'); ctx.lineWidth = 1;
  ctx.strokeRect(pad.l + 0.5, pad.t + 0.5, iw, ih);
  ctx.fillStyle = col('--muted', '#7d8794'); ctx.font = '11px ui-monospace, monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (const t of niceTicks(X[0], X[X.length - 1], 5, spec.xlog)) {
    if (t < X[0] || t > X[X.length - 1]) continue;
    ctx.fillText(spec.xfmt ? spec.xfmt(t) : fmt(t), pxv(t), pad.t + ih + 7);
  }
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (const t of niceTicks(Y[0], Y[Y.length - 1], 5, spec.ylog)) {
    if (t < Y[0] || t > Y[Y.length - 1]) continue;
    ctx.fillText(fmt(t), pad.l - 8, pyv(t));
  }
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  if (spec.xlabel) ctx.fillText(spec.xlabel, pad.l + iw / 2, h - 4);
  if (spec.ylabel) {
    ctx.save(); ctx.translate(13, pad.t + ih / 2); ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    ctx.fillText(spec.ylabel, 0, 0); ctx.restore();
  }

  // colour bar
  const bx = pad.l + iw + 16, bw = 14;
  for (let i = 0; i <= ih; i++) {
    ctx.fillStyle = colormap(i / ih);
    ctx.fillRect(bx, pad.t + ih - i, bw, 1.5);
  }
  ctx.strokeStyle = col('--line', '#39424f');
  ctx.strokeRect(bx + 0.5, pad.t + 0.5, bw, ih);
  ctx.fillStyle = col('--muted', '#7d8794');
  ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.font = '10px ui-monospace, monospace';
  const nb = 5;
  for (let i = 0; i <= nb; i++) {
    const t = i / nb;
    const v = useLog ? Math.pow(10, Math.log10(lo) + t * (Math.log10(hi) - Math.log10(lo)))
      : lo + t * (hi - lo);
    ctx.fillText(fmt(v, 2), bx + bw + 4, pad.t + ih - t * ih);
  }
  return { pxv, pyv, pad, iw, ih, lo, hi };
}

// ---------------------------------------------------------------------------
// Horizontal bars (mass and power breakdowns, sensitivities)
// ---------------------------------------------------------------------------
// items = [{label, value, color, note}]
export function bars(canvas, spec) {
  const { ctx, w, h } = dpiFit(canvas);
  ctx.clearRect(0, 0, w, h);
  const items = spec.items || [];
  if (!items.length) return;
  const padL = spec.labelWidth || 118, padR = 64, padT = 6;
  const rowH = Math.min(spec.rowHeight || 26, (h - padT * 2) / items.length);
  const vals = items.map((d) => d.value).filter(isNum);
  const maxV = Math.max(...vals.map((v) => Math.abs(v)), 1e-12);
  const signed = spec.signed || vals.some((v) => v < 0);
  const iw = w - padL - padR;
  const zero = signed ? padL + iw / 2 : padL;
  const scale = signed ? (iw / 2) / maxV : iw / maxV;

  ctx.font = '11px ui-monospace, monospace';
  items.forEach((d, i) => {
    const y = padT + i * rowH;
    ctx.fillStyle = col('--muted', '#7d8794');
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    ctx.fillText(d.label, padL - 8, y + rowH / 2);
    if (!isNum(d.value)) return;
    const len = d.value * scale;
    ctx.fillStyle = d.color || PALETTE[i % PALETTE.length];
    const bh = Math.max(rowH - 9, 6);
    ctx.fillRect(Math.min(zero, zero + len), y + (rowH - bh) / 2, Math.abs(len), bh);
    ctx.fillStyle = col('--fg', '#d5dce4');
    ctx.textAlign = 'left';
    ctx.fillText(spec.format ? spec.format(d.value) : fmt(d.value),
      Math.max(zero + len, zero) + 6, y + rowH / 2);
  });
  if (signed) {
    ctx.strokeStyle = col('--line', '#39424f');
    ctx.beginPath(); ctx.moveTo(zero + 0.5, padT); ctx.lineTo(zero + 0.5, padT + items.length * rowH);
    ctx.stroke();
  }
}

// ---------------------------------------------------------------------------
// Stacked single bar, for the mass budget
// ---------------------------------------------------------------------------
export function stack(canvas, spec) {
  const { ctx, w, h } = dpiFit(canvas);
  ctx.clearRect(0, 0, w, h);
  const parts = (spec.parts || []).filter((p) => isNum(p.value) && p.value > 0);
  const total = parts.reduce((a, b) => a + b.value, 0);
  if (!(total > 0)) return;
  const barH = spec.barHeight || 30;
  const top = 8;
  let x = 0;
  ctx.font = '11px ui-monospace, monospace';
  parts.forEach((p, i) => {
    const bw = (p.value / total) * w;
    ctx.fillStyle = p.color || PALETTE[i % PALETTE.length];
    ctx.fillRect(x, top, bw, barH);
    if (bw > 42) {
      ctx.fillStyle = '#0d1117';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(`${Math.round(100 * p.value / total)}%`, x + bw / 2, top + barH / 2);
    }
    x += bw;
  });
  ctx.textAlign = 'left'; ctx.textBaseline = 'top';
  let lx = 0;
  parts.forEach((p, i) => {
    const label = `${p.label} ${fmt(p.value, 3)}`;
    const wdt = ctx.measureText(label).width + 20;
    if (lx + wdt > w) return;
    ctx.fillStyle = p.color || PALETTE[i % PALETTE.length];
    ctx.fillRect(lx, top + barH + 10, 9, 9);
    ctx.fillStyle = col('--muted', '#7d8794');
    ctx.fillText(label, lx + 13, top + barH + 10);
    lx += wdt;
  });
}
