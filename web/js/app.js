// The tool. Eight views over one parameter set, each one a layer of the model.

import { get, post, busy } from './api.js';
import { Controls } from './controls.js';
import { linechart, heatmap, bars, stack, fmt, PALETTE } from './plot.js';
import { makeScene } from './assembly.js';

const S = {
  schema: null, groups: null, defaults: null, meta: null,
  params: null, tab: 'closure', design: null, controls: null,
  cache: {}, redrawers: [], model: 'fixed_area',
};

// ---------------------------------------------------------------------------
// tiny DOM helpers
// ---------------------------------------------------------------------------
const h = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v;
    else if (k === 'html') e.innerHTML = v;
    else if (k.startsWith('on')) e[k] = v;
    else if (v !== undefined && v !== null) e.setAttribute(k, v);
  }
  for (const kid of kids.flat()) {
    if (kid === null || kid === undefined || kid === false) continue;
    e.appendChild(typeof kid === 'string' ? document.createTextNode(kid) : kid);
  }
  return e;
};
const card = (title, sub, ...kids) =>
  h('div', { class: 'card' }, h('h3', {}, title), sub ? h('p', { class: 'sub', html: sub }) : null, ...kids);
const cv = (cls) => h('canvas', { class: cls });
const kv = (rows) => {
  const t = h('div', { class: 'kv' });
  for (const r of rows) {
    if (r === '-') { t.appendChild(h('hr')); continue; }
    const [k, v, cls] = r;
    t.appendChild(h('div', { class: 'k' }, k));
    t.appendChild(h('div', { class: 'v ' + (cls || '') }, v));
  }
  return t;
};
const stat = (label, value, cls) =>
  h('div', { class: 'stat' }, h('div', { class: 'big ' + (cls || '') }, value), h('div', { class: 'lbl' }, label));
const warns = (list, bad) => {
  if (!list || !list.length) return null;
  return h('ul', { class: 'warnlist' }, list.map((w) => h('li', { class: bad ? 'bad' : '' }, w)));
};
const secs = (s) => {
  if (!isFinite(s)) return '--';
  if (s < 90) return `${fmt(s, 3)} s`;
  if (s < 5400) return `${fmt(s / 60, 3)} min`;
  return `${fmt(s / 3600, 3)} h`;
};
const view = () => document.getElementById('view');

function draw(fn) {
  // Re-run on resize so canvases stay crisp.
  S.redrawers.push(fn);
  fn();
}
let resizeTimer = null;
window.addEventListener('resize', () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => S.redrawers.forEach((f) => { try { f(); } catch (e) { /* stale */ } }), 120);
});

// ---------------------------------------------------------------------------
// Tabs
// ---------------------------------------------------------------------------
const TABS = [
  ['build', 'Build', renderBuild],
  ['spec', 'Spec sheet', renderSpec],
  ['closure', 'Closure', renderClosure],
  ['wall', 'The wall', renderWall],
  ['map', 'Feasibility map', renderMap],
  ['scaling', 'Rotor scaling', renderScaling],
  ['machine', 'Machine', renderMachine],
  ['flight', 'Flight', renderFlight],
  ['loop', 'Energy loop', renderLoop],
  ['optimise', 'Optimise', renderOptimise],
  ['tether', 'Tether', renderTether],
];

let renderToken = 0;
async function show(id, force) {
  const token = ++renderToken;
  S.tab = id;
  for (const b of document.querySelectorAll('#tabs button')) b.classList.toggle('active', b.dataset.id === id);
  S.redrawers = [];
  const entry = TABS.find((t) => t[0] === id);
  view().innerHTML = '';
  try {
    await entry[2](force);
  } catch (e) {
    // A view that lost the race has already been replaced; say nothing.
    if (token !== renderToken) return;
    view().appendChild(card('Error', '', h('div', { class: 'note' }, String(e && e.message || e))));
    console.error(e);
  }
}

/** True while this render is still the one the user is looking at. */
function current(token) { return token === renderToken; }

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------
(async function boot() {
  const meta = await get('schema');
  S.schema = meta.schema; S.groups = meta.groups; S.defaults = meta.defaults; S.meta = meta;
  S.params = { ...meta.defaults };

  const tabs = document.getElementById('tabs');
  for (const [id, label] of TABS) {
    tabs.appendChild(h('button', { 'data-id': id, onclick: () => show(id, true) }, label));
  }

  // Named starting points, so the shape of the problem is one click away.
  // Built defensively: a browser holding a cached index.html from before this
  // element existed must not take the whole sidebar down with it.
  let pbox = document.getElementById('presetbox');
  if (!pbox) {
    pbox = h('div', { id: 'presetbox' });
    const host = document.getElementById('params');
    host.parentNode.insertBefore(pbox, host);
  }
  const psel = h('select', { style: 'width:100%' },
    h('option', { value: '' }, 'starting point...'),
    ...meta.presets.map((p) => h('option', { value: p.name }, p.name)));
  const pnote = h('div', { class: 'note', style: 'margin-top:6px;font-size:10.5px' },
    meta.presets[0].note);
  psel.onchange = () => {
    const p = meta.presets.find((x) => x.name === psel.value);
    if (!p) return;
    pnote.textContent = p.note;
    S.model = p.model || 'fixed_area';
    S.controls.setAll(p.params);
  };
  pbox.appendChild(h('div', { style: 'padding:10px 14px;border-bottom:1px solid var(--line)' },
    psel, pnote));

  let debounce = null;
  S.controls = new Controls(document.getElementById('params'), S.schema, S.groups, S.defaults,
    (values, key, live) => {
      S.params = { ...values };
      S.cache = {};
      clearTimeout(debounce);
      debounce = setTimeout(() => show(S.tab, false), live ? 220 : 40);
    });

  document.getElementById('btn-reset').onclick = () => S.controls.reset();
  document.getElementById('btn-copy').onclick = async () => {
    await navigator.clipboard.writeText(JSON.stringify(S.params, null, 2));
    flash('parameters copied');
  };
  document.getElementById('btn-paste').onclick = async () => {
    const txt = prompt('Paste a parameter JSON object');
    if (!txt) return;
    try { S.controls.setAll(JSON.parse(txt)); } catch (e) { alert('not valid JSON'); }
  };

  await show('build', true);
})();

function flash(msg) {
  const b = document.getElementById('busy-text');
  busy(true, msg);
  setTimeout(() => busy(false), 700);
}

async function design(force) {
  if (!force && S.cache.design) return S.cache.design;
  S.cache.design = await post('design', { params: S.params, model: S.model }, 'solving closure');
  return S.cache.design;
}

// ---------------------------------------------------------------------------
// 0. Build: the parts, drawn to the dimensions the closure produced
// ---------------------------------------------------------------------------
const BUILD = { spec: null, scene: null };

async function renderBuild() {
  const v = view();
  const token = renderToken;
  const spec = await post('spec', {
    params: S.params, fly: false, governor: { v_max: 2.5, a_max: 2.5 },
  }, 'sizing the machine');
  if (!current(token)) return;
  if (!spec.feasible) {
    v.appendChild(card('This machine does not close', '', h('div', { class: 'note' }, spec.reason)));
    return;
  }
  BUILD.spec = spec;

  const cvs = h('canvas', {
    class: 'anim',
    style: 'height:560px;cursor:grab;display:block;border-radius:5px',
  });
  const holder = card('Assembly',
    'Every dimension is the one the closure produced: arm length and wall from the beam model, rotor diameter and chord from the blade model, motor size from the torque it holds, cell count from the electrical chain. Drag to orbit, scroll to zoom.',
    cvs);
  v.appendChild(h('div', { class: 'row' },
    ...['iso', 'three_quarter', 'front', 'side', 'top'].map((k) =>
      h('button', { class: 'act', onclick: () => BUILD.scene.setView(k) },
        k.replace('_', ' '))),
    h('label', {}, 'explode'),
    h('input', {
      type: 'range', min: 0, max: 1, step: 0.01, value: 0, style: 'width:150px',
      oninput: (e) => { BUILD.scene.explode = +e.target.value; BUILD.scene.draw(); },
    }),
    h('label', { style: 'color:var(--fg)' },
      h('input', {
        type: 'checkbox', checked: '',
        onchange: (e) => { BUILD.scene.showLabels = e.target.checked; BUILD.scene.draw(); },
      }), ' callouts'),
    h('label', { style: 'color:var(--fg)', title: 'Draws a head at the working distance and the circle the blade tips sweep' },
      h('input', {
        type: 'checkbox',
        onchange: (e) => BUILD.scene.setUser(e.target.checked),
      }), ' show me and the blade reach')));
  v.appendChild(holder);

  const { scene, bom, span, reach } = makeScene(cvs, spec, S.params);
  BUILD.scene = scene;
  draw(() => scene.draw());

  // Hovering a row in the parts list highlights that part in the drawing.
  const groups = [...new Set(bom.map((r) => r.group))];
  const rows = [];
  let total = 0;
  for (const g of groups) {
    const items = bom.filter((r) => r.group === g);
    const sub = items.reduce((a, r) => a + r.mass, 0);
    total += sub;
    rows.push(h('tr', { style: 'background:var(--panel2)' },
      h('td', { colspan: '4', style: 'color:var(--accent);letter-spacing:.08em;text-transform:uppercase;font-size:10.5px' },
        `${g}: ${fmt(1000 * sub, 3)} g`)));
    for (const r of items) {
      const key = keyFor(r.item);
      const tr = h('tr', {},
        h('td', {}, r.item),
        h('td', {}, r.spec),
        h('td', {}, fmt(1000 * r.mass, 3) + ' g'),
        h('td', { style: 'color:var(--muted);white-space:normal' }, r.note || ''));
      tr.onmouseenter = () => { scene.hover = key; scene.draw(); };
      tr.onmouseleave = () => { scene.hover = null; scene.draw(); };
      rows.push(tr);
    }
  }
  const payload = S.params.m_screen + S.params.m_electronics + S.params.m_gimbal;
  rows.push(h('tr', {},
    h('td', { style: 'border-top:1px solid var(--fg)' }, 'PARTS TOTAL'),
    h('td', { style: 'border-top:1px solid var(--fg)' }, ''),
    h('td', { style: 'border-top:1px solid var(--fg);color:var(--accent)' }, fmt(1000 * total, 4) + ' g'),
    h('td', { style: 'border-top:1px solid var(--fg);color:var(--muted);white-space:normal' },
      `against ${fmt(spec.m, 4)} kg gross from the closure`)));

  v.appendChild(card('Parts list',
    'Hover a row to pick the part out of the drawing.',
    h('div', { style: 'overflow-x:auto' },
      h('table', { class: 'data' },
        h('thead', {}, h('tr', {}, ...['part', 'specification', 'mass', 'note'].map((t) => h('th', {}, t)))),
        h('tbody', {}, rows)))));

  const gap = (S.params.standoff + (S.params.screen_forward || 0)) - reach;
  const tipOk = gap >= S.params.d_safe;
  v.appendChild(h('div', { class: 'grid g3' },
    card('Span', '', stat('tip to tip, folds for transport', fmt(span, 3) + ' m')),
    card('Gross mass', '', stat('closed from geometry', fmt(spec.m, 4) + ' kg')),
    card('Blade tip to your face', '',
      stat(`the machine reaches ${fmt(reach, 3)} m from its centre; safe distance is ${fmt(S.params.d_safe, 3)} m`,
        fmt(gap, 3) + ' m', tipOk ? 'good' : 'bad'))));

  if (!tipOk) {
    v.appendChild(warns([
      `The standoff is measured to the screen, but the rotors reach ${fmt(reach, 3)} m back ` +
      `toward you from the machine's centre, so the nearest blade tip is only ${fmt(gap, 3)} m ` +
      `from your face against a ${fmt(S.params.d_safe, 3)} m safe distance. Stand back to ` +
      `${fmt(reach + S.params.d_safe, 3)} m, which costs apparent screen size and nothing else, ` +
      `or put the screen on a boom, which costs pitch inertia and is the worse trade.`], true));
  }

  const ch = spec.electrical.chosen;
  if (ch) {
    v.appendChild(card('Wiring it up',
      'The two rails, and the one split that is not negotiable.',
      h('div', { class: 'eq' },
        `phone  --bluetooth/wifi-->  Pi 4  --micro HDMI-->  eDP board  --eDP-->  panel\n` +
        `                             |\n` +
        `                             |-- USB --> AI camera (detection on sensor)\n` +
        `                             |\n` +
        `                             '-- UART, MAVLink --> Pixhawk --> 4 ESCs --> motors\n\n` +
        `pack ${ch.S}S (${fmt(ch.v_min, 3)}-${fmt(ch.v_full, 3)} V)  -->  ${ch.v_min < 12 ? 'buck-BOOST' : 'buck'} 12 V  -->  eDP controller board\n` +
        `                       -->  buck  5 V  -->  Pi 4 and Pixhawk`),
      kv([
        ['pack', `${ch.S}S1P ${ch.cell}`, 'accent'],
        ['12 V rail', ch.v_min < 12
          ? `buck-boost needed: the pack crosses 12 V mid-discharge (${fmt(ch.v_min, 3)} to ${fmt(ch.v_full, 3)} V)`
          : `plain buck, pack stays above 12 V (${fmt(ch.v_min, 3)} to ${fmt(ch.v_full, 3)} V)`,
          ch.v_min < 12 ? 'bad' : 'good'],
        ['5 V rail', 'Pi 4 needs 3 A capability'],
        ['motor current', `${fmt(ch.I_hover, 3)} A each at hover, ${fmt(ch.I_max, 3)} A at full thrust`],
        ['ESC', `${Math.ceil(ch.I_max * 2.5 / 5) * 5} A four-in-one is plenty`],
        '-',
        ['flight control', 'Pixhawk, always. Linux is not real time and a stalled video stream must never become a gap in the attitude loop.'],
      ])));
  }

  if (spec.breakdown.warnings && spec.breakdown.warnings.length) {
    v.appendChild(warns(spec.breakdown.warnings));
  }
}

function keyFor(item) {
  const s = item.toLowerCase();
  if (s.includes('arm')) return 'arms';
  if (s.includes('guard')) return 'guards';
  if (s.includes('motor') && !s.includes('gimbal')) return 'motors';
  if (s.includes('propeller')) return 'props';
  if (s.includes('battery')) return 'battery';
  if (s.includes('pi 4') || s.includes('raspberry')) return 'pi';
  if (s.includes('pixhawk')) return 'fc';
  if (s.includes('controller board') || s.includes('edp controller')) return 'driver';
  if (s.includes('regulator')) return 'bec';
  if (s.includes('camera')) return 'camera';
  if (s.includes('gimbal')) return 'gimbal';
  if (s.includes('panel')) return 'screen';
  if (s.includes('centre plate') || s.includes('backing')) return 'hub';
  return item;
}

// ---------------------------------------------------------------------------
// 1. Spec sheet: the whole machine, computed
// ---------------------------------------------------------------------------
// Flying every mission takes about half a minute, so the sheet lands
// immediately without it and the flight table is one click away.
const SPEC = { result: null, fly: false };

function tbl(caption, head, rows) {
  return h('div', { style: 'overflow-x:auto' },
    h('table', { class: 'data' },
      caption ? h('caption', { style: 'text-align:left;color:var(--muted);font-size:10.5px;letter-spacing:.09em;text-transform:uppercase;padding-bottom:6px' }, caption) : null,
      h('thead', {}, h('tr', {}, ...head.map((t) => h('th', {}, t)))),
      h('tbody', {}, rows.map((r) => h('tr', { class: r._c || '' },
        ...r.cells.map((c, i) => h('td', { class: (r.cls && r.cls[i]) || '' },
          c === null || c === undefined ? '--' : String(c))))))));
}

async function renderSpec() {
  const v = view();
  const flyCb = h('input', { type: 'checkbox', ...(SPEC.fly ? { checked: '' } : {}) });
  flyCb.onchange = () => { SPEC.fly = flyCb.checked; };
  // Fitting a different display changes size, mass and power together, which
  // is the whole point: a resolution is not a payload.
  const dispSel = h('select', {
    onchange: (ev) => {
      const c = (S.meta.displays || []).find((x) => x.key === ev.target.value);
      if (!c) return;
      S.controls.setAll({
        screen_w: c.w, screen_h: c.h, m_screen: c.m_screen, P_screen: c.P_screen,
        m_electronics: c.m_electronics, P_computer: c.P_computer, P_sensors: c.P_sensors,
      });
    },
  }, h('option', { value: '' }, 'fit a display...'),
    ...(S.meta.displays || []).map((c) => h('option', { value: c.key },
      `${c.name}  (${fmt(1000 * c.m_screen, 3)} g, ${fmt(c.readability.pixels_per_degree, 2)} px/deg)`)));

  v.appendChild(h('div', { class: 'row' },
    h('button', { class: 'act primary', onclick: () => go() }, 'Recompute'),
    dispSel,
    h('label', { style: 'color:var(--fg)', title: 'Runs a full 6-DOF simulation of all six missions, about half a minute' },
      flyCb, ' fly every mission'),
    h('span', { class: 'note' }, 'Every number below is computed from the assumptions on the left, not stored.')));
  const slot = h('div', {});
  v.appendChild(slot);
  if (SPEC.result) drawSpec(SPEC.result, slot);
  else await go();

  async function go() {
    const token = renderToken;
    const r = await post('spec', {
      params: S.params, fly: SPEC.fly, t_window: 14,
      governor: { v_max: 2.5, a_max: 2.5 },
    }, SPEC.fly ? 'sizing and flying the machine, about half a minute' : 'sizing the machine');
    if (!current(token)) return;          // the user moved on
    SPEC.result = r;
    slot.innerHTML = '';
    drawSpec(r, slot);
  }
}

function drawSpec(r, slot) {
  if (!r.feasible) {
    slot.appendChild(card('This machine does not close', '', h('div', { class: 'note' }, r.reason)));
    return;
  }
  const b = r.breakdown, hov = b.rotor.hover, mx = b.rotor.max;
  const arm = b.arm, pr = b.propulsion, e = r.electrical, ch = e.chosen;
  const dB = b.spl_1m;
  const dBclass = dB <= 52 ? 'good' : dB <= 60 ? '' : 'bad';

  const ac = r.acoustics;
  const atEar = ac ? ac.at_listener.total : dB;
  const earClass = atEar <= 55 ? 'good' : atEar <= 62 ? '' : 'bad';
  slot.appendChild(h('div', { class: 'grid g3' },
    card('Gross mass', '', stat('closed from geometry', fmt(r.m, 4) + ' kg')),
    card('Noise at your ear', '',
      stat(ac ? `in the room at ${fmt(S.params.standoff, 2)} m; ${fmt(dB, 3)} dBA is the free-field figure at 1 m`
        : 'free field at 1 m', fmt(atEar, 3) + ' dBA', earClass)),
    card('Hover power', '', stat('including screen and compute', fmt(b.P_total, 3) + ' W')),
    card('Session', '', stat(r.flights.length ? `worst mission ${fmt(r.session_worst / 60, 2)} min` : 'nominal',
      fmt(r.session_nominal / 60, 2) + ' min')),
    card('Span', '', stat('tip to tip', fmt(b.span, 3) + ' m')),
    card('Wake', '', stat('Beaufort 3 is 3.4 to 5.4 m/s', fmt(b.downwash, 3) + ' m/s',
      b.downwash < 6.5 ? 'good' : 'bad'))));

  // geometry + mass
  const N = S.params.N_rotors;
  slot.appendChild(h('div', { class: 'grid g2' },
    card('Geometry', '', tbl(null, ['item', 'value', 'note'], [
      { cells: ['rotors', N, 'flat X layout'] },
      { cells: ['rotor diameter', fmt(1000 * S.params.D_rotor, 4) + ' mm', fmt(S.params.D_rotor / 0.0254, 3) + ' inch'], cls: ['', 'v accent'] },
      { cells: ['blades per rotor', S.params.n_blades, `${fmt(1000 * hov.chord, 3)} mm chord, AR ${fmt(S.params.blade_AR, 3)}`] },
      { cells: ['solidity', fmt(hov.sigma, 3), `cascade limit ${fmt(S.params.sigma_max, 3)}`] },
      { cells: ['arm length', fmt(1000 * arm.L, 4) + ' mm', arm.driver + ' sized'] },
      { cells: ['span', fmt(b.span, 3) + ' m', 'folds for transport'] },
      { cells: ['screen', `${fmt(1000 * S.params.screen_w, 3)} x ${fmt(1000 * S.params.screen_h, 3)} mm`,
        r.readability ? `${fmt(r.readability.diagonal_in, 3)} inch diagonal` : ''] },
      { cells: ['apparent size', r.readability ? fmt(r.readability.angular_width, 3) + ' deg wide' : '--',
        `at ${fmt(S.params.standoff, 2)} m standoff`] },
      { cells: ['pixel density', r.readability ? fmt(r.readability.pixels_per_degree, 3) + ' px/deg' : '--',
        'sharpness only: says nothing about text size'] },
      { cells: ['16 px text at 100 %', r.readability ? fmt(r.readability.cap_arcmin_at_100, 2) + ' arcmin' : '--',
        'cap height; 16 arcmin minimum, 20 preferred (HFES 100)'] },
      { cells: ['usable desktop', r.readability ? `${fmt(r.readability.logical_w, 3)} x ${fmt(r.readability.logical_h, 3)}` : '--',
        r.readability ? `at ${fmt(100 * r.readability.ui_scale_min, 3)} % scaling: ${r.readability.verdict}` : ''],
        cls: ['', r.readability && r.readability.logical_w >= 1280 ? 'v good' : 'v bad', ''] },
      { cells: ['panel areal density', r.readability ? fmt(r.readability.areal_density, 3) + ' kg/m^2' : '--',
        'laptop 3, tablet 4.5, desktop monitor 10'] },
    ])),
    card('Mass budget', '', tbl(null, ['item', 'mass', 'note'], [
      { cells: ['screen panel + driver', fmt(1000 * S.params.m_screen, 3) + ' g', ''] },
      { cells: ['compute, camera, radio', fmt(1000 * S.params.m_electronics, 3) + ' g', ''] },
      { cells: ['gimbal', fmt(1000 * S.params.m_gimbal, 3) + ' g', ''] },
      { cells: ['fixed subtotal', fmt(1000 * b.m_fix, 3) + ' g', ''], _c: 'sub' },
      { cells: ['arms', fmt(1000 * arm.m_arms, 3) + ' g', `${fmt(2000 * S.params.arm_radius, 3)} mm tube, ${fmt(1000 * arm.wall, 2)} mm wall`] },
      { cells: ['guards', fmt(1000 * arm.m_guards, 3) + ' g', 'full rings'] },
      { cells: ['hub, mounts, wiring', fmt(1000 * (arm.m_frame - arm.m_arms - arm.m_guards), 3) + ' g', ''] },
      { cells: ['frame subtotal', fmt(1000 * b.m_frame, 3) + ' g', `f_s = ${fmt(b.f_s_implied, 3)}`], _c: 'sub' },
      { cells: ['motors', fmt(1000 * pr.m_motors, 3) + ' g', fmt(1000 * pr.m_motor_each, 3) + ' g each'] },
      { cells: ['ESCs', fmt(1000 * pr.m_esc, 3) + ' g', ''] },
      { cells: ['propellers', fmt(1000 * pr.m_props, 3) + ' g', fmt(1000 * pr.m_props / N, 3) + ' g each'] },
      { cells: ['propulsion subtotal', fmt(1000 * b.m_prop, 3) + ' g', `S_m = ${fmt(b.S_m_implied, 3)} N/kg`], _c: 'sub' },
      { cells: ['battery', fmt(1000 * r.m_bat, 3) + ' g', fmt(r.m_bat * S.params.e_b_wh_kg, 3) + ' Wh'] },
      { cells: ['GROSS', fmt(r.m, 4) + ' kg', `FM = ${fmt(b.FM_implied, 3)}`], _c: 'sub', cls: ['', 'v accent', ''] },
    ]))));

  // rotor + power
  slot.appendChild(h('div', { class: 'grid g2' },
    card('Rotor operating point', 'Figure of merit is an output of the blade model, not an input.',
      tbl(null, ['quantity', 'hover', 'full thrust'], [
        { cells: ['thrust per rotor', fmt(hov.T, 3) + ' N', fmt(mx.T, 3) + ' N'] },
        { cells: ['rotor speed', fmt(hov.rpm, 4) + ' rpm', fmt(mx.rpm, 4) + ' rpm'], cls: ['', 'v accent', ''] },
        { cells: ['tip speed', fmt(hov.v_tip, 3) + ' m/s', fmt(mx.v_tip, 3) + ' m/s'], cls: ['', 'v accent', ''] },
        { cells: ['shaft torque', fmt(hov.Q, 3) + ' N.m', fmt(mx.Q, 3) + ' N.m'] },
        { cells: ['blade loading Ct/sigma', fmt(hov.Ct_sigma, 3), fmt(mx.Ct_sigma, 3)] },
        { cells: ['blade Reynolds', fmt(hov.reynolds, 3), fmt(mx.reynolds, 3)] },
        { cells: ['effective Cd0', fmt(hov.cd0_effective, 3), fmt(mx.cd0_effective, 3)] },
        { cells: ['figure of merit', fmt(hov.FM_effective, 3), fmt(mx.FM_effective, 3)] },
      ])),
    card('Power and endurance', '', tbl(null, ['item', 'value', 'note'], [
      { cells: ['ideal induced', fmt(N * hov.P_ideal, 3) + ' W', 'momentum theory'] },
      { cells: ['induced with tip loss', fmt(N * hov.P_induced, 3) + ' W', `kappa = ${fmt(S.params.kappa_ind, 3)}`] },
      { cells: ['profile', fmt(N * hov.P_profile, 3) + ' W', 'blade drag'] },
      { cells: ['shaft', fmt(N * hov.P_shaft, 3) + ' W', ''] },
      { cells: ['electrical, rotors', fmt(N * hov.P_elec, 3) + ' W', `eta = ${fmt(S.params.eta, 3)}`] },
      { cells: ['screen, compute, sensors', fmt(b.P_aux, 3) + ' W', ''] },
      { cells: ['TOTAL HOVER', fmt(b.P_total, 4) + ' W', ''], _c: 'sub', cls: ['', 'v accent', ''] },
      { cells: ['pack energy usable', fmt(r.m_bat * r.derived.e_b / 3600, 3) + ' Wh', `to ${fmt(100 * S.params.soc_reserve, 2)}% reserve`] },
      { cells: ['session, hovering', fmt(r.session_nominal / 60, 3) + ' min', ''] },
      { cells: ['session, worst mission', fmt(r.session_worst / 60, 3) + ' min', `at ${fmt(r.worst_power, 4)} W`], cls: ['', 'v accent', ''] },
    ]))));

  // electrical chain
  if (ch) {
    slot.appendChild(card('Electrical chain',
      'Torque sets motor size, but voltage and KV decide whether it can be driven. KV is sized so full rotor speed is reached at the <b>minimum</b> pack voltage, so the machine still hovers on a tired battery.',
      tbl(null, ['parameter', 'value', 'against'], [
        { cells: ['pack', `${ch.S}S1P ${ch.cell}`, `${fmt(1000 * ch.mass, 3)} g, ${fmt(ch.energy_wh, 3)} Wh`], cls: ['', 'v accent', ''] },
        { cells: ['pack voltage', `${fmt(ch.v_min, 3)} to ${fmt(ch.v_full, 3)} V`, `${fmt(ch.v_nom, 3)} V nominal`] },
        { cells: ['motor constant', fmt(ch.KV, 3) + ' KV', `Kt = ${fmt(ch.Kt, 3)} N.m/A`], cls: ['', 'v accent', ''] },
        { cells: ['throttle at hover', `${fmt(100 * ch.throttle_at_min, 3)} %`, 'at the minimum pack voltage'] },
        { cells: ['current at hover', fmt(ch.I_hover, 3) + ' A', `${fmt(N * ch.I_hover, 3)} A total`] },
        { cells: ['current at full thrust', fmt(ch.I_max, 3) + ' A', 'per channel, sets the ESC'] },
        { cells: ['continuous discharge', fmt(ch.cont_w_kg, 4) + ' W/kg', `cell rated ${fmt(ch.cont_limit, 4)}`], cls: ['', ch.cont_w_kg < ch.cont_limit ? 'v good' : 'v bad', ''] },
        { cells: ['peak discharge', fmt(ch.burst_w_kg, 4) + ' W/kg', `cell rated ${fmt(ch.burst_limit, 4)}`], cls: ['', ch.burst_w_kg < ch.burst_limit ? 'v good' : 'v bad', ''] },
        ...(ch.phase_resistance || []).map((x) => ({
          cells: [`phase resistance at ${fmt(100 * x.eta, 2)}% motor efficiency`,
            fmt(x.r_ohm, 3) + ' ohm', fmt(x.loss_w, 3) + ' W lost per motor'],
        })),
      ])),
      e.note ? warns([e.note]) : null);
  }

  // acoustics, written out
  if (ac) {
    const s = ac.source;
    slot.appendChild(card('Acoustics, term by term',
      'Broadband rotor noise goes as the sixth power of tip speed, which is why the whole machine is large and slow. The room is what the free-field figure leaves out.',
      h('div', { class: 'eq' },
        `L_p(1 m) = C + 60 log10(V_tip/100) + 10 log10(T/5) + 10 log10(N)\n` +
        `         = ${fmt(s.anchor, 4)} ${s.tip_term >= 0 ? '+' : ''}${fmt(s.tip_term, 3)} ` +
        `${s.thrust_term >= 0 ? '+' : ''}${fmt(s.thrust_term, 3)} ` +
        `${s.count_term >= 0 ? '+' : ''}${fmt(s.count_term, 3)} = ${fmt(s.spl_1m, 4)} dBA\n\n` +
        `L_p(r)   = L_W + 10 log10( Q/(4 pi r^2) + 4/R )     R = S a / (1 - a)`),
      tbl(null, ['quantity', 'value', 'note'], [
        { cells: ['anchor C', fmt(s.anchor, 4) + ' dB', 'calibrated on four measured aircraft'] },
        { cells: ['tip speed term', fmt(s.tip_term, 3) + ' dB', `at ${fmt(s.v_tip, 3)} m/s against 100`], cls: ['', 'v accent', ''] },
        { cells: ['thrust term', fmt(s.thrust_term, 3) + ' dB', `${fmt(s.T_rotor, 3)} N per rotor against 5`] },
        { cells: ['rotor count term', fmt(s.count_term, 3) + ' dB', `${s.N} incoherent sources`] },
        { cells: ['free field at 1 m', fmt(s.spl_1m, 4) + ' dBA', 'the figure usually quoted'] },
        { cells: ['sound power L_WA', fmt(ac.L_WA, 4) + ' dBA', 'what a datasheet declares'] },
        { cells: ['room constant R', fmt(ac.room.R, 3) + ' m^2', `${fmt(ac.room.surface_area, 4)} m^2 of surface at alpha ${fmt(ac.room.alpha_bar, 2)}`] },
        { cells: ['critical distance', fmt(ac.critical_distance, 3) + ' m', 'past this, distance stops helping'] },
        { cells: ['AT YOUR EAR', fmt(ac.at_listener.total, 4) + ' dBA',
          `direct ${fmt(ac.at_listener.direct, 3)}, reverberant ${fmt(ac.at_listener.reverberant, 3)}`],
          _c: 'sub', cls: ['', 'v ' + (earClass === 'good' ? 'good' : earClass === 'bad' ? 'bad' : 'accent'), ''] },
        { cells: ['anywhere else in the room', fmt(ac.reverberant_level, 4) + ' dBA', 'does not fall off with distance'] },
        { cells: ['blade passage', fmt(ac.bpf, 3) + ' Hz', `A-weighting discounts it by ${fmt(-ac.harmonics[0].a_weight, 3)} dB`] },
      ]),
      h('h3', { style: 'margin-top:22px' }, 'What the room finish is worth at your ear'),
      tbl(null, ['room', 'level'], Object.entries(ac.absorption_options || {}).map(([k, val]) => ({
        cells: [k, fmt(val, 4) + ' dBA'],
      }))),
      warns(ac.notes)));
  }

  // rotor aerodynamics beyond the plain actuator disk
  if (r.surface) {
    const su = r.surface, it = r.interference, rc = r.recirculation;
    slot.appendChild(card('Beyond the actuator disk',
      'Momentum theory assumes an isolated rotor in still air. Three of the ways that is untrue are now modelled; the fourth is measured and left as a warning.',
      tbl(null, ['effect', 'factor', 'note'], [
        { cells: ['ground effect', 'x' + fmt(su.k_ground, 5),
          `rotor plane ${fmt(su.z_floor, 3)} m above the floor, ${fmt(100 * (1 - 1 / su.k_ground), 2)}% less induced power`] },
        { cells: ['ceiling effect', 'x' + fmt(su.k_ceiling, 5),
          `${fmt(su.z_ceiling, 3)} m below a ${fmt(S.params.ceiling_height, 3)} m ceiling, ${fmt(100 * (1 - 1 / su.k_ceiling), 2)}% less`] },
        { cells: ['rotor interference', 'x' + fmt(it.kappa_int, 5),
          `tip-to-tip gap ${fmt(1000 * it.gap, 3)} mm = ${fmt(it.gap_ratio, 3)} D, ${fmt(100 * (it.kappa_int - 1), 2)}% more induced power`],
          cls: ['', it.kappa_int > 1.03 ? 'v bad' : '', ''] },
        { cells: ['net on induced power', 'x' + fmt(hov.kappa_total / S.params.kappa_ind, 5), 'the two largely cancel here'] },
        { cells: ['air moved', fmt(rc.volume_flow, 3) + ' m^3/s', `through a ${fmt(rc.room_volume, 4)} m^3 room`] },
        { cells: ['room air turnover', fmt(rc.exchange_time, 3) + ' s',
          'after this the machine is flying in its own wake'], cls: ['', 'v bad', ''] },
      ]),
      warns([`Recirculation is not modelled. At ${fmt(rc.volume_flow, 3)} m^3/s the machine passes ` +
        `every cubic metre of air in the room through its disk every ${fmt(rc.exchange_time, 2)} seconds, ` +
        `so the still, undisturbed inflow that momentum theory assumes has stopped being true within a minute. ` +
        `That is the largest remaining aerodynamic gap, along with the absence of blade element momentum theory.`])));
  }

  // control
  const a = r.authority, g = r.suggested_gains;
  slot.appendChild(h('div', { class: 'grid g2' },
    card('Control authority',
      'Yaw comes from rotor reaction torque, which is exactly what a quiet slow rotor has least of.',
      tbl(null, ['axis', 'torque', 'acceleration'], [
        { cells: ['roll', fmt(a.tau_roll, 3) + ' N.m', fmt(a.alpha_roll, 3) + ' rad/s^2'] },
        { cells: ['pitch', fmt(a.tau_pitch, 3) + ' N.m', fmt(a.alpha_pitch, 3) + ' rad/s^2'] },
        { cells: ['yaw', fmt(a.tau_yaw, 3) + ' N.m', fmt(a.alpha_yaw, 3) + ' rad/s^2'],
          cls: ['', '', a.alpha_yaw < 3 ? 'v bad' : ''] },
      ]),
      a.alpha_yaw < 3 ? warns(['Yaw authority is ' + fmt(a.alpha_yaw, 2) +
        ' rad/s^2 against ' + fmt(a.alpha_pitch, 2) + ' in pitch. The airframe cannot turn to face '
        + 'the user quickly: put yaw on the gimbal and let the airframe keep any heading it likes.']) : null),
    card('Gains and structure', '', tbl(null, ['item', 'in use', 'sized to the machine'], [
      { cells: ['attitude gain K_R', fmt(S.params.K_R, 3), fmt(g.K_R, 3)],
        cls: ['', S.params.K_R > 1.3 * g.K_R ? 'v bad' : 'v good', ''] },
      { cells: ['rate gain K_w', fmt(S.params.K_w, 3), fmt(g.K_w, 3)] },
      { cells: ['position Kp', fmt(S.params.Kp_pos, 3), fmt(g.Kp_pos, 3)] },
      { cells: ['position Kd', fmt(S.params.Kd_pos, 3), fmt(g.Kd_pos, 3)] },
      { cells: ['Jxx / Jyy / Jzz', `${fmt(r.inertia.Jxx, 3)} / ${fmt(r.inertia.Jyy, 3)} / ${fmt(r.inertia.Jzz, 3)}`, 'kg m^2'] },
      { cells: ['arm working stress', fmt(arm.sigma / 1e6, 3) + ' MPa', `allowable ${fmt(S.params.sigma_allow, 4)}`] },
      { cells: ['arm tip deflection', fmt(1000 * arm.tip_deflection, 3) + ' mm', `at ${fmt(S.params.SF_struct, 2)}x limit load`] },
      { cells: ['first bending mode', fmt(arm.f_bending, 3) + ' Hz', `attitude loop ${fmt(Math.sqrt(S.params.K_R) / 2 / Math.PI, 3)} Hz`],
        cls: ['', arm.f_bending > 25 ? 'v good' : 'v bad', ''] },
    ]),
      h('div', { class: 'row', style: 'margin-top:10px' },
        h('button', {
          class: 'act', onclick: () => S.controls.setAll({
            K_R: g.K_R, K_w: g.K_w, Kp_pos: g.Kp_pos, Kd_pos: g.Kd_pos, Ki_pos: g.Ki_pos,
          }),
        }, 'Adopt the sized gains')))));

  // flights
  if (r.flights && r.flights.length) {
    slot.appendChild(card('Flown, every mission',
      `Reference governor at ${fmt(r.governor.v_max, 2)} m/s and ${fmt(r.governor.a_max, 2)} m/s^2. Lag is what the reader sees; the governor deliberately lets it grow during a turn rather than whipping the display around a head.`,
      tbl(null, ['mission', 'power', 'lag rms', 'track max', 'screen tilt', 'airframe', 'saturation', 'rotor to eye', 'constraints'],
        r.flights.map((f) => ({
          cells: f.ok
            ? [f.mission.replace('_', ' '), fmt(f.P_mean, 4) + ' W',
              fmt(1000 * f.e_lag_rms, 3) + ' mm', fmt(1000 * f.e_track_max_all, 3) + ' mm',
              fmt(f.screen_tilt_max_deg, 3) + ' deg', fmt(f.tilt_max_deg, 3) + ' deg',
              fmt(100 * f.saturation_fraction, 2) + ' %', fmt(f.rotor_clearance_min, 3) + ' m',
              f.constraints_ok ? 'met' : (f.violations || []).join('; ')]
            : [f.mission, 'FAILED', f.reason, '', '', '', '', '', ''],
          cls: ['', '', '', f.ok && !f.margin_holds ? 'v bad' : '',
            f.ok && f.screen_tilt_max_deg > S.params.theta_readable ? 'v bad' : '',
            '', f.ok && f.saturation_fraction > 0.1 ? 'v bad' : '',
            f.ok && f.rotor_clearance_min < S.params.d_safe ? 'v bad' : '',
            f.ok ? (f.constraints_ok ? 'v good' : 'v bad') : ''],
        })))));
  }

  // tether comparison
  if (r.tether && r.tether.feasible) {
    const t = r.tether;
    slot.appendChild(card('The same airframe on a wire',
      'At this power the conductor is set by handling, not resistance, so the tether is nearly free and the machine gets lighter and quieter.',
      tbl(null, ['', 'roam, on the battery', 'desk, on the tether'], [
        { cells: ['energy carried', fmt(1000 * r.m_bat, 3) + ' g of cells',
          `${fmt(1000 * t.m_reserve, 3)} g reserve + ${fmt(1000 * t.m_tether_carried, 3)} g of cable`] },
        { cells: ['gross mass', fmt(r.m, 4) + ' kg', fmt(t.m, 4) + ' kg'], cls: ['', '', 'v good'] },
        { cells: ['hover power', fmt(b.P_total, 4) + ' W', fmt(t.P_bus, 4) + ' W'], cls: ['', '', 'v good'] },
        { cells: ['noise at 1 m', fmt(b.spl_1m, 3) + ' dBA', fmt(t.spl_1m, 3) + ' dBA'], cls: ['', '', 'v good'] },
        { cells: ['wake', fmt(b.downwash, 3) + ' m/s', fmt(t.downwash, 3) + ' m/s'] },
        { cells: ['tether', '', `${fmt(t.conductor_area_mm2, 3)} mm^2 per leg, ${fmt(1000 * t.m_tether_total, 3)} g for ${fmt(S.params.tether_len, 3)} m`] },
        { cells: ['current and loss', '', `${fmt(t.current, 3)} A at ${fmt(S.params.tether_V, 4)} V, ${fmt(t.P_loss, 3)} W lost`] },
        { cells: ['endurance', fmt(r.session_worst / 60, 3) + ' min', 'unlimited'], cls: ['', '', 'v good'] },
      ])));
  }

  if (b.warnings && b.warnings.length) slot.appendChild(warns(b.warnings));
}

// ---------------------------------------------------------------------------
// 1. Closure
// ---------------------------------------------------------------------------
async function renderClosure() {
  const d = await design(true);
  const dp = d.design, c = dp.coeffs || {}, cur = d.curve, wall = d.wall;
  const feasible = dp.feasible;
  const v = view();

  const modelSel = h('select', {
    onchange: (e) => { S.model = e.target.value; S.cache = {}; show('closure', true); },
  }, ...['fixed_area', 'constant_dl', 'power_law'].map((m) =>
    h('option', { value: m, selected: S.model === m ? '' : null },
      { fixed_area: 'Fixed rotor area  (p = 0)', constant_dl: 'Constant disk loading  (p = 1)', power_law: 'Power law  A ~ m^p' }[m])));

  v.appendChild(h('div', { class: 'row' },
    h('label', {}, 'sizing model'), modelSel,
    h('span', { class: feasible ? 'pill ok' : 'pill no' }, feasible ? 'design exists' : 'no solution'),
    dp.stable === false ? h('span', { class: 'pill warn' }, 'unstable fixed point') : null));

  const eq = (S.model === 'constant_dl')
    ? 'm = M0 / (1 - f_s - gamma_m - gamma_b)'
    : `alpha m - beta m^${fmt(cur.q, 3)} = M0`;

  const left = card('Mass closure',
    'The supportable fixed load as a function of gross mass. Where the curve crosses M0 the design closes; the peak is the wall.',
    h('div', { class: 'eq' }, eq),
    cv('tall'));

  const rows = [
    ['alpha  = 1 - f_s - lambda g / S_m', fmt(c.alpha, 4), 'accent'],
    [`beta   (coefficient of m^${fmt(cur.q, 3)})`, fmt(cur.beta, 4), 'accent'],
    ['M0     = m_fix + P_aux t_f / e_b', fmt(cur.M0, 4) + ' kg', 'accent'],
    '-',
    ['fixed mass m_fix', fmt(d.derived.m_fix, 4) + ' kg'],
    ['auxiliary power P_aux', fmt(d.derived.P_aux, 4) + ' W'],
    ['battery just for P_aux', fmt(c.m_bat_aux, 4) + ' kg'],
    ['usable specific energy', fmt(d.derived.e_b / 3600, 4) + ' Wh/kg'],
    ['total disk area A', fmt(d.derived.A, 4) + ' m^2'],
  ];
  if (cur.has_fold) {
    rows.push('-',
      ['fold mass m* = 4 alpha^2 / 9 beta^2', fmt(cur.m_star, 4) + ' kg'],
      ['wall M0_max = 4 alpha^3 / 27 beta^2', fmt(cur.M0_max, 4) + ' kg', feasible ? 'good' : 'bad'],
      ['utilisation M0 / M0_max', fmt(wall.utilisation, 4), wall.utilisation > 1 ? 'bad' : 'good']);
  }
  const right = card('Coefficients', 'Every number here comes from the assumptions on the left.', kv(rows));

  v.appendChild(h('div', { class: 'grid g2' }, left, right));

  draw(() => linechart(left.querySelector('canvas'), {
    series: [{ x: cur.m, y: cur.F, label: 'alpha m - beta m^q', color: PALETTE[0], width: 2 }],
    hlines: [{ y: cur.M0, label: `M0 = ${fmt(cur.M0, 3)} kg`, color: PALETTE[1] }],
    vlines: cur.m_star ? [{ x: cur.m_star, label: `m* = ${fmt(cur.m_star, 3)} kg`, color: '#8a94a3', row: 1 }] : [],
    markers: [
      cur.m_light !== null && cur.m_light !== undefined ?
        { x: cur.m_light, y: cur.M0, label: `design ${fmt(cur.m_light, 3)} kg`, color: PALETTE[2] } : null,
      cur.m_heavy ? { x: cur.m_heavy, y: cur.M0, label: `unstable ${fmt(cur.m_heavy, 3)} kg`, color: PALETTE[3] } : null,
    ].filter(Boolean),
    xlabel: 'gross mass m  [kg]', ylabel: 'supportable fixed load  [kg]', legend: false,
  }));

  if (!feasible) {
    v.appendChild(card('Why there is no solution', '',
      h('div', { class: 'note', html:
        `<b>${dp.reason}</b><br><br>
         The fixed load you are asking for is ${fmt(cur.M0, 3)} kg. The most this rotor system can
         support at ${secs(S.params.t_f)} of endurance is ${fmt(cur.M0_max, 3)} kg, reached at a gross
         mass of ${fmt(cur.m_star, 3)} kg. Past that point every extra kilogram of aircraft needs more
         than a kilogram of battery to fly it, so the two roots of the closure have merged and vanished.
         This is a fold, not an inconvenience: no amount of iteration finds a design.<br><br>
         The levers, in order of how hard they pull: endurance (squared), battery energy density
         (squared), figure of merit and electrical efficiency (squared), and disk area (linear, so
         rotor diameter squared).`}),
      h('div', { class: 'row', style: 'margin-top:12px' },
        h('button', { class: 'act', onclick: () => S.controls.setAll({ t_f: Math.max(120, S.params.t_f / 2) }) }, 'halve the endurance'),
        h('button', { class: 'act', onclick: () => S.controls.setAll({ D_rotor: Math.min(2, S.params.D_rotor * 1.5) }) }, 'grow the rotors 50%'),
        h('button', { class: 'act', onclick: () => S.controls.setAll({ m_screen: Math.max(0.02, S.params.m_screen / 2) }) }, 'halve the screen'),
        h('button', { class: 'act', onclick: () => show('tether', true) }, 'try a tether'))));
    return;
  }

  // mass and power budgets
  const massCard = card('Mass budget',
    'The lumped closure on the left of each bar, the first-principles component build-up on the right.',
    cv('stackbar'));
  const powerCard = card('Power budget', 'Where the watts go in steady hover.', cv('short'));
  const perfCard = card('Hover performance', '', kv([
    ['gross mass', fmt(dp.m, 4) + ' kg', 'accent'],
    ['hover electrical power', fmt(dp.P_total, 4) + ' W', 'accent'],
    ['disk loading', fmt(dp.disk_loading, 4) + ' N/m^2'],
    ['induced velocity', fmt(dp.v_induced, 3) + ' m/s'],
    ['wake velocity (downwash)', fmt(2 * dp.v_induced, 3) + ' m/s'],
    ['power loading', fmt(dp.power_loading, 3) + ' N/W'],
    ['specific power', fmt(dp.hover_specific_power, 4) + ' W/kg'],
    ['thrust per rotor at hover', fmt(dp.T_hover_per_rotor, 3) + ' N'],
    ['maximum total thrust', fmt(dp.T_max, 3) + ' N'],
    '-',
    ['battery sizing driver', dp.battery_limit],
    ['d(m_str+m_prop+m_bat)/dm', fmt(dp.dmdm, 3) + '  (needs < 1)', dp.dmdm < 1 ? 'good' : 'bad'],
    ['second root of the closure', dp.closure.m_heavy ? fmt(dp.closure.m_heavy, 4) + ' kg' : 'none'],
  ]));

  v.appendChild(h('div', { class: 'grid g2' }, massCard, perfCard));
  v.appendChild(h('div', { class: 'grid g2' }, powerCard,
    card('First principles check',
      'The same design sized from rotor geometry, beam bending and motor torque instead of f_s, S_m and FM.',
      d.first_principles.feasible ? kv([
        ['gross mass', fmt(d.first_principles.m, 4) + ' kg', 'accent'],
        ['implied f_s', `${fmt(d.first_principles.breakdown.f_s_implied, 3)}  (assumed ${fmt(S.params.f_s, 3)})`],
        ['implied S_m', `${fmt(d.first_principles.breakdown.S_m_implied, 4)}  (assumed ${fmt(S.params.S_m, 4)}) N/kg`],
        ['implied FM', `${fmt(d.first_principles.breakdown.FM_implied, 3)}  (assumed ${fmt(S.params.FM, 3)})`],
        ['span tip to tip', fmt(d.first_principles.breakdown.span, 3) + ' m'],
        ['noise at 1 m', fmt(d.first_principles.breakdown.spl_1m, 3) + ' dB'],
      ]) : h('div', { class: 'note' }, d.first_principles.reason))));

  draw(() => stack(massCard.querySelector('canvas'), {
    parts: [
      { label: 'fixed', value: dp.m_fix, color: PALETTE[0] },
      { label: 'structure', value: dp.m_str, color: PALETTE[1] },
      { label: 'propulsion', value: dp.m_prop, color: PALETTE[2] },
      { label: 'battery', value: dp.m_bat, color: PALETTE[3] },
    ],
  }));
  draw(() => bars(powerCard.querySelector('canvas'), {
    items: [
      { label: 'rotors', value: dp.P_prop, color: PALETTE[0] },
      { label: 'screen', value: S.params.P_screen, color: PALETTE[1] },
      { label: 'computer', value: S.params.P_computer, color: PALETTE[2] },
      { label: 'sensors', value: S.params.P_sensors, color: PALETTE[3] },
    ],
    format: (x) => fmt(x, 4) + ' W', rowHeight: 30,
  }));

  if (dp.warnings && dp.warnings.length) v.appendChild(warns(dp.warnings));
}

// ---------------------------------------------------------------------------
// 2. The wall
// ---------------------------------------------------------------------------
async function renderWall() {
  const w = await post('wall', { params: S.params }, 'mapping the wall');
  const fp = await post('fixed_point', { params: S.params, m0: Math.max(0.2, S.params.m_screen) }, 'iterating');
  const v = view();
  const wall = w.wall, b = w.boundary;

  const top = h('div', { class: 'grid g3' },
    card('Fixed load asked for', '', stat('M0 = m_fix + P_aux t_f / e_b', fmt(wall.M0, 3) + ' kg')),
    card('Most the design can carry', '', stat('M0_max = 4 alpha^3 / 27 beta^2',
      fmt(wall.M0_max, 3) + ' kg', wall.M0_max > wall.M0 ? 'good' : 'bad')),
    card('Screen budget left', '', stat('after fixed electronics and the auxiliary battery',
      fmt(wall.m_screen_max, 3) + ' kg', wall.m_screen_max > S.params.m_screen ? 'good' : 'bad')));
  v.appendChild(top);

  const c1 = card('The wall against endurance',
    'Fixed disk area. The screen budget falls as one over the square of the mission time, so four times the flight time costs sixteen times the payload.',
    cv('tall'));
  const c2 = card('Sensitivity of the wall',
    'Logarithmic derivatives d ln M0_max / d ln x. These exponents are the whole argument: energy density and efficiency enter squared, disk area linearly, endurance squared and negative.',
    cv('tall'));
  v.appendChild(h('div', { class: 'grid g2' }, c1, c2));

  draw(() => linechart(c1.querySelector('canvas'), {
    series: [
      { x: b.t, y: b.M0_max, label: 'M0_max', color: PALETTE[0] },
      { x: b.t, y: b.m_screen_max.map((x) => (x > 0 ? x : null)), label: 'screen budget', color: PALETTE[2] },
    ],
    xlog: true, ylog: true, xlabel: 'endurance t_f  [s]', ylabel: 'mass  [kg]',
    xfmt: (t) => (t >= 3600 ? `${fmt(t / 3600, 2)} h` : `${fmt(t / 60, 2)} m`),
    vlines: [{ x: S.params.t_f, label: 'you are here', color: PALETTE[1] }],
    hlines: [{ y: S.params.m_screen, label: `screen ${fmt(S.params.m_screen, 3)} kg`, color: PALETTE[3] }],
  }));

  const order = Object.entries(w.sensitivities).sort((a, b2) => Math.abs(b2[1]) - Math.abs(a[1]));
  const labels = Object.fromEntries(S.schema.map((s) => [s.key, s.label]));
  draw(() => bars(c2.querySelector('canvas'), {
    items: order.map(([k, val]) => ({
      label: labels[k] || k, value: val,
      color: val > 0 ? PALETTE[2] : PALETTE[3],
    })),
    signed: true, labelWidth: 150, rowHeight: 24,
    format: (x) => (x >= 0 ? '+' : '') + fmt(x, 3),
  }));

  const c3 = card('Naive iteration: does the mass spiral converge?',
    `m_{k+1} = m_fix + f_s m_k + gamma_m m_k + m_bat(m_k). Status: <b>${fp.status}</b>${fp.rate !== null && fp.rate !== undefined ? `, contraction factor ${fmt(fp.rate, 3)}` : ''}.
     Near the fold the map barely contracts, so convergence becomes very slow just before it stops existing altogether.`,
    cv('mid'));
  v.appendChild(c3);
  const hist = fp.history.map((x, i) => [i, x]).filter(([, x]) => x !== null && isFinite(x));
  draw(() => linechart(c3.querySelector('canvas'), {
    series: [{ x: hist.map((p) => p[0]), y: hist.map((p) => p[1]), color: fp.status === 'diverging' ? PALETTE[3] : PALETTE[2], type: 'line' },
    { x: hist.map((p) => p[0]), y: hist.map((p) => p[1]), color: fp.status === 'diverging' ? PALETTE[3] : PALETTE[2], type: 'dots', r: 2 }],
    xlabel: 'iteration k', ylabel: 'gross mass estimate  [kg]', legend: false,
    hlines: fp.m_final && fp.status !== 'diverging' ? [{ y: fp.m_final, label: `fixed point ${fmt(fp.m_final, 4)} kg` }] : [],
  }));
}

// ---------------------------------------------------------------------------
// 3. Feasibility map
// ---------------------------------------------------------------------------
const MAP_STATE = { xkey: 't_f', ykey: 'm_screen', metric: 'm', fp: false, n: 46 };

async function renderMap() {
  const v = view();
  const numeric = S.schema.filter((s) => !s.boolean);
  const sel = (val, onchange, list) => h('select', { onchange },
    ...list.map(([k, label]) => h('option', { value: k, selected: k === val ? '' : null }, label)));
  const keyList = numeric.map((s) => [s.key, `${s.group}: ${s.label}`]);

  const controls = h('div', { class: 'row' },
    h('label', {}, 'x'), sel(MAP_STATE.xkey, (e) => { MAP_STATE.xkey = e.target.value; show('map', true); }, keyList),
    h('label', {}, 'y'), sel(MAP_STATE.ykey, (e) => { MAP_STATE.ykey = e.target.value; show('map', true); }, keyList),
    h('label', {}, 'colour'), sel(MAP_STATE.metric, (e) => { MAP_STATE.metric = e.target.value; show('map', true); },
      Object.entries(S.meta.metrics)),
    h('label', {}, 'resolution'), sel(String(MAP_STATE.n), (e) => { MAP_STATE.n = +e.target.value; show('map', true); },
      [['30', 'coarse'], ['46', 'medium'], ['70', 'fine']].map(([a, b2]) => [a, b2])),
    h('label', {}, h('input', {
      type: 'checkbox', ...(MAP_STATE.fp ? { checked: '' } : {}),
      onchange: (e) => { MAP_STATE.fp = e.target.checked; show('map', true); },
    }), ' first-principles closure'));
  v.appendChild(controls);

  const sx = S.schema.find((s) => s.key === MAP_STATE.xkey);
  const sy = S.schema.find((s) => s.key === MAP_STATE.ykey);
  const axis = (s) => ({
    key: s.key, lo: s.lo, hi: s.hi, n: MAP_STATE.n, log: !!s.log,
  });
  const g = await post('sweep', {
    params: S.params, x: axis(sx), y: axis(sy),
    metric: MAP_STATE.metric, model: S.model,
    first_principles: MAP_STATE.fp,
  }, 'sweeping the design space');

  const total = g.feasible.length * g.feasible[0].length;
  const nf = g.feasible.reduce((a, r) => a + r.reduce((x, y) => x + y, 0), 0);

  const c = card(`${g.metric_label} over ${sx.label} and ${sy.label}`,
    `Hatched cells have no solution at all. ${nf} of ${total} cells close.` +
    (g.analytic_boundary ? ' The dashed line is the closed-form wall, drawn from the algebra rather than the sweep.' : ''),
    cv('tall'));
  v.appendChild(c);
  const cvs = c.querySelector('canvas');
  cvs.style.height = '460px';

  draw(() => heatmap(cvs, {
    x: g.x, y: g.y, z: g.z, feasible: g.feasible,
    xlabel: `${g.x_label}  [${g.x_unit}]`, ylabel: `${g.y_label}  [${g.y_unit}]`,
    xlog: !!sx.log, ylog: !!sy.log, log: true,
    xfmt: sx.key === 't_f' ? (t) => (t >= 3600 ? `${fmt(t / 3600, 2)} h` : `${fmt(t / 60, 2)} m`) : undefined,
    overlay: g.analytic_boundary ? { x: g.analytic_boundary.t, y: g.analytic_boundary.m_screen_max } : null,
    point: { x: S.params[MAP_STATE.xkey], y: S.params[MAP_STATE.ykey] },
  }));

  cvs.onclick = (ev) => {
    const r = cvs.getBoundingClientRect();
    const px = ev.clientX - r.left, py = ev.clientY - r.top;
    const pad = { l: 66, r: 74, t: 14, b: 42 };
    const iw = r.width - pad.l - pad.r, ih = r.height - pad.t - pad.b;
    if (px < pad.l || px > pad.l + iw || py < pad.t || py > pad.t + ih) return;
    const fx = (px - pad.l) / iw, fy = (pad.t + ih - py) / ih;
    const inv = (s, f) => s.log
      ? s.lo * Math.pow(s.hi / s.lo, f) : s.lo + (s.hi - s.lo) * f;
    S.controls.setAll({ [sx.key]: inv(sx, fx), [sy.key]: inv(sy, fy) });
  };
  v.appendChild(h('div', { class: 'note' }, 'Click anywhere on the map to move the design point there.'));

  if (Object.keys(g.reasons || {}).length) {
    v.appendChild(card('Why cells fail', '',
      kv(Object.entries(g.reasons).map(([r, n]) => [r, `${n} cells`]))));
  }
}

// ---------------------------------------------------------------------------
// 4. Rotor scaling
// ---------------------------------------------------------------------------
async function renderScaling() {
  const sc = await post('scaling', { params: S.params }, 'sweeping the exponent');
  const v = view();
  const pts = sc.points;

  v.appendChild(card('What the disk-area exponent decides',
    `If rotor area grows with gross mass as A ~ m^p, then hover power goes as m^((3-p)/2).
     At p = 0 the rotors are fixed and power grows as m^1.5, which is the fold.
     At p = 1 disk loading is constant, power is linear in mass, and the fold is gone.
     Every curve here is anchored at the same physical rotor at ${fmt(sc.anchor, 3)} kg,
     so they differ only in what happens when more endurance is asked for.`,
    h('div', { class: 'eq' }, 'P  ~  m^((3 - p) / 2)          fold exists only while (3 - p) / 2 > 1, that is p < 1')));

  const c1 = card('Endurance wall against the exponent',
    'The longest mission that still closes. It runs away to infinity exactly at p = 1.', cv('tall'));
  const c2 = card('Gross mass at multiples of the current endurance',
    'Gaps are where the design stops existing.', cv('tall'));
  v.appendChild(h('div', { class: 'grid g2' }, c1, c2));

  const finite = pts.map((p) => (p.t_wall === null || p.t_wall === undefined ? null : p.t_wall));
  draw(() => linechart(c1.querySelector('canvas'), {
    series: [{ x: pts.map((p) => p.p), y: finite, color: PALETTE[0], width: 2 }],
    ylog: true, xlabel: 'disk area exponent p', ylabel: 'endurance wall  [s]', legend: false,
    vlines: [{ x: 1.0, label: 'p = 1, constant disk loading', color: PALETTE[2] },
    { x: S.params.area_p, label: 'current p', color: PALETTE[1], row: 1 }],
    hlines: [{ y: S.params.t_f, label: 'asked for', color: PALETTE[3] }],
  }));

  draw(() => linechart(c2.querySelector('canvas'), {
    series: sc.factors.map((f, i) => ({
      x: pts.map((p) => p.p),
      y: pts.map((p) => (p.endurance_scan[i] ? p.endurance_scan[i].m : null)),
      label: `${f}x  (${secs(sc.t_f * f)})`, color: PALETTE[i],
    })),
    ylog: true, xlabel: 'disk area exponent p', ylabel: 'gross mass  [kg]',
    vlines: [{ x: 1.0, color: PALETTE[2], label: 'p = 1' }],
  }));

  const rows = pts.filter((_, i) => i % 3 === 0);
  v.appendChild(card('The cost of escaping the fold',
    'Growing the disk faster than the mass removes the energy wall, but the rotor diameter it asks for is what then becomes unreasonable next to a person.',
    h('table', { class: 'data' },
      h('thead', {}, h('tr', {}, ...['p', 'q = (3-p)/2', 'fold', 'gross mass', 'rotor D', 'disk loading', 'endurance wall']
        .map((t) => h('th', {}, t)))),
      h('tbody', {}, rows.map((p) => h('tr', {},
        h('td', {}, fmt(p.p, 3)), h('td', {}, fmt(p.q, 3)),
        h('td', {}, p.has_fold ? 'yes' : 'no'),
        h('td', {}, p.m ? fmt(p.m, 4) + ' kg' : '--'),
        h('td', {}, p.D_rotor ? fmt(p.D_rotor, 3) + ' m' : '--'),
        h('td', {}, p.disk_loading ? fmt(p.disk_loading, 4) : '--'),
        h('td', {}, p.t_wall ? secs(p.t_wall) : 'none')))))));
}

// ---------------------------------------------------------------------------
// 5. Machine
// ---------------------------------------------------------------------------
async function renderMachine() {
  const r = await post('components', { params: S.params }, 'sizing components');
  const v = view();
  if (!r.closure.feasible) {
    v.appendChild(card('No machine closes', '', h('div', { class: 'note' }, r.closure.reason)));
    return;
  }
  const b = r.breakdown, hov = b.rotor.hover, mx = b.rotor.max, arm = b.arm, pr = b.propulsion;

  v.appendChild(h('div', { class: 'grid g3' },
    card('Gross mass', '', stat('closed from geometry, not fractions', fmt(b.m, 4) + ' kg')),
    card('Hover power', '', stat('electrical, including screen and compute', fmt(b.P_total, 4) + ' W')),
    card('Span', '', stat('tip to tip', fmt(b.span, 3) + ' m'))));

  const massCard = card('Mass build-up', 'Each block is computed, not assumed.', cv('stackbar'));
  v.appendChild(massCard);
  draw(() => stack(massCard.querySelector('canvas'), {
    parts: [
      { label: 'fixed', value: b.m_fix, color: PALETTE[0] },
      { label: 'frame', value: b.m_frame, color: PALETTE[1] },
      { label: 'propulsion', value: b.m_prop, color: PALETTE[2] },
      { label: 'battery', value: b.m_bat, color: PALETTE[3] },
    ],
  }));

  const tip = r.optimal_tip_speed;
  v.appendChild(h('div', { class: 'grid g2' },
    card('Rotor', 'Momentum theory for the induced part, blade element for the profile part. The figure of merit falls out rather than going in.',
      kv([
        ['thrust per rotor at hover', fmt(hov.T, 3) + ' N'],
        ['tip speed', fmt(hov.v_tip, 4) + ' m/s'],
        ['rotational speed', fmt(hov.rpm, 5) + ' rpm'],
        ['solidity sigma', fmt(hov.sigma, 3)],
        ['blade loading Ct/sigma', fmt(hov.Ct_sigma, 3), hov.stalled ? 'bad' : 'good'],
        ['induced velocity', fmt(hov.v_induced, 3) + ' m/s'],
        ['downwash in the wake', fmt(b.downwash, 3) + ' m/s'],
        '-',
        ['ideal induced power', fmt(hov.P_ideal, 4) + ' W'],
        ['induced with tip loss', fmt(hov.P_induced, 4) + ' W'],
        ['profile power', fmt(hov.P_profile, 4) + ' W'],
        ['shaft power', fmt(hov.P_shaft, 4) + ' W'],
        ['figure of merit implied', fmt(hov.FM_effective, 3), 'accent'],
        '-',
        ['k_T  (T = k_T omega^2)', fmt(hov.k_T, 3)],
        ['k_Q  (Q = k_Q omega^2)', fmt(hov.k_Q, 3)],
        ['k_P  (P = k_P omega^3)', fmt(hov.k_P, 3)],
        '-',
        ['best tip speed at this thrust', tip.feasible ? fmt(tip.v_tip, 4) + ' m/s' : '--'],
        ['figure of merit there', tip.feasible ? fmt(tip.FM, 3) : '--'],
        ['noise at 1 m (indicative)', fmt(b.spl_1m, 3) + ' dB'],
      ])),
    card('Structure', 'Each arm is a thin-walled tube sized by bending stress, tip deflection and its first bending mode.',
      kv([
        ['arm length', fmt(arm.L, 3) + ' m'],
        ['wall thickness', fmt(arm.wall * 1000, 3) + ' mm'],
        ['sizing driver', arm.driver, 'accent'],
        ['  from stress', fmt(arm.wall_stress * 1000, 3) + ' mm'],
        ['  from stiffness', fmt(arm.wall_stiff * 1000, 3) + ' mm'],
        ['  minimum gauge', fmt(S.params.wall_min * 1000, 3) + ' mm'],
        ['working stress', fmt(arm.sigma / 1e6, 4) + ' MPa'],
        ['tip deflection at limit load', fmt(arm.tip_deflection * 1000, 3) + ' mm'],
        ['first bending mode', fmt(arm.f_bending, 4) + ' Hz', arm.f_bending > 25 ? 'good' : 'bad'],
        '-',
        ['arms', fmt(arm.m_arms, 3) + ' kg'],
        ['guards', fmt(arm.m_guards, 3) + ' kg'],
        ['frame total', fmt(arm.m_frame, 3) + ' kg'],
        ['implied f_s', fmt(b.f_s_implied, 3) + `  (assumed ${fmt(S.params.f_s, 3)})`, 'accent'],
        '-',
        ['Jxx', fmt(b.J.Jxx, 3) + ' kg m^2'],
        ['Jyy', fmt(b.J.Jyy, 3) + ' kg m^2'],
        ['Jzz', fmt(b.J.Jzz, 3) + ' kg m^2'],
      ]))));

  v.appendChild(h('div', { class: 'grid g2' },
    card('Propulsion', 'Motors sized by the torque they must deliver at maximum thrust.',
      kv([
        ['thrust per rotor at T_max', fmt(mx.T, 3) + ' N'],
        ['speed at T_max', fmt(mx.rpm, 5) + ' rpm'],
        ['tip speed at T_max', fmt(mx.v_tip, 4) + ' m/s'],
        ['blade loading at T_max', fmt(mx.Ct_sigma, 3), mx.stalled ? 'bad' : 'good'],
        ['torque at T_max', fmt(pr.Q_max, 3) + ' N.m'],
        '-',
        ['motors', fmt(pr.m_motors, 3) + ' kg'],
        ['ESCs', fmt(pr.m_esc, 3) + ' kg'],
        ['propellers', fmt(pr.m_props, 3) + ' kg'],
        ['propulsion total', fmt(pr.m_prop_total, 3) + ' kg'],
        ['implied S_m', fmt(pr.S_m_implied, 4) + ` N/kg  (assumed ${fmt(S.params.S_m, 4)})`, 'accent'],
        ['peak electrical power', fmt(pr.P_elec_max, 5) + ' W'],
      ])),
    card('One-dimensional trade study',
      'Sweep a single variable and watch every consequence at once.', h('div', { id: 'pareto-slot' }))));

  const slot = document.getElementById('pareto-slot');
  const varSel = h('select', { onchange: (e) => runPareto(e.target.value) },
    ...S.meta.design_vars.map((k) => {
      const s = S.schema.find((x) => x.key === k);
      return h('option', { value: k }, s ? s.label : k);
    }));
  slot.appendChild(h('div', { class: 'row' }, h('label', {}, 'variable'), varSel));
  const pcv = cv('mid');
  slot.appendChild(pcv);

  async function runPareto(variable) {
    const pr2 = await post('pareto', { params: S.params, variable, n: 26 }, 'trade study');
    const ok = pr2.points.filter((p) => p.feasible);
    const norm = (key) => {
      const vals = ok.map((p) => p[key]).filter((x) => isFinite(x));
      const mx2 = Math.max(...vals) || 1;
      return ok.map((p) => p[key] / mx2);
    };
    draw(() => linechart(pcv, {
      series: [
        { x: ok.map((p) => p.value), y: norm('m'), label: 'gross mass', color: PALETTE[0] },
        { x: ok.map((p) => p.value), y: norm('P_total'), label: 'hover power', color: PALETTE[1] },
        { x: ok.map((p) => p.value), y: norm('span'), label: 'span', color: PALETTE[2] },
        { x: ok.map((p) => p.value), y: norm('spl'), label: 'noise', color: PALETTE[3] },
        { x: ok.map((p) => p.value), y: norm('downwash'), label: 'downwash', color: PALETTE[4] },
      ],
      xlabel: `${pr2.label}  [${pr2.unit}]`, ylabel: 'normalised to the largest value',
      vlines: [{ x: S.params[variable], label: 'current', color: '#8a94a3' }],
    }));
  }
  await runPareto(S.meta.design_vars[0]);

  if (b.warnings && b.warnings.length) v.appendChild(warns(b.warnings));
}

// ---------------------------------------------------------------------------
// 6. Flight
// ---------------------------------------------------------------------------
const FLIGHT = { mission: 'pacing', t_window: 12, dt: 0.004, result: null, anim: null, governed: true };

async function renderFlight() {
  const v = view();
  const missionSel = h('select', { onchange: (e) => { FLIGHT.mission = e.target.value; } },
    ...S.meta.missions.map((m) => h('option', { value: m, selected: m === FLIGHT.mission ? '' : null }, m.replace('_', ' '))));
  const winSel = h('select', { onchange: (e) => { FLIGHT.t_window = +e.target.value; } },
    ...[6, 12, 20, 40].map((t) => h('option', { value: t, selected: t === FLIGHT.t_window ? '' : null }, `${t} s window`)));
  const runBtn = h('button', { class: 'act primary', onclick: () => runFlight() }, 'Run simulation');
  const linBtn = h('button', { class: 'act', onclick: () => runLin() }, 'Linearise at hover');
  const govCb = h('input', { type: 'checkbox', ...(FLIGHT.governed ? { checked: '' } : {}) });
  govCb.onchange = () => { FLIGHT.governed = govCb.checked; };
  v.appendChild(h('div', { class: 'row' }, h('label', {}, 'mission'), missionSel, winSel,
    h('label', { style: 'color:var(--fg)', title: 'Rate limits the reference so the screen yields during a turn instead of being whipped around the head' },
      govCb, ' reference governor'), runBtn, linBtn));

  v.appendChild(h('div', { class: 'note', html:
    `The full nonlinear model: <b>m v&#775; = T R e3 - m g e3 + F_D</b>, <b>J w&#775; + w x J w = tau</b>,
     first-order rotor speed lag, a battery energy state, a two-axis screen gimbal, and a cascaded
     position to attitude to rate controller. The screen is a flat plate, so its drag depends on
     orientation and feeds torque back into the attitude loop.`}));

  const slot = h('div', { id: 'flight-slot', style: 'margin-top:14px' });
  v.appendChild(slot);
  if (FLIGHT.result) drawFlight(FLIGHT.result, slot);
  else slot.appendChild(h('div', { class: 'note' }, 'Nothing simulated yet.'));

  async function runLin() {
    const lin = await post('linearise', { params: S.params }, 'linearising');
    if (lin.ok === false) { slot.innerHTML = ''; slot.appendChild(card('Cannot linearise', '', h('div', { class: 'note' }, lin.reason))); return; }
    const ev = lin.eigenvalues.map(([re, im]) => ({ re, im }));
    const c = card('Hover linearisation',
      `A and B computed numerically about hover for the ${lin.n_states} rigid-body and rotor states.
       Controllability rank ${lin.rank} of ${lin.n_states}: <b>${lin.controllable ? 'controllable' : 'not controllable'}</b>.
       The cluster at the origin is the six open-loop integrators, which is exactly why a multirotor
       needs the cascade rather than a single loop.`,
      cv('mid'));
    slot.insertBefore(c, slot.firstChild);
    draw(() => linechart(c.querySelector('canvas'), {
      series: [{ x: ev.map((e) => e.re), y: ev.map((e) => e.im), type: 'dots', color: PALETTE[0], r: 4 }],
      xlabel: 'real part  [1/s]', ylabel: 'imaginary part  [rad/s]', legend: false,
      vlines: [{ x: 0, color: '#8a94a3', dash: [2, 3] }],
    }));
  }

  async function runFlight() {
    runBtn.disabled = true;
    try {
      const res = await post('simulate', {
        params: S.params, mission: FLIGHT.mission,
        t_window: FLIGHT.t_window, dt: FLIGHT.dt,
        governor: FLIGHT.governed ? { v_max: 2.5, a_max: 2.5 } : null,
      }, `simulating ${FLIGHT.t_window} s of flight`);
      FLIGHT.result = res;
      slot.innerHTML = '';
      drawFlight(res, slot);
    } finally { runBtn.disabled = false; }
  }
}

function drawFlight(res, slot) {
  if (res.ok === false) {
    slot.appendChild(card('Simulation failed', '', h('div', { class: 'note' }, res.reason)));
    return;
  }
  const t = res.t, tel = res.telemetry, tr = res.trajectory;

  slot.appendChild(h('div', { class: 'grid g3' },
    card('Mean electrical power', '', stat(`steady hover is ${fmt(res.P_hover_ref, 4)} W, so this mission costs ${fmt(res.overhead, 3)} times that`,
      fmt(res.P_mean, 4) + ' W')),
    card('Lag behind the person', '',
      stat(`peak ${fmt(res.e_lag_max, 3)} m; the controller itself tracks its own reference to ${fmt(res.e_track_rms, 3)} m`,
        fmt(res.e_lag_rms, 3) + ' m rms',
        res.e_lag_rms < 0.15 ? 'good' : res.e_lag_rms < 0.5 ? '' : 'bad')),
    card('Screen tilt', '', stat(`airframe tilts up to ${fmt(res.tilt_max_deg, 3)} deg; the gimbal removes most of it`,
      fmt(res.screen_tilt_max_deg, 3) + ' deg peak',
      res.screen_tilt_max_deg < S.params.theta_readable ? 'good' : 'bad'))));

  const anim = card('Following', 'Top view on the left, side view on the right. The dot is the person, the cross is where the screen should be, the vehicle is drawn to scale.',
    h('canvas', { class: 'anim', style: 'height:280px' }),
    h('div', { class: 'row', style: 'margin-top:8px' },
      h('button', { class: 'act', onclick: () => { FLIGHT.playing = !FLIGHT.playing; } }, 'Play / pause'),
      h('input', { type: 'range', min: 0, max: t.length - 1, value: 0, style: 'flex:1', oninput: (e) => { FLIGHT.frame = +e.target.value; FLIGHT.playing = false; } })));
  slot.appendChild(anim);
  startAnimation(anim.querySelector('canvas'), res, anim.querySelector('input[type=range]'));

  const c1 = card('Power and state of charge', '', cv('mid'));
  const c2 = card('Tracking error and tilt', '', cv('mid'));
  const c3 = card('Thrust and rotor speed', '', cv('mid'));
  const c4 = card('Speed', '', cv('mid'));
  slot.appendChild(h('div', { class: 'grid g2' }, c1, c2));
  slot.appendChild(h('div', { class: 'grid g2' }, c3, c4));

  draw(() => linechart(c1.querySelector('canvas'), {
    series: [{ x: t, y: tel.P, label: 'electrical power  [W]', color: PALETTE[0] }],
    xlabel: 'time  [s]', ylabel: 'W',
    hlines: [{ y: res.P_mean, label: `mean ${fmt(res.P_mean, 4)} W`, color: PALETTE[1] }],
  }));
  draw(() => linechart(c2.querySelector('canvas'), {
    series: [
      { x: t, y: tel.e_track, label: 'tracking error  [m]', color: PALETTE[0] },
      { x: t, y: tel.screen_tilt_deg.map((d) => d / 10), label: 'screen tilt / 10  [deg]', color: PALETTE[2] },
      { x: t, y: tel.tilt_deg.map((d) => d / 10), label: 'airframe tilt / 10  [deg]', color: PALETTE[1] },
    ],
    xlabel: 'time  [s]', ylabel: 'm  and  deg / 10',
  }));
  draw(() => linechart(c3.querySelector('canvas'), {
    series: [
      { x: t, y: tel.thrust_ratio, label: 'thrust / weight', color: PALETTE[0] },
      { x: t, y: tel.omega_mean.map((o) => o / res.vehicle.omega_max), label: 'mean rotor speed / max', color: PALETTE[2] },
      { x: t, y: tel.sat, label: 'saturated', color: PALETTE[3] },
    ],
    xlabel: 'time  [s]', ylabel: '-',
    hlines: [{ y: S.params.lam, label: `lambda = ${fmt(S.params.lam, 3)}`, color: PALETTE[1] }],
  }));
  draw(() => linechart(c4.querySelector('canvas'), {
    series: [{ x: t, y: tel.v_speed, label: 'vehicle speed  [m/s]', color: PALETTE[0] }],
    xlabel: 'time  [s]', ylabel: 'm/s',
  }));

  slot.appendChild(card('Mission energy', '', kv([
    ['mean power over the window', fmt(res.P_mean, 4) + ' W'],
    ['steady hover of the same rotor', fmt(res.P_hover_ref, 4) + ' W'],
    ['what the closed-form layer assumes', fmt(res.P_ideal_lumped, 4) + ' W'],
    ['cost of the mission over hovering', fmt(res.overhead, 3) + ' x', res.overhead > 1.4 ? 'bad' : ''],
    ['energy for the full mission', fmt(res.E_mission / 3600 / 1000, 4) + ' kWh  (' + fmt(res.E_mission, 4) + ' J)'],
    ['pack energy on board', fmt(res.vehicle.E_max / 3600 / 1000, 4) + ' kWh'],
    ['state of charge at the end', fmt(100 * res.soc_end, 3) + ' %', res.soc_end > S.params.soc_reserve ? 'good' : 'bad'],
    ['endurance at this power', secs(res.endurance_est) + `  (asked for ${secs(S.params.t_f)})`],
    '-',
    ['motor saturation', fmt(100 * res.saturation_fraction, 3) + ' % of the time'],
    ['peak acceleration the mission asks for', fmt(res.ref_accel_max, 3) + ' m/s^2'],
    ['thrust margin that peak alone needs', fmt(res.lam_needed, 3) + `  (lambda is ${fmt(S.params.lam, 3)})`,
      S.params.lam > res.lam_needed ? 'good' : 'bad'],
    ['nearest rotor disk to the eye', fmt(res.rotor_clearance_min, 3) + ' m',
      res.rotor_clearance_min >= S.params.d_safe ? 'good' : 'bad'],
    ['envelope sphere to the eye (conservative)', fmt(res.min_human_distance, 3) + ' m'],
    ['tracking error, worst', fmt(res.e_track_max_all, 3) + ' m' +
      (res.tracking_margin ? `  (keep-out allows ${fmt(res.tracking_margin, 2)} m)` : ''),
      res.margin_holds ? '' : 'bad'],
    ['gross mass simulated', fmt(res.m, 4) + ' kg'],
    ['span', fmt(res.vehicle.span, 3) + ' m'],
  ])));

  if (res.authority) {
    const a = res.authority, g = res.suggested_gains || {};
    slot.appendChild(card('Control authority',
      'How much angular acceleration the rotors can actually produce, and the gains that follow from it. Yaw comes from rotor reaction torque, which is exactly what a quiet slow rotor has least of.',
      kv([
        ['roll', fmt(a.alpha_roll, 3) + ' rad/s^2'],
        ['pitch', fmt(a.alpha_pitch, 3) + ' rad/s^2'],
        ['yaw', fmt(a.alpha_yaw, 3) + ' rad/s^2', a.alpha_yaw < 3 ? 'bad' : ''],
        '-',
        ['attitude gain in use', fmt(S.params.K_R, 3)],
        ['attitude gain this machine can back', fmt(g.K_R, 3),
          S.params.K_R > 1.3 * g.K_R ? 'bad' : 'good'],
        ['rate gain suggested', fmt(g.K_w, 3)],
        ['position gains suggested', `Kp ${fmt(g.Kp_pos, 3)}, Kd ${fmt(g.Kd_pos, 3)}`],
      ]),
      h('div', { class: 'row', style: 'margin-top:10px' },
        h('button', {
          class: 'act', onclick: () => S.controls.setAll({
            K_R: g.K_R, K_w: g.K_w, Kp_pos: g.Kp_pos, Kd_pos: g.Kd_pos, Ki_pos: g.Ki_pos,
          }),
        }, 'Adopt the sized gains'))));
  }
  if (res.warnings && res.warnings.length) slot.appendChild(warns(res.warnings));
}

function startAnimation(canvas, res, slider) {
  if (FLIGHT.anim) cancelAnimationFrame(FLIGHT.anim);
  FLIGHT.frame = 0; FLIGHT.playing = true;
  const tr = res.trajectory, span = res.vehicle.span;
  const all = tr.r.concat(tr.r_h);
  const xs = all.map((p) => p[0]), ys = all.map((p) => p[1]), zs = all.map((p) => p[2]);
  const pad = span;
  const bx = [Math.min(...xs) - pad, Math.max(...xs) + pad];
  const by = [Math.min(...ys) - pad, Math.max(...ys) + pad];
  const bz = [Math.min(0, Math.min(...zs) - 0.3), Math.max(...zs) + pad];

  function frame() {
    const r = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(r.width * dpr); canvas.height = Math.round(r.height * dpr);
    const g = canvas.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, r.width, r.height);
    const half = r.width / 2;
    const i = Math.min(FLIGHT.frame | 0, tr.r.length - 1);

    const panel = (x0, w, ax1, ax2, b1, b2, label) => {
      g.save();
      g.beginPath(); g.rect(x0, 0, w, r.height); g.clip();
      g.strokeStyle = '#2a323e'; g.strokeRect(x0 + 0.5, 0.5, w - 1, r.height - 1);
      const sc = Math.min((w - 40) / (b1[1] - b1[0]), (r.height - 40) / (b2[1] - b2[0]));
      const P = (p) => [x0 + w / 2 + (p[ax1] - (b1[0] + b1[1]) / 2) * sc,
      r.height / 2 - (p[ax2] - (b2[0] + b2[1]) / 2) * sc];
      // ground line for the side view
      if (ax2 === 2) {
        const [, gy] = P([0, 0, 0]);
        g.strokeStyle = '#39424f'; g.setLineDash([3, 4]);
        g.beginPath(); g.moveTo(x0, gy); g.lineTo(x0 + w, gy); g.stroke(); g.setLineDash([]);
      }
      // trails
      g.strokeStyle = 'rgba(94,200,242,0.28)'; g.lineWidth = 1.2; g.beginPath();
      for (let k = Math.max(0, i - 220); k <= i; k++) {
        const [px, py] = P(tr.r[k]);
        k === Math.max(0, i - 220) ? g.moveTo(px, py) : g.lineTo(px, py);
      }
      g.stroke();
      g.strokeStyle = 'rgba(140,233,154,0.30)'; g.beginPath();
      for (let k = Math.max(0, i - 220); k <= i; k++) {
        const [px, py] = P(tr.r_h[k]);
        k === Math.max(0, i - 220) ? g.moveTo(px, py) : g.lineTo(px, py);
      }
      g.stroke();
      // person
      const [hx, hy] = P(tr.r_h[i]);
      g.fillStyle = '#8ce99a';
      g.beginPath(); g.arc(hx, hy, 5, 0, 7); g.fill();
      if (ax2 === 2) { g.strokeStyle = '#8ce99a'; g.lineWidth = 2; g.beginPath(); g.moveTo(hx, hy); g.lineTo(hx, hy - 1.7 * sc); g.stroke(); }
      // target
      const [tx, ty] = P(tr.r_d[i]);
      g.strokeStyle = '#f2a65e'; g.lineWidth = 1.4;
      g.beginPath(); g.moveTo(tx - 6, ty); g.lineTo(tx + 6, ty); g.moveTo(tx, ty - 6); g.lineTo(tx, ty + 6); g.stroke();
      // vehicle
      const [vx, vy] = P(tr.r[i]);
      const eul = tr.euler[i];
      g.save(); g.translate(vx, vy);
      if (ax2 === 2) g.rotate(-eul[1] * Math.PI / 180);
      else g.rotate(-eul[2] * Math.PI / 180);
      g.strokeStyle = '#5ec8f2'; g.lineWidth = 2.4;
      const L = span * sc / 2;
      g.beginPath(); g.moveTo(-L, 0); g.lineTo(L, 0); g.stroke();
      g.fillStyle = '#5ec8f2';
      g.beginPath(); g.arc(-L, 0, 3.2, 0, 7); g.fill();
      g.beginPath(); g.arc(L, 0, 3.2, 0, 7); g.fill();
      // screen slab hanging below
      g.fillStyle = '#d7dee7';
      const sw = Math.max(S.params.screen_w * sc, 5), sh = Math.max(S.params.screen_h * sc, 3);
      g.fillRect(-sw / 2, S.params.r_cp * sc, sw, ax2 === 2 ? sh : sh * 0.35);
      g.restore();
      g.fillStyle = '#7f8b9a'; g.font = '11px ui-monospace, monospace';
      g.textAlign = 'left'; g.textBaseline = 'top';
      g.fillText(label, x0 + 8, 8);
      g.fillText(`t = ${fmt(res.t[i], 3)} s`, x0 + 8, 22);
      g.restore();
    };
    panel(0, half - 4, 0, 1, bx, by, 'top view  (x, y)');
    panel(half + 4, half - 4, 0, 2, bx, bz, 'side view  (x, z)');

    if (FLIGHT.playing) {
      FLIGHT.frame = (FLIGHT.frame + 1) % tr.r.length;
      if (slider) slider.value = FLIGHT.frame;
    }
    FLIGHT.anim = requestAnimationFrame(frame);
  }
  frame();
}

// ---------------------------------------------------------------------------
// 7. Energy loop
// ---------------------------------------------------------------------------
const LOOP = { mission: 'pacing', t_window: 8, m_bat0: null, result: null };

async function renderLoop() {
  const v = view();
  v.appendChild(h('div', { class: 'note', html:
    `Stage six of the plan. Instead of assuming <b>E = P t</b> with an ideal hover power, guess a
     battery mass, close the rest of the vehicle around it, fly the actual mission, integrate the
     actual power, and ask what battery that needed. Repeat. The residual pair being driven to zero is
     <b>R1 = m - (m_fix + m_str + m_prop + m_bat)</b> and
     <b>R2 = m_bat - max(E_mission / (e_b (1 - reserve)), P_peak / p_b)</b>. Converging closes the mass and
     energy balance; the last flight is then checked separately for reserve, peak power, rotor clearance
     and screen attitude.`}));

  const missionSel = h('select', { onchange: (e) => { LOOP.mission = e.target.value; } },
    ...S.meta.missions.map((m) => h('option', { value: m, selected: m === LOOP.mission ? '' : null }, m.replace('_', ' '))));
  const startInput = h('input', { type: 'number', step: '0.05', placeholder: 'auto', style: 'width:90px' });
  const btn = h('button', { class: 'act primary', onclick: run }, 'Run the loop');
  v.appendChild(h('div', { class: 'row' },
    h('label', {}, 'mission'), missionSel,
    h('label', {}, 'starting battery guess'), startInput, h('span', { class: 'note' }, 'kg'),
    btn));

  const slot = h('div', {});
  v.appendChild(slot);
  if (LOOP.result) drawLoop(LOOP.result, slot);

  async function run() {
    btn.disabled = true;
    try {
      const guess = parseFloat(startInput.value);
      const r = await post('closure', {
        params: S.params, mission: LOOP.mission, t_window: LOOP.t_window,
        m_bat0: isFinite(guess) ? guess : null, max_iter: 8,
      }, 'closing the battery loop, this runs a full simulation per iteration');
      LOOP.result = r;
      slot.innerHTML = '';
      drawLoop(r, slot);
    } finally { btn.disabled = false; }
  }
}

function drawLoop(r, slot) {
  const cls = r.status === 'converged' ? 'ok' : r.status === 'diverging' ? 'no' : 'warn';
  slot.appendChild(h('div', { class: 'row' },
    h('span', { class: 'pill ' + cls }, r.status),
    r.converged ? h('span', { class: 'pill ' + (r.feasible ? 'ok' : 'no') },
      r.feasible ? 'constraints met' : 'constraints violated') : null,
    h('span', { class: 'note' }, `${r.iterations || 0} updates, ${r.sim_calls || 0} simulations`),
    r.reason ? h('span', { class: 'note' }, r.reason) : null));

  if (r.trace && r.trace.length) {
    const c = card('Battery mass iteration',
      'Each point is one full 6-DOF mission simulation. Converging is the design existing; running away is the mass spiral, found rather than assumed.',
      cv('mid'));
    slot.appendChild(c);
    const it = r.trace.map((t) => t.iteration);
    draw(() => linechart(c.querySelector('canvas'), {
      series: [
        { x: it, y: r.trace.map((t) => t.m_bat), label: 'battery guessed', color: PALETTE[0] },
        { x: it, y: r.trace.map((t) => t.m_bat_required), label: 'battery required', color: PALETTE[1] },
        { x: it, y: r.trace.map((t) => t.m), label: 'gross mass', color: PALETTE[2] },
      ],
      xlabel: 'iteration', ylabel: 'mass  [kg]',
    }));
    slot.appendChild(card('Iteration detail', '',
      h('table', { class: 'data' },
        h('thead', {}, h('tr', {}, ...['it', 'battery guess', 'battery required', 'gross mass', 'mean power', 'overhead', 'R2 residual', 'tracking rms'].map((t) => h('th', {}, t)))),
        h('tbody', {}, r.trace.map((t) => h('tr', {},
          h('td', {}, String(t.iteration) + (t.kind === 'verify' ? ' (verify)' : '')),
          h('td', {}, fmt(t.m_bat, 4) + ' kg'),
          h('td', {}, fmt(t.m_bat_required, 4) + ' kg'),
          h('td', {}, fmt(t.m, 4) + ' kg'),
          h('td', {}, fmt(t.P_mean, 4) + ' W'),
          h('td', {}, fmt(t.overhead, 3) + ' x'),
          h('td', {}, fmt(1000 * t.residual, 3) + ' g'),
          h('td', {}, fmt(t.e_track_rms, 3) + ' m')))))));
  }

  if (r.converged) {
    slot.appendChild(h('div', { class: 'grid g3' },
      card('Gross mass', '', stat('with the simulator in the loop', fmt(r.m, 4) + ' kg')),
      card('Battery', '', stat(`${fmt(100 * r.m_bat / r.m, 3)} percent of gross mass`, fmt(r.m_bat, 4) + ' kg')),
      card('Mission energy', '', stat(`${fmt(r.overhead, 3)} times the ideal hover integral`, fmt(r.E_mission / 3.6e6, 4) + ' kWh'))));
    slot.appendChild(card('Margins on the final flight', '', kv([
      ['energy margin above the reserve', fmt(r.energy_margin / 3600, 3) + ' Wh', r.energy_margin >= 0 ? 'good' : 'bad'],
      ['peak power margin', fmt(r.power_margin, 3) + ' W', r.power_margin >= 0 ? 'good' : 'bad'],
      ['state of charge left', fmt(100 * r.reserve_end, 4) + ' %'],
      ['rotor disk to eye, minimum', r.sim && isFinite(r.sim.rotor_clearance_min) ? fmt(r.sim.rotor_clearance_min, 3) + ' m' : 'n/a',
        r.sim && isFinite(r.sim.rotor_clearance_min) ? (r.sim.rotor_clearance_min >= S.params.d_safe ? 'good' : 'bad') : ''],
      ['flight constraints', r.constraints_ok ? 'met' : 'violated', r.constraints_ok ? 'good' : 'bad'],
    ])));
  }

  const an = r.analytic || {};
  slot.appendChild(card('Against the closed form',
    'The analytic closure assumes ideal hover for the whole mission. The difference is what the controller, the drag and the human motion actually cost.',
    kv([
      ['analytic closure', an.feasible ? fmt(an.m, 4) + ' kg' : 'no solution: ' + (an.reason || ''), an.feasible ? '' : 'bad'],
      ['analytic battery', an.feasible ? fmt(an.m_bat, 4) + ' kg' : '--'],
      ['analytic hover power', an.feasible ? fmt(an.P_total, 4) + ' W' : '--'],
      '-',
      ['simulated gross mass', fmt(r.m, 4) + ' kg', 'accent'],
      ['simulated battery', fmt(r.m_bat, 4) + ' kg', 'accent'],
      ['simulated mean power', fmt(r.P_mean, 4) + ' W', 'accent'],
      ['power overhead', fmt(r.overhead, 3) + ' x'],
    ])));

  if (r.warnings && r.warnings.length) slot.appendChild(warns(r.warnings));
}

// ---------------------------------------------------------------------------
// 8. Optimise
// ---------------------------------------------------------------------------
const OPT = { vars: null, weights: null, limits: null, result: null, sim: false };

async function renderOptimise() {
  const v = view();
  OPT.vars = OPT.vars || [...S.meta.default_vars];
  OPT.weights = OPT.weights || { ...S.meta.weights };
  OPT.limits = OPT.limits || { ...S.meta.limits };

  v.appendChild(h('div', { class: 'note', html:
    `Minimise <b>J = w_m m + w_P P + w_N SPL + w_span span</b> over the chosen variables, subject to the
     mass closure existing, blade loading below stall, a thrust margin large enough for the mission,
     and limits on span, noise and downwash. Constraints are penalised rather than clipped so the
     search still has somewhere to go inside the infeasible region.`}));

  const varBox = h('div', { class: 'row' }, h('label', {}, 'variables'));
  for (const k of S.meta.design_vars) {
    const s = S.schema.find((x) => x.key === k);
    const id = 'ov_' + k;
    const cb = h('input', { type: 'checkbox', id, ...(OPT.vars.includes(k) ? { checked: '' } : {}) });
    cb.onchange = () => {
      OPT.vars = OPT.vars.filter((x) => x !== k);
      if (cb.checked) OPT.vars.push(k);
    };
    varBox.appendChild(h('label', { style: 'color:var(--fg)' }, cb, ' ' + (s ? s.label : k)));
  }
  v.appendChild(varBox);

  const wBox = h('div', { class: 'row' }, h('label', {}, 'weights'));
  for (const [k, val] of Object.entries(OPT.weights)) {
    const inp = h('input', { type: 'number', value: String(val), step: '0.001', style: 'width:80px' });
    inp.onchange = () => { OPT.weights[k] = parseFloat(inp.value); };
    wBox.appendChild(h('label', {}, k.replace('w_', '') + ' '));
    wBox.appendChild(inp);
  }
  v.appendChild(wBox);

  const lBox = h('div', { class: 'row' }, h('label', {}, 'limits'));
  for (const [k, val] of Object.entries(OPT.limits)) {
    const inp = h('input', { type: 'number', value: String(val), step: '0.05', style: 'width:80px' });
    inp.onchange = () => { OPT.limits[k] = parseFloat(inp.value); };
    lBox.appendChild(h('label', {}, k + ' '));
    lBox.appendChild(inp);
  }
  v.appendChild(lBox);

  const simCb = h('input', { type: 'checkbox', ...(OPT.sim ? { checked: '' } : {}) });
  simCb.onchange = () => { OPT.sim = simCb.checked; };
  const btn = h('button', { class: 'act primary', onclick: run }, 'Search');
  v.appendChild(h('div', { class: 'row' }, btn,
    h('label', { style: 'color:var(--fg)' }, simCb, ' simulate the winner')));

  const slot = h('div', {});
  v.appendChild(slot);
  if (OPT.result) drawOpt(OPT.result, slot);

  async function run() {
    btn.disabled = true;
    try {
      const r = await post('optimize', {
        params: S.params, variables: OPT.vars, weights: OPT.weights,
        limits: OPT.limits, maxiter: 35, popsize: 12,
        refine_with_sim: OPT.sim, mission: FLIGHT.mission,
      }, 'searching the design space');
      OPT.result = r;
      slot.innerHTML = '';
      drawOpt(r, slot);
    } finally { btn.disabled = false; }
  }
}

function drawOpt(r, slot) {
  const b = r.best, base = r.baseline;
  slot.appendChild(h('div', { class: 'grid g3' },
    card('Objective', '', stat(`baseline was ${fmt(base.J, 4)}`, fmt(b.J, 4), b.J < base.J ? 'good' : '')),
    card('Gross mass', '', stat(`baseline ${fmt(base.m, 4)} kg`, fmt(b.m, 4) + ' kg')),
    card('Hover power', '', stat(`baseline ${fmt(base.P_total, 4)} W`, fmt(b.P_total, 4) + ' W'))));

  const rows = Object.entries(b.x).map(([k, val]) => {
    const s = S.schema.find((x) => x.key === k);
    return h('tr', {},
      h('td', {}, s ? s.label : k),
      h('td', {}, fmt(S.params[k], 4)),
      h('td', {}, fmt(val, 4)),
      h('td', {}, s ? s.unit : ''));
  });
  slot.appendChild(card('Design variables', '',
    h('table', { class: 'data' },
      h('thead', {}, h('tr', {}, ...['variable', 'current', 'optimum', 'unit'].map((t) => h('th', {}, t)))),
      h('tbody', {}, rows)),
    h('div', { class: 'row', style: 'margin-top:10px' },
      h('button', { class: 'act primary', onclick: () => S.controls.setAll(b.x) }, 'Adopt this design'))));

  slot.appendChild(h('div', { class: 'grid g2' },
    card('Consequences', '', kv([
      ['span', fmt(b.span, 3) + ` m  (limit ${fmt(r.limits.span_max, 3)})`, b.span <= r.limits.span_max ? 'good' : 'bad'],
      ['noise at 1 m', fmt(b.spl, 4) + ` dB  (limit ${fmt(r.limits.spl_max, 4)})`, b.spl <= r.limits.spl_max ? 'good' : 'bad'],
      ['downwash', fmt(b.downwash, 3) + ` m/s  (limit ${fmt(r.limits.downwash_max, 3)})`, b.downwash <= r.limits.downwash_max ? 'good' : 'bad'],
      ['battery', fmt(b.m_bat, 4) + ' kg'],
      ['thrust margin needed', fmt(b.lam_required, 3)],
      ['endurance implied', secs(b.endurance)],
    ])),
    card('Search', '', kv([
      ['generations', String(r.iterations)],
      ['objective evaluations', String(r.evaluations)],
      ['converged', r.success ? 'yes' : 'stopped on the iteration limit'],
      ['constraints', b.violations.length ? b.violations.join('; ') : 'all satisfied',
        b.violations.length ? 'bad' : 'good'],
    ]))));

  if (r.history && r.history.length > 1) {
    const c = card('Best objective by generation', '', cv('mid'));
    slot.appendChild(c);
    draw(() => linechart(c.querySelector('canvas'), {
      series: [{ x: r.history.map((_, i) => i), y: r.history, color: PALETTE[0] }],
      xlabel: 'generation', ylabel: 'J', legend: false,
    }));
  }
  if (r.sim) {
    slot.appendChild(card('The winner, simulated', '', r.sim.ok ? kv([
      ['mean power', fmt(r.sim.P_mean, 4) + ' W'],
      ['overhead over ideal hover', fmt(r.sim.overhead, 3) + ' x'],
      ['tracking error rms', fmt(r.sim.e_track_rms, 3) + ' m'],
      ['peak screen tilt', fmt(r.sim.screen_tilt_max_deg, 3) + ' deg'],
      ['motor saturation', fmt(100 * r.sim.saturation_fraction, 3) + ' %'],
    ]) : h('div', { class: 'note' }, r.sim.reason),
      r.sim.warnings ? warns(r.sim.warnings) : null));
  }
}

// ---------------------------------------------------------------------------
// 9. Tether
// ---------------------------------------------------------------------------
async function renderTether() {
  const r = await post('tether', { params: S.params }, 'solving the tethered closure');
  const v = view();
  const bat = r.battery, te = r.tether;

  v.appendChild(h('div', { class: 'note', html:
    `Removing the battery deletes the term that carries the m^1.5 and makes the closure fold.
     What replaces it is a conductor whose mass goes as <b>4 rho_e rho_m L^2 P / (drop V^2)</b>.
     Note what is missing from that expression: the mission time. Endurance stops being a design
     variable and becomes an operating choice.`}));

  v.appendChild(h('div', { class: 'grid g2' },
    card('Onboard battery', '',
      h('div', { class: 'row' }, h('span', { class: bat.feasible ? 'pill ok' : 'pill no' },
        bat.feasible ? 'closes' : 'no solution')),
      bat.feasible ? kv([
        ['gross mass', fmt(bat.m, 4) + ' kg', 'accent'],
        ['battery', fmt(bat.m_bat, 4) + ' kg'],
        ['hover power', fmt(bat.P_total, 4) + ' W'],
        ['endurance', secs(S.params.t_f)],
        ['disk loading', fmt(bat.disk_loading, 4) + ' N/m^2'],
        ['beta (energy coupling)', fmt(bat.coeffs.beta, 4)],
        ['wall M0_max', fmt(r.wall.M0_max, 4) + ' kg'],
      ]) : h('div', { class: 'note' }, bat.reason)),
    card('Tethered', '',
      h('div', { class: 'row' }, h('span', { class: te.feasible ? 'pill ok' : 'pill no' },
        te.feasible ? 'closes' : 'no solution'),
        te.gauge_limited ? h('span', { class: 'pill warn' }, 'at minimum practical gauge') : null),
      te.feasible ? kv([
        ['gross mass', fmt(te.m, 4) + ' kg', 'accent'],
        ['tether carried', fmt(te.m_tether_carried, 4) + ' kg'],
        ['bus power', fmt(te.P_bus, 4) + ' W'],
        ['source power', fmt(te.P_source, 4) + ' W'],
        ['resistive loss', fmt(te.P_loss, 4) + ' W'],
        ['current', fmt(te.current, 3) + ' A'],
        ['conductor per leg', fmt(te.conductor_area_mm2, 3) + ` mm^2  (${fmt(te.conductor_dia_mm, 3)} mm)`],
        ['tether mass per metre', fmt(te.tether_mass_per_m * 1000, 3) + ' g/m'],
        ['beta (tether coupling)', fmt(te.beta_tether, 3)],
        ['endurance', 'unlimited', 'good'],
        ['wall M0_max', te.M0_max ? fmt(te.M0_max, 4) + ' kg' : 'none', 'good'],
      ]) : h('div', { class: 'note' }, te.reason))));

  // conductor mass against voltage and length
  const c1 = card('Conductor mass against bus voltage',
    'The whole tethered idea lives or dies on voltage: conductor mass falls as one over voltage squared.',
    cv('mid'));
  const c2 = card('Gross mass against tether length',
    'Length enters squared, so a long leash is expensive even at high voltage.', cv('mid'));
  v.appendChild(h('div', { class: 'grid g2' }, c1, c2));

  const Vs = [], Ms = [];
  for (let i = 0; i < 40; i++) {
    const V = 24 * Math.pow(1000 / 24, i / 39);
    const P = te.feasible ? te.P_source : 200;
    const k = S.params.tether_insul * 4 * S.params.tether_rho_e * S.params.tether_rho_m *
      Math.pow(S.params.tether_len, 2) / (S.params.tether_drop * V * V);
    Vs.push(V); Ms.push(Math.max(k * P, S.params.tether_insul * S.params.tether_rho_m * 0.13e-6 * 2 * S.params.tether_len));
  }
  draw(() => linechart(c1.querySelector('canvas'), {
    series: [{ x: Vs, y: Ms, color: PALETTE[0] }],
    xlog: true, ylog: true, xlabel: 'bus voltage  [V]', ylabel: 'tether mass  [kg]', legend: false,
    vlines: [{ x: S.params.tether_V, label: 'current', color: PALETTE[1] }],
  }));

  const Ls = [], Gs = [];
  for (let i = 0; i < 30; i++) {
    const L = 1 + i * (40 / 29);
    const k = S.params.tether_insul * 4 * S.params.tether_rho_e * S.params.tether_rho_m *
      L * L / (S.params.tether_drop * S.params.tether_V * S.params.tether_V);
    const P = te.feasible ? te.P_source : 200;
    Ls.push(L); Gs.push(Math.max(k * P, S.params.tether_insul * S.params.tether_rho_m * 0.13e-6 * 2 * L));
  }
  draw(() => linechart(c2.querySelector('canvas'), {
    series: [{ x: Ls, y: Gs, color: PALETTE[2] }],
    ylog: true, xlabel: 'tether length  [m]', ylabel: 'tether mass  [kg]', legend: false,
    vlines: [{ x: S.params.tether_len, label: 'current', color: PALETTE[1] }],
  }));

  v.appendChild(card('What the comparison says', '', h('div', { class: 'note', html:
    `${r.note}<br><br>
     The battery closure carries <b>beta = t g^1.5 / (e_b FM eta sqrt(2 rho A))</b>, which grows
     linearly with mission time and squares its way into the payload wall. The tethered closure
     carries a coefficient built only from geometry and electrical constants. That is the whole
     structural difference, and it is why the same screen that has no solution at eight hours on a
     battery closes immediately on a wire.`})));
}
