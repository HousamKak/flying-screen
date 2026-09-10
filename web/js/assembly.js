// Turn a solved design into an assembly drawing.
//
// Every dimension here comes from the closure: arm length and wall thickness
// from the beam model, rotor diameter and chord from the blade model, motor
// size from the torque it has to hold, cell count from the electrical chain.
// Nothing is drawn to look right.

import { Part, Scene, box, tube, torus, disc, blade, V } from './scene3d.js';

const C = {
  arm: '#6d7b8a',
  motor: '#d8842f',
  prop: '#4a5c6e',
  guard: '#3f8f7f',
  hub: '#5a6675',
  battery: '#c25a4a',
  cell: '#d9705e',
  pi: '#2f7d4f',
  fc: '#8a5cc4',
  driver: '#2f6d8f',
  camera: '#c9a227',
  gimbal: '#9aa5b1',
  screen: '#cfd8e3',
  disc: '#5ec8f2',
  bec: '#7a6a4a',
};

const CELL = {
  '21700 high energy': [0.0211, 0.0704, 0.070],
  '18650 high energy': [0.0185, 0.0650, 0.048],
  'LiPo pouch': [0.034, 0.0600, 0.055],
};

const mm = (x) => `${Math.round(x * 1000)} mm`;
const gg = (x) => `${Math.round(x * 1000)} g`;

/**
 * Build every part of the aircraft from a /api/spec result.
 * Returns {parts, span, bom} where bom is the flat list for the table.
 */
export function buildAssembly(spec, params) {
  const b = spec.breakdown;
  const arm = b.arm, pr = b.propulsion, hov = b.rotor.hover;
  const N = Math.round(params.N_rotors);
  const R = params.D_rotor / 2;
  const L = arm.L;
  const rArm = params.arm_radius;
  const parts = [];
  const bom = [];

  const add = (p, row) => { parts.push(p); if (row) bom.push(row); };

  // ---- rotors, arms, motors, guards ------------------------------------
  const psi = (i) => (2 * Math.PI * i) / N + Math.PI / N;
  // Motor body sized from its mass at an effective 3000 kg/m^3 packing.
  const mVol = pr.m_motor_each / 3000;
  const mD = Math.cbrt(mVol / 0.353);
  const mH = 0.45 * mD;

  const armFaces = [], motorFaces = [], propFaces = [], guardFaces = [], discFaces = [];
  for (let i = 0; i < N; i++) {
    const a = psi(i);
    const hub = [L * Math.cos(a), L * Math.sin(a), 0];
    armFaces.push(...tube([0, 0, 0], hub, rArm, C.arm, 10));
    motorFaces.push(...tube([hub[0], hub[1], 0], [hub[0], hub[1], mH], mD / 2, C.motor, 14));
    const spin = i % 2 === 0 ? 1 : -1;
    for (let k = 0; k < Math.round(params.n_blades); k++) {
      const ang = a + (2 * Math.PI * k) / params.n_blades + (spin > 0 ? 0 : 0.4);
      propFaces.push(...blade([hub[0], hub[1], mH + 0.006], ang, R, hov.chord, C.prop, spin * 0.22));
    }
    guardFaces.push(...torus([hub[0], hub[1], mH * 0.5], R * 1.04, 0.003, C.guard, 20, 4));
    discFaces.push(...disc([hub[0], hub[1], mH + 0.004], R, C.disc, 30, 0.055));
  }
  const a0 = psi(0);
  add(new Part('arms', armFaces, [L * 0.55 * Math.cos(a0), L * 0.55 * Math.sin(a0), 0],
    [`${N} arms, ${mm(L)} each`, `${mm(2 * rArm)} tube, ${arm.wall * 1000 < 1 ? (arm.wall * 1000).toFixed(2) : (arm.wall * 1000).toFixed(1)} mm wall, ${gg(arm.m_arms)}`],
    { key: 'arms' }),
    { group: 'Frame', item: `${N} carbon arms`, spec: `${mm(L)} long, ${mm(2 * rArm)} OD, ${(arm.wall * 1000).toFixed(2)} mm wall`, mass: arm.m_arms, note: `${arm.driver} sized, first mode ${arm.f_bending.toFixed(0)} Hz` });

  add(new Part('motors', motorFaces, [L * Math.cos(a0), L * Math.sin(a0), mH],
    [`${N} motors, ${gg(pr.m_motor_each)} each`, `${mm(mD)} dia, ${spec.electrical.chosen ? Math.round(spec.electrical.chosen.KV) + ' KV' : ''}, ${hov.Q.toFixed(3)} N.m`],
    { key: 'motors', explode: [0, 0, 0.10] }),
    { group: 'Propulsion', item: `${N} motors`, spec: `${spec.electrical.chosen ? Math.round(spec.electrical.chosen.KV) : '?'} KV, ${hov.Q.toFixed(3)} N.m hover, ${pr.Q_max.toFixed(3)} N.m peak`, mass: pr.m_motors, note: 'custom low-KV wind, not a catalogue part' });

  add(new Part('props', propFaces, [L * Math.cos(a0) + R * 0.6, L * Math.sin(a0), mH],
    [`${N} x ${mm(params.D_rotor)} propellers`, `${params.n_blades}-blade, ${mm(hov.chord)} chord, ${hov.rpm.toFixed(0)} rpm`],
    { key: 'props', explode: [0, 0, 0.16] }),
    { group: 'Propulsion', item: `${N} propellers`, spec: `${mm(params.D_rotor)} dia, ${params.n_blades}-blade, ${mm(hov.chord)} chord (AR ${params.blade_AR.toFixed(1)})`, mass: pr.m_props, note: `${hov.rpm.toFixed(0)} rpm hover, ${hov.v_tip.toFixed(1)} m/s tip` });

  add(new Part('guards', guardFaces, [L * Math.cos(a0) - R, L * Math.sin(a0), mH * 0.5],
    ['prop guards', `${N} rings, ${gg(arm.m_guards)} total`], { key: 'guards', explode: [0, 0, 0.05] }),
    { group: 'Frame', item: 'prop guard rings', spec: `${N} x ${mm(2 * R * 1.04)} rings, 6 mm carbon tube`, mass: arm.m_guards, note: 'not optional beside a head' });

  add(new Part('disc', discFaces, [0, 0, mH], [], { key: 'disc' }));

  // ---- hub, boards, battery --------------------------------------------
  const hubW = Math.max(0.12, L * 0.55);
  add(new Part('hub', box([0, 0, -0.004], [hubW, hubW * 0.72, 0.008], C.hub), [0, -hubW * 0.36, 0],
    ['centre plate', `hub, mounts, wiring ${gg(arm.m_frame - arm.m_arms - arm.m_guards)}`], { key: 'hub' }),
    { group: 'Frame', item: 'centre plate, mounts, wiring', spec: `${mm(hubW)} across`, mass: arm.m_frame - arm.m_arms - arm.m_guards, note: '' });

  // Battery: draw the actual cells.
  const ch = spec.electrical.chosen;
  if (ch) {
    const cell = CELL[ch.cell] || CELL['21700 high energy'];
    const [cd, cl] = cell;
    const cells = [];
    const S = ch.S;
    const startY = -((S - 1) * cd * 1.06) / 2;
    for (let i = 0; i < S; i++) {
      const y = startY + i * cd * 1.06;
      cells.push(...tube([-cl / 2, y, -0.028], [cl / 2, y, -0.028], cd / 2, C.cell, 12));
    }
    add(new Part('battery', cells, [0, 0, -0.028],
      [`${ch.S}S1P ${ch.cell.split(' ')[0]} pack`, `${gg(spec.m_bat)}, ${ch.energy_wh.toFixed(1)} Wh, ${ch.v_min.toFixed(1)}-${ch.v_full.toFixed(1)} V`],
      { key: 'battery', explode: [0, 0, -0.16] }),
      { group: 'Power', item: `battery, ${ch.S}S1P ${ch.cell}`, spec: `${ch.S} cells, ${ch.energy_wh.toFixed(1)} Wh, ${ch.I_hover.toFixed(2)} A/motor at hover`, mass: spec.m_bat, note: `${ch.cont_w_kg.toFixed(0)} W/kg continuous of ${ch.cont_limit.toFixed(0)} rated` });
  }

  // Boards on the centre plate.
  const boards = [
    ['pi', 'Raspberry Pi 4B', [0.085, 0.056, 0.020], [0.028, 0.030, 0.014], C.pi, 0.046,
      'HDMI to the panel, remote desktop client, follow logic'],
    ['fc', 'Pixhawk 6C mini', [0.0593, 0.0353, 0.0136], [-0.040, 0.028, 0.012], C.fc, 0.035,
      'attitude and rate loops at 1 kHz, ArduCopter or PX4'],
    ['driver', 'eDP controller board', [0.120, 0.055, 0.010], [0.0, -0.036, 0.011], C.driver, 0.030,
      'HDMI in, eDP out, firmware matched to the panel'],
    ['bec', '5 V and 12 V regulators', [0.045, 0.028, 0.010], [-0.048, -0.026, 0.011], C.bec, 0.030,
      'the panel board wants 12 V, the Pi wants 5'],
  ];
  for (const [key, name, size, pos, color, mass, note] of boards) {
    add(new Part(key, box(pos, size, color), [pos[0], pos[1], pos[2] + size[2] / 2],
      [name, `${mm(size[0])} x ${mm(size[1])}, ${gg(mass)}`],
      { key, explode: [pos[0] * 1.6, pos[1] * 1.6, 0.22] }),
      { group: key === 'driver' ? 'Display' : key === 'bec' ? 'Power' : 'Compute', item: name, spec: `${mm(size[0])} x ${mm(size[1])} x ${mm(size[2])}`, mass, note });
  }

  // Camera looking forward.
  add(new Part('camera', box([0.052, 0, 0.004], [0.014, 0.030, 0.014], C.camera), [0.059, 0, 0.010],
    ['AI camera', 'on-sensor detection, 25 g'], { key: 'camera', explode: [0.22, 0, 0.06] }),
    { group: 'Compute', item: 'camera with on-sensor inference', spec: 'IMX500 AI Camera or OAK-D-Lite', mass: 0.025, note: 'outputs coordinates, not frames, so the Pi stays free to decode video' });

  // ---- gimbal and screen -------------------------------------------------
  // Positive z is up, so the screen sits above the rotor plane when
  // screen_above is set. The user faces the machine from -x.
  const up = params.screen_above >= 0.5 ? 1 : -1;
  const zs = up * params.r_cp;
  const fwd = -(params.screen_forward || 0);   // toward the user
  const sw = params.screen_w, sh = params.screen_h;
  const mastTop = zs - up * (sh / 2 + 0.02);
  const gimbal = [];
  gimbal.push(...tube([0, 0, up * 0.008], [fwd, 0, mastTop], 0.005, C.gimbal, 8));
  gimbal.push(...box([fwd, 0, mastTop], [0.030, 0.024, 0.016], C.gimbal));
  gimbal.push(...box([fwd, -0.020, mastTop + up * 0.014], [0.024, 0.016, 0.024], C.gimbal));
  gimbal.push(...box([fwd + 0.020, 0, mastTop + up * 0.030], [0.016, 0.024, 0.024], C.gimbal));
  add(new Part('gimbal', gimbal, [fwd, 0, (zs + mastTop) / 2],
    ['3-axis gimbal', `${gg(params.m_gimbal)}, yaw axis carries the heading`],
    { key: 'gimbal', explode: [0, 0, up * 0.08] }),
    { group: 'Display', item: '3-axis gimbal', spec: 'pitch, roll and yaw', mass: params.m_gimbal, note: 'yaw on the gimbal because the airframe has almost no yaw authority' });

  if (params.screen_forward > 1e-6) {
    add(new Part('boom', tube([0, 0, up * 0.006], [fwd, 0, up * 0.006], 0.007, C.gimbal, 8),
      [fwd / 2, 0, up * 0.006],
      [`${mm(params.screen_forward)} boom`, 'costs pitch inertia; a longer standoff is cheaper'],
      { key: 'boom' }),
      { group: 'Frame', item: 'screen boom', spec: `${mm(params.screen_forward)} cantilever`, mass: spec.breakdown.arm.m_boom || 0, note: 'doubles pitch inertia; prefer a longer standoff' });
  }

  const screenFaces = box([fwd, 0.003, zs], [sw, 0.006, sh], C.screen);
  add(new Part('screen', screenFaces, [fwd - sw * 0.42, 0, zs + up * sh * 0.3],
    [`${mm(sw)} x ${mm(sh)} panel`, `${spec.readability ? spec.readability.diagonal_in.toFixed(1) : ''} in, ${gg(params.m_screen)}, ${spec.readability ? spec.readability.pixels_per_degree.toFixed(0) : ''} px/deg`],
    { key: 'screen', explode: [0, -0.30, up * 0.06] }),
    { group: 'Display', item: 'bare eDP panel', spec: `${mm(sw)} x ${mm(sh)}, ${spec.readability ? spec.readability.diagonal_in.toFixed(1) : '?'} in, 1920 x 1080`, mass: params.m_screen - 0.030 - 0.035, note: 'BOE NV156FHM-N4M class, buy by panel model number' });

  bom.push({ group: 'Display', item: 'carbon backing frame', spec: 'panel to gimbal interface', mass: 0.035, note: '' });

  const span = 2 * (L + R);
  return { parts, span, bom, up, reach: L + R };
}

/**
 * A head at the working distance, and the circle the blade tips sweep.
 * Drawn to the same scale as the machine, because the whole safety question
 * is a geometric one that a table of numbers hides.
 */
export function buildUserReference(spec, params, reach) {
  const parts = [];
  const geo = spec.geometry || {};
  const up = params.screen_above >= 0.5 ? 1 : -1;
  const zs = up * params.r_cp;
  // The user stands at -x, `standoff` from the screen, eyes level with it.
  const ux = -(params.standoff + (params.screen_forward || 0));
  const eyeZ = zs;

  const head = [];
  const seg = 10;
  for (let i = 0; i < seg; i++) {
    const a0 = (Math.PI * i) / seg - Math.PI / 2, a1 = (Math.PI * (i + 1)) / seg - Math.PI / 2;
    head.push(...tube([ux, 0, eyeZ + 0.10 * Math.sin(a0)], [ux, 0, eyeZ + 0.10 * Math.sin(a1)],
      0.098 * Math.max(Math.cos(a0), 0.05), '#7f8b9a', 10, false));
  }
  head.push(...tube([ux, 0, eyeZ - 0.11], [ux, 0, eyeZ - 0.55], 0.055, '#66707c', 10));
  parts.push(new Part('user', head, [ux, 0, eyeZ + 0.10],
    ['you', `${mm(params.standoff)} to the screen, eyes level with it`], { key: 'user' }));

  // The circle the tips sweep, and the keep-out sphere around the head.
  const tipRing = torus([0, 0, 0], reach, 0.004, '#f28b82', 40, 3);
  const gap = (params.standoff + (params.screen_forward || 0)) - reach;
  parts.push(new Part('reach', tipRing, [0, -reach, 0],
    [`blade tips reach ${mm(reach)}`, `${mm(gap)} of clear air to your face`],
    { key: 'reach' }));
  return parts;
}

export function makeScene(canvas, spec, params, opts = {}) {
  const scene = new Scene(canvas);
  const { parts, span, bom, up, reach } = buildAssembly(spec, params);
  scene.parts = parts.slice();
  scene.userParts = buildUserReference(spec, params, reach);
  scene.showUser = !!opts.showUser;
  if (scene.showUser) scene.parts.push(...scene.userParts);
  scene.setUser = (on) => {
    scene.showUser = on;
    scene.parts = on ? parts.concat(scene.userParts) : parts.slice();
    scene.dist = (on ? span * 2.1 + params.standoff * 1.5 : span * 2.1);
    scene.target = on
      ? [-(params.standoff + (params.screen_forward || 0)) / 2, 0, up * params.r_cp * 0.5]
      : [0, 0, up * params.r_cp * 0.45];
    scene.draw();
  };
  scene.dist = span * 2.1;
  scene.target = [0, 0, up * params.r_cp * 0.45];
  return { scene, bom, span, reach };
}
