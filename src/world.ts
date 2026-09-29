// MJCF for the intersection: ground, kerbs, buildings, poles, signal masts, and a pool of
// agents (cars, trucks, motorcycles and bicycles with riders, pedestrians).
// Agents that are not on stage are "parked": floating far away with gravity compensation on and
// collisions off, so they cost almost nothing. The sim spawns them by teleporting.
import {
  buildings, kerbSegments, signalMasts, streetTrees, utilityPoles,
} from './layout.ts';

export const SIM_DT = 0.004; // 250 Hz

export type Kind = 'car' | 'truck' | 'moto' | 'bicycle' | 'ped';

export interface KindSpec {
  len: number; // overall length [m]
  wid: number; // overall width [m]
  r: number; // wheel radius (0 for pedestrians)
  wb: number; // wheelbase
  tw: number; // track width
  accel: number; // comfortable acceleration [m/s²]
  decel: number; // maximum braking [m/s²]
  eye: number; // driver eye height above the road [m]
  maxSteer: number;
}

export const KIND: Record<Kind, KindSpec> = {
  car: { len: 4.6, wid: 1.75, r: 0.32, wb: 2.7, tw: 1.52, accel: 2.4, decel: 7, eye: 1.2, maxSteer: 0.6 },
  truck: { len: 7.6, wid: 2.3, r: 0.5, wb: 4.2, tw: 1.9, accel: 1.3, decel: 5.5, eye: 2.3, maxSteer: 0.55 },
  moto: { len: 2.1, wid: 0.8, r: 0.31, wb: 1.45, tw: 0, accel: 3.2, decel: 7, eye: 1.45, maxSteer: 0.5 },
  bicycle: { len: 1.75, wid: 0.6, r: 0.34, wb: 1.05, tw: 0, accel: 1.0, decel: 3.5, eye: 1.55, maxSteer: 0.6 },
  ped: { len: 0.45, wid: 0.5, r: 0, wb: 0, tw: 0, accel: 1.2, decel: 2.5, eye: 1.6, maxSteer: 0 },
};

export interface PoolSizes { car: number; truck: number; moto: number; bicycle: number; ped: number }
export const DEFAULT_POOL: PoolSizes = { car: 18, truck: 4, moto: 6, bicycle: 5, ped: 14 };

export interface AgentDef {
  kind: Kind;
  name: string; // root body name; a rider is `${name}_rider`
  ego: boolean;
  park: [number, number, number];
  color: string;
}

type Attrs = Record<string, string | number>;

class El {
  children: El[] = [];
  tag: string;
  attrs: Attrs;
  constructor(tag: string, attrs: Attrs = {}) {
    this.tag = tag;
    this.attrs = attrs;
  }
  sub(tag: string, attrs: Attrs = {}): El {
    const e = new El(tag, attrs);
    this.children.push(e);
    return e;
  }
  toString(indent = ''): string {
    const a = Object.entries(this.attrs)
      .map(([k, v]) => ` ${k}="${v}"`)
      .join('');
    if (!this.children.length) return `${indent}<${this.tag}${a}/>`;
    const inner = this.children.map((c) => c.toString(indent + ' ')).join('\n');
    return `${indent}<${this.tag}${a}>\n${inner}\n${indent}</${this.tag}>`;
  }
}

const v3 = (a: number, b: number, c: number) => `${a} ${b} ${c}`;
const f = (n: number) => +n.toFixed(4);

export const EGO_COLOR = '0.98 0.5 0.08 1';
const CAR_COLORS = [
  '0.93 0.93 0.93 1', '0.72 0.73 0.75 1', '0.08 0.08 0.1 1', '0.62 0.08 0.1 1', '0.12 0.22 0.5 1',
  '0.85 0.84 0.8 1', '0.3 0.32 0.35 1', '0.15 0.35 0.3 1', '0.55 0.5 0.42 1', '0.95 0.95 0.9 1',
];
const TRUCK_COLORS = ['0.95 0.95 0.95 1', '0.2 0.45 0.75 1', '0.9 0.9 0.85 1', '0.25 0.55 0.3 1'];
const MOTO_COLORS = ['0.75 0.05 0.08 1', '0.1 0.1 0.12 1', '0.1 0.3 0.7 1', '0.95 0.95 0.95 1', '0.1 0.5 0.2 1', '0.9 0.6 0.05 1'];
const SHIRTS = [
  '0.85 0.2 0.2 1', '0.2 0.4 0.8 1', '0.95 0.95 0.95 1', '0.25 0.6 0.3 1', '0.95 0.75 0.2 1',
  '0.5 0.3 0.6 1', '0.1 0.1 0.12 1', '0.9 0.5 0.6 1', '0.3 0.7 0.8 1', '0.6 0.45 0.3 1',
];
const PANTS = ['0.15 0.18 0.3 1', '0.1 0.1 0.1 1', '0.4 0.35 0.3 1', '0.25 0.25 0.28 1'];
const SKIN = '0.93 0.78 0.66 1';
const GLASS = '0.12 0.16 0.22 1';
const DARK = '0.1 0.1 0.11 1';
const TIRE = '0.08 0.08 0.09 1';
const RIM = '0.7 0.72 0.75 1';

/** Named geom helper. Collidable geoms get contype/conaffinity 1. */
function geom(body: El, name: string, type: string, attrs: Attrs, rgba: string, mass: number, collide: boolean) {
  body.sub('geom', {
    name, type, ...attrs, rgba, mass,
    contype: collide ? 1 : 0, conaffinity: collide ? 1 : 0,
  });
}

// ── Four-wheeled vehicles ─────────────────────────────────

function wheeledVehicle(wb: El, act: El, def: AgentDef) {
  const k = KIND[def.kind];
  const n = def.name;
  const [px, py, pz] = def.park;
  const body = wb.sub('body', { name: n, pos: v3(px, py, pz), gravcomp: 1 });
  body.sub('freejoint', { name: `${n}_free` });
  const c = def.color;
  const box = (g: string, pos: string, size: string, rgba: string, mass: number, collide = false) =>
    geom(body, `${n}_${g}`, 'box', { pos, size }, rgba, mass, collide);

  if (def.kind === 'car') {
    // Origin at axle height (0.32 m above the road)
    box('body', '0 0 0.18', '2.28 0.87 0.22', c, 800, true);
    box('hood', '1.5 0 0.44', '0.78 0.84 0.06', c, 60);
    box('trunk', '-1.86 0 0.46', '0.42 0.84 0.07', c, 40);
    box('belt', '-0.3 0 0.52', '1.12 0.87 0.12', c, 100, true);
    box('cabin', '-0.32 0 0.9', '1.02 0.8 0.26', GLASS, 100, true);
    box('roof', '-0.42 0 1.17', '0.86 0.81 0.03', c, 10);
    box('dash', '0.62 0 0.62', '0.18 0.78 0.06', DARK, 5);
    box('bumper_f', '2.3 0 0.02', '0.06 0.86 0.12', DARK, 5);
    box('bumper_r', '-2.3 0 0.02', '0.06 0.86 0.12', DARK, 5);
    box('grille', '2.29 0 0.26', '0.02 0.4 0.07', DARK, 1);
    box('lamp_fl', '2.29 0.62 0.3', '0.02 0.18 0.05', '1 1 0.92 1', 0.5);
    box('lamp_fr', '2.29 -0.62 0.3', '0.02 0.18 0.05', '1 1 0.92 1', 0.5);
    box('tail_l', '-2.29 0.66 0.34', '0.02 0.16 0.06', '0.85 0.05 0.05 1', 0.5);
    box('tail_r', '-2.29 -0.66 0.34', '0.02 0.16 0.06', '0.85 0.05 0.05 1', 0.5);
    box('mirror_l', '0.62 0.93 0.66', '0.06 0.08 0.05', c, 0.5);
    box('mirror_r', '0.62 -0.93 0.66', '0.06 0.08 0.05', c, 0.5);
  } else {
    // Box truck. Origin at axle height (0.5 m).
    box('frame', '-0.4 0 0.05', '3.4 0.5 0.12', DARK, 1400, true);
    box('cab', '2.35 0 0.95', '0.85 1.12 0.95', c, 1300, true);
    box('windshield', '3.21 0 1.4', '0.02 0.98 0.36', GLASS, 1);
    box('side_win', '2.55 0 1.4', '0.42 1.125 0.3', GLASS, 1);
    box('cargo', '-1.35 0 1.55', '2.8 1.16 1.25', c === TRUCK_COLORS[1] ? '0.92 0.92 0.92 1' : c, 2500, true);
    box('stripe_l', '-1.35 1.165 1.0', '2.7 0.005 0.08', '0.2 0.4 0.75 1', 1);
    box('stripe_r', '-1.35 -1.165 1.0', '2.7 0.005 0.08', '0.2 0.4 0.75 1', 1);
    box('bumper_f', '3.22 0 0.15', '0.06 1.1 0.14', DARK, 10);
    box('lamp_fl', '3.23 0.8 0.35', '0.02 0.18 0.07', '1 1 0.92 1', 0.5);
    box('lamp_fr', '3.23 -0.8 0.35', '0.02 0.18 0.07', '1 1 0.92 1', 0.5);
    box('tail_l', '-4.16 0.9 0.4', '0.02 0.12 0.08', '0.85 0.05 0.05 1', 0.5);
    box('tail_r', '-4.16 -0.9 0.4', '0.02 0.12 0.08', '0.85 0.05 0.05 1', 0.5);
  }

  const wb2 = k.wb / 2, tw2 = k.tw / 2;
  const heavy = def.kind === 'truck';
  for (const [w, x, y, steer] of [
    ['fl', wb2, tw2, true], ['fr', wb2, -tw2, true], ['rl', -wb2, tw2, false], ['rr', -wb2, -tw2, false],
  ] as const) {
    const wn = `${n}_w_${w}`;
    const wbody = body.sub('body', { name: wn, pos: v3(x, y, 0), gravcomp: 1 });
    if (steer) {
      wbody.sub('joint', { name: `${wn}_steer`, type: 'hinge', axis: '0 0 1', range: `${-k.maxSteer} ${k.maxSteer}`, damping: heavy ? 400 : 60 });
    }
    wbody.sub('joint', { name: `${wn}_roll`, type: 'hinge', axis: '0 1 0', damping: 0.5 });
    geom(wbody, `${wn}_contact`, 'sphere', { size: k.r, friction: '1.0 0.02 0.0005' }, '0 0 0 0', heavy ? 60 : 15, true);
    const hw = heavy ? (steer ? 0.16 : 0.26) : 0.1;
    geom(wbody, `${wn}_tire`, 'cylinder', { size: `${k.r} ${hw}`, euler: '1.5708 0 0' }, TIRE, 0.1, false);
    geom(wbody, `${wn}_rim`, 'cylinder', { size: `${f(k.r * 0.6)} ${hw + 0.008}`, euler: '1.5708 0 0' }, RIM, 0.1, false);
  }

  if (def.ego) {
    // Right-hand drive: the driver sits on the right.
    body.sub('camera', { name: 'driver_cam', pos: '-0.05 -0.38 0.88', xyaxes: '0 -1 0 0.02 0 1', fovy: 62 });
    body.sub('camera', { name: 'chase_cam', pos: '-9 0 3.6', xyaxes: '0 -1 0 0.33 0 0.94', fovy: 55 });
  }

  const kp = heavy ? 60000 : 8000;
  for (const w of ['fl', 'fr']) {
    act.sub('position', { name: `${n}_steer_${w}`, joint: `${n}_w_${w}_steer`, kp, ctrlrange: `${-k.maxSteer} ${k.maxSteer}` });
  }
  const kv = heavy ? 3000 : 450;
  for (const w of ['fl', 'fr', 'rl', 'rr']) {
    act.sub('velocity', { name: `${n}_drive_${w}`, joint: `${n}_w_${w}_roll`, kv, ctrlrange: '-100 100' }); // 100 rad/s ≈ 115 km/h for a car
  }
}

// ── Humans (pedestrians and riders) ───────────────────────

export interface Pose { hip: number; knee: number; shoulder: number }
export const SEATED_MOTO: Pose = { hip: -1.3, knee: 1.5, shoulder: -1.1 };
export const SEATED_BIKE: Pose = { hip: -0.55, knee: 0.75, shoulder: -0.9 };
export const STAND: Pose = { hip: 0, knee: 0.05, shoulder: 0 };
export const PELVIS_H = 0.95; // standing pelvis height above the soles

function human(parent: El, act: El, name: string, pos: string, shirt: string, pants: string, pose: Pose, free: boolean) {
  const body = parent.sub('body', { name, pos, gravcomp: 1 });
  if (free) body.sub('freejoint', { name: `${name}_free` });
  const fr = { friction: '0.8 0.02 0.001' };
  geom(body, `${name}_torso`, 'capsule', { fromto: '0 0 0.1 0 0 0.48', size: 0.15, ...fr }, shirt, 30, true);
  geom(body, `${name}_hips`, 'capsule', { fromto: '0 0.1 0.02 0 -0.1 0.02', size: 0.1, ...fr }, pants, 4, true);
  geom(body, `${name}_head`, 'sphere', { pos: '0 0 0.74', size: 0.11, ...fr }, SKIN, 5, true);
  geom(body, `${name}_hair`, 'sphere', { pos: '-0.015 0 0.77', size: 0.105 }, '0.12 0.09 0.07 1', 0.1, false);
  for (const [s, y] of [['l', 0.1], ['r', -0.1]] as const) {
    const thigh = body.sub('body', { name: `${name}_thigh_${s}`, pos: v3(0, y, 0), gravcomp: 1 });
    thigh.sub('joint', { name: `${name}_hip_${s}`, type: 'hinge', axis: '0 1 0', range: '-2.0 1.2', damping: 6, ref: 0 });
    geom(thigh, `${name}_thigh_${s}_g`, 'capsule', { fromto: '0 0 0 0 0 -0.43', size: 0.075, ...fr }, pants, 7, true);
    const shin = thigh.sub('body', { name: `${name}_shin_${s}`, pos: '0 0 -0.45', gravcomp: 1 });
    shin.sub('joint', { name: `${name}_knee_${s}`, type: 'hinge', axis: '0 1 0', range: '0 2.4', damping: 4 });
    geom(shin, `${name}_shin_${s}_g`, 'capsule', { fromto: '0 0 0 0 0 -0.41', size: 0.06, ...fr }, pants, 3.5, true);
    geom(shin, `${name}_foot_${s}`, 'box', { pos: '0.06 0 -0.47', size: '0.11 0.045 0.03', ...fr }, DARK, 0.8, true);
    const arm = body.sub('body', { name: `${name}_arm_${s}`, pos: v3(0, 2.2 * y, 0.45), gravcomp: 1 });
    arm.sub('joint', { name: `${name}_sh_${s}`, type: 'hinge', axis: '0 1 0', range: '-3 1.5', damping: 2 });
    geom(arm, `${name}_arm_${s}_g`, 'capsule', { fromto: '0 0 0 0 0 -0.56', size: 0.045, ...fr }, shirt, 3, true);
    geom(arm, `${name}_hand_${s}`, 'sphere', { pos: '0 0 -0.6', size: 0.045 }, SKIN, 0.3, false);
  }
  for (const s of ['l', 'r']) {
    act.sub('position', { name: `${name}_hip_${s}`, joint: `${name}_hip_${s}`, kp: 350, ctrlrange: '-2 1.2' });
    act.sub('position', { name: `${name}_knee_${s}`, joint: `${name}_knee_${s}`, kp: 180, ctrlrange: '0 2.4' });
    act.sub('position', { name: `${name}_sh_${s}`, joint: `${name}_sh_${s}`, kp: 60, ctrlrange: '-3 1.5' });
  }
  void pose;
  return body;
}

/** Body names of a human, for contact exclusion with the bike it rides. */
const humanBodies = (n: string) =>
  [n, `${n}_thigh_l`, `${n}_shin_l`, `${n}_arm_l`, `${n}_thigh_r`, `${n}_shin_r`, `${n}_arm_r`];

// ── Two-wheelers ──────────────────────────────────────────

function twoWheeler(wb: El, act: El, contact: El, eq: El, def: AgentDef, shirt: string) {
  const k = KIND[def.kind];
  const n = def.name;
  const moto = def.kind === 'moto';
  const [px, py, pz] = def.park;
  const body = wb.sub('body', { name: n, pos: v3(px, py, pz), gravcomp: 1 });
  body.sub('freejoint', { name: `${n}_free` });
  const c = def.color;
  const box = (g: string, pos: string, size: string, rgba: string, mass: number, collide = false) =>
    geom(body, `${n}_${g}`, 'box', { pos, size }, rgba, mass, collide);
  const cap = (g: string, fromto: string, r: number, rgba: string, mass: number, collide = false) =>
    geom(body, `${n}_${g}`, 'capsule', { fromto, size: r }, rgba, mass, collide);
  const front = k.wb / 2, rear = -k.wb / 2;

  if (moto) {
    box('frame', '0 0 0.22', '0.45 0.1 0.12', DARK, 80, true);
    box('engine', '0 0 0.02', '0.24 0.15 0.14', '0.35 0.35 0.37 1', 50, true);
    box('tank', '0.18 0 0.46', '0.24 0.16 0.1', c, 10, true);
    box('seat', '-0.28 0 0.5', '0.28 0.13 0.05', DARK, 4);
    box('tail', '-0.6 0 0.47', '0.18 0.1 0.07', c, 3);
    box('cowl', '0.55 0 0.55', '0.1 0.16 0.16', c, 3);
    box('taillamp', '-0.78 0 0.47', '0.02 0.07 0.03', '0.85 0.05 0.05 1', 0.2);
    cap('muffler', `${rear + 0.1} -0.2 0.08 0.15 -0.2 0.02`, 0.06, '0.6 0.6 0.62 1', 3);
  } else {
    cap('down', `0.35 0 0.45 -0.05 0 0.05`, 0.022, c, 3, true);
    cap('top', `0.35 0 0.5 -0.2 0 0.5`, 0.02, c, 2, true);
    cap('seat_tube', `-0.05 0 0.05 -0.2 0 0.55`, 0.022, c, 2);
    cap('stay', `-0.05 0 0.05 ${rear} 0 0`, 0.018, c, 1);
    cap('seat_stay', `-0.2 0 0.52 ${rear} 0 0`, 0.016, c, 1);
    box('saddle', '-0.22 0 0.6', '0.12 0.06 0.03', DARK, 1);
    box('basket', '0.66 0 0.55', '0.16 0.17 0.1', '0.6 0.6 0.62 1', 1);
  }

  // Fork (steers) carrying the front wheel; rear wheel on the frame.
  const fork = body.sub('body', { name: `${n}_fork`, pos: v3(front, 0, 0), gravcomp: 1 });
  fork.sub('joint', { name: `${n}_steer`, type: 'hinge', axis: '0 0 1', range: `${-k.maxSteer} ${k.maxSteer}`, damping: 5 });
  geom(fork, `${n}_forkleg`, 'capsule', { fromto: '0 0 0 -0.16 0 0.62' , size: moto ? 0.035 : 0.018 }, moto ? '0.6 0.6 0.62 1' : c, 2, false);
  geom(fork, `${n}_bar`, 'capsule', { fromto: `-0.2 ${moto ? 0.36 : 0.3} 0.66 -0.2 ${moto ? -0.36 : -0.3} 0.66`, size: 0.018 }, DARK, 1, true);
  if (moto) geom(fork, `${n}_headlamp`, 'sphere', { pos: '-0.05 0 0.55', size: 0.08 }, '1 1 0.9 1', 0.5, false);
  const wheel = (parent: El, wn: string, pos: string) => {
    const w = parent.sub('body', { name: wn, pos, gravcomp: 1 });
    w.sub('joint', { name: `${wn}_roll`, type: 'hinge', axis: '0 1 0', damping: 0.05 });
    geom(w, `${wn}_contact`, 'sphere', { size: k.r, friction: '1.0 0.02 0.0005' }, '0 0 0 0', moto ? 8 : 1, true);
    geom(w, `${wn}_tire`, 'cylinder', { size: `${k.r} ${moto ? 0.07 : 0.02}`, euler: '1.5708 0 0' }, TIRE, 0.1, false);
    geom(w, `${wn}_rim`, 'cylinder', { size: `${f(k.r * (moto ? 0.62 : 0.88))} ${moto ? 0.075 : 0.024}`, euler: '1.5708 0 0' }, moto ? RIM : '0.8 0.8 0.82 1', 0.1, false);
  };
  wheel(fork, `${n}_wf`, '0 0 0');
  wheel(body, `${n}_wr`, v3(rear, 0, 0));

  act.sub('position', { name: `${n}_steer`, joint: `${n}_steer`, kp: moto ? 400 : 60, ctrlrange: `${-k.maxSteer} ${k.maxSteer}` });
  act.sub('velocity', { name: `${n}_drive`, joint: `${n}_wr_roll`, kv: moto ? 40 : 4, ctrlrange: '-80 80' });

  // Rider: a separate free body welded to the bike; the weld is released on impact.
  const rn = `${n}_rider`;
  const seat = moto ? [-0.3, 0.6] : [-0.24, 0.66];
  const pose = moto ? SEATED_MOTO : SEATED_BIKE;
  const rider = human(wb, act, rn, v3(px + seat[0], py, pz + seat[1]), shirt, PANTS[def.name.length % PANTS.length], pose, true);
  if (moto) geom(rider, `${rn}_helmet`, 'sphere', { pos: '0 0 0.75', size: 0.14 }, def.color, 0.5, false);
  eq.sub('weld', { name: `${n}_seat`, body1: rn, body2: n, solref: '0.01 1' });
  for (const a of humanBodies(rn)) {
    for (const b of [n, `${n}_fork`, `${n}_wf`, `${n}_wr`]) contact.sub('exclude', { body1: a, body2: b });
  }
}

// ── Whole world ───────────────────────────────────────────

export function agentDefs(pool: PoolSizes): AgentDef[] {
  const defs: AgentDef[] = [];
  let i = 0;
  const park = (): [number, number, number] => {
    const p: [number, number, number] = [-300 + (i % 20) * 14, -320 - Math.floor(i / 20) * 14, 3];
    i++;
    return p;
  };
  defs.push({ kind: 'car', name: 'ego', ego: true, park: park(), color: EGO_COLOR });
  for (let j = 0; j < pool.car; j++) defs.push({ kind: 'car', name: `car${j}`, ego: false, park: park(), color: CAR_COLORS[j % CAR_COLORS.length] });
  for (let j = 0; j < pool.truck; j++) defs.push({ kind: 'truck', name: `truck${j}`, ego: false, park: park(), color: TRUCK_COLORS[j % TRUCK_COLORS.length] });
  for (let j = 0; j < pool.moto; j++) defs.push({ kind: 'moto', name: `moto${j}`, ego: false, park: park(), color: MOTO_COLORS[j % MOTO_COLORS.length] });
  for (let j = 0; j < pool.bicycle; j++) defs.push({ kind: 'bicycle', name: `bike${j}`, ego: false, park: park(), color: MOTO_COLORS[(j + 3) % MOTO_COLORS.length] });
  for (let j = 0; j < pool.ped; j++) defs.push({ kind: 'ped', name: `ped${j}`, ego: false, park: park(), color: SHIRTS[j % SHIRTS.length] });
  return defs;
}

export function buildWorldXml(defs: AgentDef[]): string {
  const root = new El('mujoco', { model: 'intersection' });
  root.sub('compiler', { angle: 'radian' });
  root.sub('option', { gravity: '0 0 -9.81', timestep: SIM_DT, integrator: 'implicitfast' });
  root.sub('size', { memory: '128M' });
  const dflt = root.sub('default');
  dflt.sub('geom', { friction: '0.7 0.02 0.001', condim: 3 });

  const wb = root.sub('worldbody');
  wb.sub('geom', { name: 'ground', type: 'plane', size: '600 600 1', rgba: '0.35 0.36 0.36 1', contype: 1, conaffinity: 1, friction: '0.9 0.02 0.001' });
  wb.sub('light', { directional: 'true', diffuse: '0.8 0.78 0.72', pos: '0 0 300', dir: '-0.4 0.5 -1', castshadow: 'false' });
  wb.sub('light', { directional: 'true', diffuse: '0.25 0.27 0.33', pos: '0 0 300', dir: '0.5 -0.3 -1', castshadow: 'false' });

  // Kerbs (low, collidable) along the arms beyond the crosswalks and along the side streets;
  // the corners and side-street mouths are open.
  kerbSegments().forEach(([x0, y0, x1, y1], i) => {
    const hx = Math.max(0.1, Math.abs(x1 - x0) / 2), hy = Math.max(0.1, Math.abs(y1 - y0) / 2);
    wb.sub('geom', { name: `curb_${i}`, type: 'box', pos: v3(f((x0 + x1) / 2), f((y0 + y1) / 2), 0.06), size: v3(f(hx), f(hy), 0.06), rgba: '0.72 0.72 0.7 1' });
  });

  // Buildings (drawn with facades by the renderer; collidable here).
  buildings().forEach((b, i) => {
    wb.sub('geom', { name: `bld_${i}`, type: 'box', pos: v3(f(b.x), f(b.y), f(b.h / 2)), size: v3(f(b.hx), f(b.hy), f(b.h / 2)), rgba: '0.8 0.8 0.8 1' });
  });
  streetTrees().forEach(([x, y], i) => {
    wb.sub('geom', { name: `tree_${i}`, type: 'cylinder', pos: v3(x, y, 1.6), size: '0.16 1.6', rgba: '0.36 0.27 0.19 1' });
  });
  utilityPoles().forEach(([x, y], i) => {
    wb.sub('geom', { name: `upole_${i}`, type: 'cylinder', pos: v3(x, y, 5.5), size: '0.17 5.5', rgba: '0.62 0.62 0.6 1' });
  });
  signalMasts().forEach((m, i) => {
    wb.sub('geom', { name: `sig_pole_${i}`, type: 'cylinder', pos: v3(m.x, m.y, 3), size: '0.14 3', rgba: '0.55 0.57 0.58 1' });
    const L = 10;
    wb.sub('geom', {
      name: `sig_arm_${i}`, type: 'capsule', size: 0.09,
      fromto: `${m.x} ${m.y} 5.6 ${f(m.x + m.armDir[0] * L)} ${f(m.y + m.armDir[1] * L)} 5.6`, rgba: '0.55 0.57 0.58 1',
    });
  });

  const act = root.sub('actuator');
  const contact = root.sub('contact');
  const eq = root.sub('equality');
  defs.forEach((d, i) => {
    if (d.kind === 'car' || d.kind === 'truck') wheeledVehicle(wb, act, d);
    else if (d.kind === 'moto' || d.kind === 'bicycle') twoWheeler(wb, act, contact, eq, d, SHIRTS[(i * 3) % SHIRTS.length]);
    else {
      const [px, py, pz] = d.park;
      human(wb, act, d.name, v3(px, py, pz + PELVIS_H), d.color, PANTS[i % PANTS.length], STAND, true);
    }
  });
  if (!contact.children.length) root.children.splice(root.children.indexOf(contact), 1);
  if (!eq.children.length) root.children.splice(root.children.indexOf(eq), 1);
  return root.toString();
}
