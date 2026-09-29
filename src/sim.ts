// Traffic simulation on top of MuJoCo: spawning, signals, driver / pedestrian controllers,
// gap acceptance with line-of-sight (mj_ray), the ego's figure-of-eight lap through the town with
// its lane changes, collision detection.
//
// Before an impact, cars and trucks drive on their own wheels (steering + wheel-speed actuators),
// while motorcycles, bicycles and pedestrians are steered by forces (xfrc_applied) along their paths.
// On impact every controller involved switches off: riders' welds are released, bodies go limp, and
// from then on MuJoCo alone decides what happens — spins, rollovers, riders thrown, secondary hits.
import type { DoubleBuffer, IntBuffer, MainModule, MjData, MjModel } from '@mujoco/mujoco';
import {
  ARM, HUG_LATEST, HUG_T0, INNER, ROAD_HALF, WALK_OUT, JUNCTION_POS, LAP, LC_EARLIEST, LC_LATEST, LEG_START, OUTER, PATH_STEP, Path, angleDiff,
  buildLeg, buildRoute, buildWalk, conflicts, rng, signalAt,
  type Arm, type Axis, type JunctionId, type SignalState, type Turn,
} from './layout.ts';
import {
  KIND, PELVIS_H, SEATED_BIKE, SEATED_MOTO, SIM_DT, STAND, agentDefs, buildWorldXml,
  type AgentDef, type Kind, type KindSpec, type PoolSizes, type Pose, DEFAULT_POOL,
} from './world.ts';

export interface Settings {
  oncoming: number; // oncoming (southbound) vehicles per minute
  cross: number; // cross-street vehicles per minute (both directions)
  speed: number; // typical speed [km/h]
  truckPct: number; // share of trucks [%]
  motoPct: number; // share of motorcycles [%]
  peds: number; // pedestrians per minute
  bikes: number; // bicycles per minute
  gap: number; // gap the ego driver accepts before turning [s]
  reaction: number; // driver reaction time [s]
  yellowRun: number; // share of drivers who push through a yellow [%]
  keepLeft: number; // 1: the ego pulls over to the kerb before left turns, 0: it turns from the middle of the lane
}

export const DEFAULT_SETTINGS: Settings = {
  oncoming: 18, cross: 10, speed: 50, truckPct: 20, motoPct: 25, peds: 16, bikes: 6,
  gap: 4.5, reaction: 0.8, yellowRun: 30, keepLeft: 1,
};

export type EgoMode = 'auto' | 'pedal' | 'manual';
export type Pedal = 'accel' | 'hold' | 'brake' | 'none';
export type EgoPhase = 'approach' | 'lane' | 'red' | 'wait' | 'stop' | 'right' | 'left' | 'free' | 'crash';

export interface CrashEvent {
  t: number;
  a: number; // agent index
  b: number; // agent index, or -1 for a fixed object
  speed: number; // closing speed [m/s]
  ego: boolean;
  x: number;
  y: number;
  rollover?: boolean; // agent a tipped over (b is -1)
}

export interface Sight { x: number; y: number; visible: boolean }

const PLAN_EVERY = 5; // controllers re-plan at 50 Hz
const PLAN_DT = PLAN_EVERY * SIM_DT;
const EGO_START = 75; // ego starts this far south of the centre [m], in the inner lane
const EGO_SPEED = 40 / 3.6;
const EGO_MAX_SPEED = 100 / 3.6;
const RESPAWN_AFTER_CRASH = 8; // s
const TOW_AFTER = 7; // wrecks are cleared this long after they stop [s]
const ROLLED_UP = 0.35; // a vehicle whose up axis has tipped past ~70° has rolled over
const STUCK_LIMIT = 90; // an NPC that has not moved for this long is removed [s]
const WELD_ON = 0.01, WELD_OFF = 1e6; // weld solref time constant: attached / released
const MOTO_LANE = OUTER + 1.7; // motorcycles ride by the kerb and filter past cars on their left
const FILTER_DV = 25 / 3.6; // ...at most this much faster than the car they are passing
// Ego driver's view, as bearings from straight ahead (left > 0): through the windows, and in the mirrors.
// Between them, over each rear quarter, is the blind spot.
const VIEW_FRONT = 1.75; // ±100°
const VIEW_LEFT_MIRROR = 2.6; // 149°..180° behind on the left
const VIEW_RIGHT_MIRROR = -2.7;
const LIMP_KP = 12; // joint stiffness of an unconscious body [N·m/rad]

interface Agent {
  idx: number;
  def: AgentDef;
  kind: Kind;
  spec: KindSpec;
  ego: boolean;
  root: number;
  rider: number; // rider root body (two-wheelers) or -1
  fq: number; fv: number; // free joint qpos / qvel address of the root
  rq: number; rv: number; // same for the rider
  mass: number;
  geoms: number[];
  bodies: number[];
  joints: { qadr: number; vadr: number; body: number; name: string }[];
  dofs: [number, number][]; // [adr, count] of every tree
  steerAct: number[];
  driveAct: number[];
  humanAct: { hip: number[]; knee: number[]; sh: number[] };
  weld: number;
  // state
  active: boolean;
  crashed: boolean;
  crashT: number;
  rolled: boolean; // tipped over (reported once)
  path: Path | null;
  s: number; // arc length of the agent centre along its path
  v: number; // commanded speed (force-driven agents)
  targetV: number;
  desired: number;
  wheelVel: number;
  steerCmd: number;
  committed: boolean; // right turn: decided to go
  fullStop: boolean; // stop sign: has come to a complete stop at the line
  yellowGo: boolean | null;
  runner: boolean; // pushes through yellows
  reactId: number;
  reactSince: number;
  stillSince: number;
  gait: number;
  label: string;
}

function quatMul(a: number[], b: number[]) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

export class IntersectionSim {
  readonly model: MjModel;
  readonly data: MjData;
  readonly agents: Agent[] = [];
  readonly ego: Agent;
  readonly settings: Settings = { ...DEFAULT_SETTINGS };
  readonly events: CrashEvent[] = []; // all crashes since reset
  /** Crashes since the caller last drained them (for alarms). */
  readonly fresh: CrashEvent[] = [];
  readonly stats = { turns: 0, rights: 0, lefts: 0, laps: 0, laneChanges: 0, egoCrashes: 0, crashes: 0, rollovers: 0, spawned: 0 };
  egoMode: EgoMode = 'auto';
  pedal: Pedal = 'none';
  manualSteer = 0; // rad, left > 0
  egoPhase: EgoPhase = 'approach';
  egoSight: Sight[] = [];
  signal: SignalState;
  /** Index into LAP of the turn the ego is heading for. */
  egoLeg = 0;
  private egoLcT0: number | null = null; // where this leg's lane change starts (null: none)
  private egoLcLocked = true; // lane change planned (or none needed) on this leg
  private egoHug: boolean | null = null; // pulling over before this left turn (null: not decided yet)
  private egoOff = false; // manual driving: off the lap route
  /** Geom → agent index (-1: scenery). */
  readonly geomAgent: Int32Array;
  private mj: MainModule;
  private stepCount = 0;
  private rand: () => number;
  private routes = new Map<string, Path>();
  private conflictCache = new Map<string, { sa: number; sb: number }[]>();
  private objectGeom: Uint8Array; // 1: fixed obstacle that counts as a crash (building, pole, tree)
  private seenPairs = new Set<string>();
  private egoSeen = new Map<number, number>(); // agent → sim time last seen by the ego driver
  private rayGeom: IntBuffer;
  private rayNormal: DoubleBuffer;
  private gain0: Float64Array;
  private ct0!: Int32Array;
  private ca0!: Int32Array;
  private bias0: Float64Array;

  constructor(mj: MainModule, pool: PoolSizes = DEFAULT_POOL, seed = 1) {
    this.mj = mj;
    this.rand = rng(seed);
    const defs = agentDefs(pool);
    this.model = mj.MjModel.from_xml_string(buildWorldXml(defs));
    this.data = new mj.MjData(this.model);
    this.rayGeom = new mj.IntBuffer(1);
    this.rayNormal = new mj.DoubleBuffer(3);
    const m = this.model;
    this.gain0 = Float64Array.from(m.actuator_gainprm as Float64Array);
    this.bias0 = Float64Array.from(m.actuator_biasprm as Float64Array);
    this.geomAgent = new Int32Array(m.ngeom).fill(-1);
    this.objectGeom = new Uint8Array(m.ngeom);
    const GEOM = mj.mjtObj.mjOBJ_GEOM.value;
    for (let g = 0; g < m.ngeom; g++) {
      const nm = mj.mj_id2name(m, GEOM, g);
      if (/^(bld_|tree_|upole_|sig_)/.test(nm)) this.objectGeom[g] = 1;
    }
    defs.forEach((d, i) => this.agents.push(this.makeAgent(d, i)));
    // Collision classes: vehicles (type 1, affinity 3) hit everything; walkers (type 2, affinity 1)
    // hit vehicles and the world but pass through each other. Scenery is type 1 / affinity 1.
    for (const a of this.agents) {
      const walker = a.kind === 'ped' || a.kind === 'bicycle';
      for (const g of a.geoms) {
        if (!m.geom_contype[g] && !m.geom_conaffinity[g]) continue;
        m.geom_contype[g] = walker ? 2 : 1;
        m.geom_conaffinity[g] = walker ? 1 : 3;
      }
    }
    this.ct0 = Int32Array.from(m.geom_contype as Int32Array);
    this.ca0 = Int32Array.from(m.geom_conaffinity as Int32Array);
    this.ego = this.agents[0];
    this.signal = signalAt(0);
    this.reset();
  }

  private makeAgent(def: AgentDef, idx: number): Agent {
    const mj = this.mj, m = this.model;
    const BODY = mj.mjtObj.mjOBJ_BODY.value, JOINT = mj.mjtObj.mjOBJ_JOINT.value;
    const ACT = mj.mjtObj.mjOBJ_ACTUATOR.value, EQ = mj.mjtObj.mjOBJ_EQUALITY.value;
    const root = mj.mj_name2id(m, BODY, def.name);
    const rider = mj.mj_name2id(m, BODY, `${def.name}_rider`);
    const roots = rider >= 0 ? [root, rider] : [root];
    const bodies: number[] = [];
    for (let b = 1; b < m.nbody; b++) if (roots.includes(m.body_rootid[b])) bodies.push(b);
    const geoms: number[] = [];
    for (let g = 0; g < m.ngeom; g++) {
      if (bodies.includes(m.geom_bodyid[g])) {
        geoms.push(g);
        this.geomAgent[g] = idx;
      }
    }
    const joints: Agent['joints'] = [];
    const dofs: [number, number][] = [];
    for (let j = 0; j < m.njnt; j++) {
      if (!bodies.includes(m.jnt_bodyid[j])) continue;
      if (m.jnt_type[j] !== 0) {
        joints.push({ qadr: m.jnt_qposadr[j], vadr: m.jnt_dofadr[j], body: m.jnt_bodyid[j], name: mj.mj_id2name(m, JOINT, j) });
      }
    }
    for (const r of roots) {
      let a = Infinity, b = -Infinity;
      for (let d = 0; d < m.nv; d++) {
        if (m.body_rootid[m.dof_bodyid[d]] === r) { a = Math.min(a, d); b = Math.max(b, d); }
      }
      dofs.push([a, b - a + 1]);
    }
    const free = (b: number) => {
      const j = m.body_jntadr[b];
      return [m.jnt_qposadr[j], m.jnt_dofadr[j]];
    };
    const [fq, fv] = free(root);
    const [rq, rv] = rider >= 0 ? free(rider) : [-1, -1];
    const a = (n: string) => mj.mj_name2id(m, ACT, n);
    const hn = rider >= 0 ? `${def.name}_rider` : def.name;
    const isHuman = def.kind === 'ped' || rider >= 0;
    const spec = KIND[def.kind];
    return {
      idx, def, kind: def.kind, spec, ego: def.ego, root, rider, fq, fv, rq, rv,
      mass: m.body_subtreemass[root] + (rider >= 0 ? m.body_subtreemass[rider] : 0),
      geoms, bodies, joints, dofs,
      steerAct: def.kind === 'car' || def.kind === 'truck'
        ? ['fl', 'fr'].map((w) => a(`${def.name}_steer_${w}`))
        : def.kind === 'ped' ? [] : [a(`${def.name}_steer`)],
      driveAct: def.kind === 'car' || def.kind === 'truck'
        ? ['fl', 'fr', 'rl', 'rr'].map((w) => a(`${def.name}_drive_${w}`))
        : def.kind === 'ped' ? [] : [a(`${def.name}_drive`)],
      humanAct: isHuman
        ? { hip: ['l', 'r'].map((s) => a(`${hn}_hip_${s}`)), knee: ['l', 'r'].map((s) => a(`${hn}_knee_${s}`)), sh: ['l', 'r'].map((s) => a(`${hn}_sh_${s}`)) }
        : { hip: [], knee: [], sh: [] },
      weld: rider >= 0 ? mj.mj_name2id(m, EQ, `${def.name}_seat`) : -1,
      active: false, crashed: false, crashT: 0, rolled: false, path: null, s: 0, v: 0, targetV: 0, desired: 0,
      wheelVel: 0, steerCmd: 0, committed: false, fullStop: false, yellowGo: null, runner: false,
      reactId: -1, reactSince: 0, stillSince: 0, gait: 0, label: def.name,
    };
  }

  // ── Reset / spawn / park ────────────────────────────────

  reset() {
    this.mj.mj_resetData(this.model, this.data);
    for (const a of this.agents) this.park(a);
    this.events.length = 0;
    this.fresh.length = 0;
    this.seenPairs.clear();
    for (const k in this.stats) (this.stats as Record<string, number>)[k] = 0;
    this.stepCount = 0;
    this.signal = signalAt(0);
    this.spawnEgo();
    this.mj.mj_forward(this.model, this.data);
  }

  get time() {
    return this.data.time;
  }

  private setBodiesFloating(a: Agent, floating: boolean) {
    const m = this.model;
    for (const b of a.bodies) m.body_gravcomp[b] = floating ? 1 : 0;
    for (const g of a.geoms) {
      m.geom_contype[g] = floating ? 0 : this.ct0[g];
      m.geom_conaffinity[g] = floating ? 0 : this.ca0[g];
    }
  }

  /** After an impact a walker collides with everything, so a thrown body can hit other people. */
  private setFullyCollidable(a: Agent) {
    const m = this.model;
    for (const g of a.geoms) {
      if (!this.ct0[g]) continue;
      m.geom_contype[g] = 3;
      m.geom_conaffinity[g] = 3;
    }
  }

  private setWeld(a: Agent, on: boolean) {
    if (a.weld >= 0) this.model.eq_solref[2 * a.weld] = on ? WELD_ON : WELD_OFF;
  }

  private setLimp(a: Agent, limp: boolean) {
    const m = this.model;
    const all = [...a.humanAct.hip, ...a.humanAct.knee, ...a.humanAct.sh];
    for (const id of all) {
      const kp = limp ? LIMP_KP : this.gain0[10 * id];
      m.actuator_gainprm[10 * id] = kp;
      m.actuator_biasprm[10 * id + 1] = -kp;
    }
  }

  /** Move an agent off stage: floating, no collisions, no control. */
  private park(a: Agent) {
    const d = this.data, m = this.model;
    a.active = false;
    a.crashed = false;
    a.path = null;
    this.setBodiesFloating(a, true);
    this.setWeld(a, true);
    this.setLimp(a, false);
    for (const r of [a.root, a.rider]) {
      if (r < 0) continue;
      const j = m.body_jntadr[r], qa = m.jnt_qposadr[j];
      for (let i = 0; i < 7; i++) d.qpos[qa + i] = m.qpos0[qa + i];
    }
    for (const jt of a.joints) d.qpos[jt.qadr] = m.qpos0[jt.qadr];
    for (const [adr, n] of a.dofs) for (let i = 0; i < n; i++) d.qvel[adr + i] = 0;
    for (const b of a.bodies) for (let i = 0; i < 6; i++) d.xfrc_applied[6 * b + i] = 0;
    for (const id of [...a.steerAct, ...a.driveAct]) d.ctrl[id] = 0;
  }

  /** Put an agent on its path at arc length s0, moving at v0. */
  private place(a: Agent, path: Path, s0: number, v0: number) {
    const d = this.data, m = this.model;
    const P = path.at(s0);
    const c = Math.cos(P.h), sn = Math.sin(P.h);
    const qz = [Math.cos(P.h / 2), 0, 0, Math.sin(P.h / 2)];
    const [px, py, pz] = a.def.park;
    const zBase = a.kind === 'ped' ? 0.03 : a.spec.r + 0.03;
    this.setBodiesFloating(a, false);
    this.setWeld(a, true);
    this.setLimp(a, false);
    for (const r of [a.root, a.rider]) {
      if (r < 0) continue;
      const qa = m.jnt_qposadr[m.body_jntadr[r]];
      const dx = m.qpos0[qa] - px, dy = m.qpos0[qa + 1] - py, dz = m.qpos0[qa + 2] - pz;
      d.qpos[qa] = P.x + c * dx - sn * dy;
      d.qpos[qa + 1] = P.y + sn * dx + c * dy;
      d.qpos[qa + 2] = zBase + dz;
      const q = quatMul(qz, [m.qpos0[qa + 3], m.qpos0[qa + 4], m.qpos0[qa + 5], m.qpos0[qa + 6]]);
      for (let i = 0; i < 4; i++) d.qpos[qa + 3 + i] = q[i];
    }
    const pose: Pose = a.kind === 'moto' ? SEATED_MOTO : a.kind === 'bicycle' ? SEATED_BIKE : STAND;
    for (const jt of a.joints) {
      let v = 0;
      if (/_hip_/.test(jt.name)) v = pose.hip;
      else if (/_knee_/.test(jt.name)) v = pose.knee;
      else if (/_sh_/.test(jt.name)) v = pose.shoulder;
      d.qpos[jt.qadr] = v;
    }
    for (const [adr, n] of a.dofs) for (let i = 0; i < n; i++) d.qvel[adr + i] = 0;
    for (const r of [a.fv, a.rv]) {
      if (r < 0) continue;
      d.qvel[r] = v0 * c;
      d.qvel[r + 1] = v0 * sn;
    }
    if (a.spec.r > 0) {
      for (const jt of a.joints) if (jt.name.endsWith('_roll')) d.qvel[jt.vadr] = v0 / a.spec.r;
    }
    this.setPose(a, pose);
    a.active = true;
    a.crashed = false;
    a.rolled = false;
    a.path = path;
    a.s = s0;
    a.v = v0;
    a.targetV = v0;
    a.wheelVel = a.spec.r > 0 ? v0 / a.spec.r : 0;
    a.steerCmd = 0;
    a.committed = false;
    a.fullStop = false;
    a.yellowGo = null;
    a.reactId = -1;
    a.stillSince = this.time;
    a.gait = this.rand() * 6;
  }

  private setPose(a: Agent, p: Pose) {
    const ctrl = this.data.ctrl;
    for (const id of a.humanAct.hip) ctrl[id] = p.hip;
    for (const id of a.humanAct.knee) ctrl[id] = p.knee;
    for (const id of a.humanAct.sh) ctrl[id] = p.shoulder;
  }

  spawnEgo() {
    const e = this.ego;
    this.park(e);
    this.egoLeg = 0;
    this.egoLcT0 = null;
    this.egoLcLocked = true;
    this.egoHug = null;
    this.egoOff = false;
    const path = this.legPath(0, null, EGO_START);
    // Make room at the start line (traffic now shares the south arm).
    const P = path.at(0);
    for (const b of this.agents) {
      if (!b.active || b.ego) continue;
      const [x, y] = this.pos(b);
      if ((x - P.x) ** 2 + (y - P.y) ** 2 < 15 * 15) this.park(b);
    }
    // Start standing at the start line; the driver (or the auto pilot) pulls away from rest.
    this.place(e, path, 0, 0);
    e.desired = EGO_SPEED;
    this.egoPhase = 'approach';
    this.egoSeen.clear();
    this.pedal = 'none';
    this.manualSteer = 0;
  }

  /** Route of lap leg i (cached). lcT0: start of the lane change, null when none is needed. */
  private legPath(i: number, lcT0: number | null, start = LEG_START, hugT0: number | null = null) {
    const key = `leg|${i}|${lcT0}|${start}|${hugT0}`;
    let p = this.routes.get(key);
    if (!p) this.routes.set(key, (p = buildLeg(i, lcT0, start, hugT0)));
    return p;
  }

  /** Put the ego on another route, keeping it where it is. */
  private setEgoPath(p: Path, around?: number) {
    const a = this.ego;
    const [x, y] = this.pos(a);
    a.path = p;
    const k = around === undefined ? p.project(x, y, p.n >> 1, p.n, p.n).k : p.project(x, y, p.kOf(around), 20, 20).k;
    a.s = k * PATH_STEP;
  }

  private setLeg(i: number, lcT0: number | null) {
    const a = this.ego;
    this.egoLeg = i;
    this.egoLcT0 = lcT0;
    this.egoLcLocked = lcT0 === null;
    this.egoHug = null;
    a.committed = false;
    a.fullStop = false;
    a.yellowGo = null;
    this.setEgoPath(this.legPath(i, lcT0));
  }

  /** The next turn of the lap: junction and direction. */
  egoNext(): { j: JunctionId; turn: 'left' | 'right' } {
    const L = LAP[this.egoLeg];
    return { j: L.j, turn: L.turn };
  }

  private route(approach: Arm, turn: Turn, o1: number, o2: number, keepLeft = false) {
    const key = `${approach}|${turn}|${o1}|${o2}|${keepLeft && turn === 'left'}`;
    let p = this.routes.get(key);
    if (!p) this.routes.set(key, (p = buildRoute(approach, turn, o1, o2, ARM, ARM, keepLeft)));
    return p;
  }

  private walk(along: 'x' | 'y', side: 1 | -1, dir: 1 | -1, lat: number) {
    const key = `w|${along}|${side}|${dir}|${lat}`;
    let p = this.routes.get(key);
    if (!p) this.routes.set(key, (p = buildWalk(along, side, dir, lat)));
    return p;
  }

  private free(kind: Kind) {
    return this.agents.find((a) => a.kind === kind && !a.active && !a.ego);
  }

  /** Nothing within `clear` metres of the start of `path`. */
  private entryClear(path: Path, clear: number) {
    const x0 = path.x[0], y0 = path.y[0];
    for (const b of this.agents) {
      if (!b.active) continue;
      const [x, y] = this.pos(b);
      if ((x - x0) ** 2 + (y - y0) ** 2 < clear * clear) return false;
    }
    return true;
  }

  /**
   * The ego is pulling out across a main road close to where traffic enters the stage (the south and
   * east junctions are 60 m from the ends of the arms). A car appearing now would have been in plain
   * view further back, so the ego would not have gone: don't spawn one.
   */
  private egoCrossingNear(path: Path) {
    const e = this.ego, p = e.path;
    if (!e.active || e.crashed || !p || (p.control !== 'stop' && p.control !== 'yield') || e.s < p.waitS - 25 || e.s > p.exitS) return false;
    const [x, y] = this.pos(e);
    return (path.x[0] - x) ** 2 + (path.y[0] - y) ** 2 < 110 * 110;
  }

  private spawnTraffic() {
    const st = this.settings;
    const r = this.rand;
    const streams: [Arm, 'inner' | 'outer', number, [Turn, number][]][] = [
      ['N', 'inner', st.oncoming * 0.5, [['straight', 0.7], ['right', 0.3]]],
      ['N', 'outer', st.oncoming * 0.5, [['straight', 0.85], ['left', 0.15]]],
      ['S', 'inner', st.oncoming * 0.35, [['straight', 0.75], ['right', 0.25]]],
      ['S', 'outer', st.oncoming * 0.35, [['straight', 0.8], ['left', 0.2]]],
      ['E', 'inner', st.cross * 0.25, [['straight', 0.75], ['right', 0.25]]],
      ['E', 'outer', st.cross * 0.25, [['straight', 0.8], ['left', 0.2]]],
      ['W', 'inner', st.cross * 0.25, [['straight', 0.75], ['right', 0.25]]],
      ['W', 'outer', st.cross * 0.25, [['straight', 0.8], ['left', 0.2]]],
    ];
    for (const [arm, lane0, perMin, turns] of streams) {
      if (r() >= (perMin / 60) * PLAN_DT) continue;
      const u = r() * 100;
      const kind: Kind = u < st.motoPct ? 'moto' : u < st.motoPct + st.truckPct ? 'truck' : 'car';
      const a = this.free(kind);
      if (!a) continue;
      const lane = kind === 'moto' ? 'outer' : lane0;
      let turn: Turn = 'straight';
      let acc = 0;
      const t = r();
      for (const [tn, p] of turns) if (t < (acc += p)) { turn = tn; break; }
      if (lane !== lane0 && turn === 'right') turn = 'straight';
      // Motorcycles keep to the kerb side of the outer lane, where they can filter past cars.
      const o1 = lane === 'inner' ? INNER : kind === 'moto' ? MOTO_LANE : OUTER;
      const o2 = turn === 'right' ? INNER : turn === 'left' ? (kind === 'moto' ? OUTER + 0.6 : OUTER) : o1;
      // Other drivers pull over before turning left.
      const path = this.route(arm, turn, o1, o2, kind !== 'moto');
      const v = (st.speed / 3.6) * (0.88 + 0.24 * r()) * (kind === 'moto' ? 1.1 : kind === 'truck' ? 0.9 : 1);
      // Room to stop behind a queue that has backed up to the end of the road
      if (!this.entryClear(path, Math.max(kind === 'truck' ? 26 : 20, 8 + v * 2.2)) || this.egoCrossingNear(path)) continue;
      this.place(a, path, 0, v);
      a.desired = v;
      a.runner = r() * 100 < st.yellowRun;
      a.label = `${a.def.name}:${path.label}`;
      this.stats.spawned++;
    }
    // Pedestrians and bicycles on the sidewalks, crossing on the crosswalks
    for (const [kind, perMin] of [['ped', st.peds], ['bicycle', st.bikes]] as const) {
      if (r() >= (perMin / 60) * PLAN_DT) continue;
      const a = this.free(kind);
      if (!a) continue;
      const along = r() < 0.5 ? 'x' : 'y';
      const side = r() < 0.5 ? 1 : -1;
      const dir = r() < 0.5 ? 1 : -1;
      // Keep left on the sidewalk: pedestrians inside, bicycles outside, one line per direction.
      const lat = (kind === 'ped' ? 0.5 : 1.5) * dir;
      const path = this.walk(along, side, dir, lat);
      const s0 = kind === 'ped' ? 8 + r() * 20 : r() * 10;
      let clear = true;
      const P = path.at(s0);
      for (const b of this.agents) {
        if (!b.active || (b.kind !== 'ped' && b.kind !== 'bicycle')) continue;
        const [x, y] = this.pos(b);
        if ((x - P.x) ** 2 + (y - P.y) ** 2 < 4) clear = false;
      }
      if (!clear) continue;
      const v = kind === 'ped' ? 1.1 + r() * 0.5 : 3.5 + r() * 2;
      this.place(a, path, s0, v);
      a.desired = v;
      a.label = `${a.def.name}:${path.label}`;
      this.stats.spawned++;
    }
  }

  // ── State helpers ───────────────────────────────────────

  pos(a: Agent): [number, number, number] {
    const q = this.data.qpos;
    return [q[a.fq], q[a.fq + 1], q[a.fq + 2]];
  }
  vel(a: Agent): [number, number, number] {
    const v = this.data.qvel;
    return [v[a.fv], v[a.fv + 1], v[a.fv + 2]];
  }
  yaw(a: Agent) {
    const q = this.data.qpos, i = a.fq + 3;
    return Math.atan2(2 * (q[i] * q[i + 3] + q[i + 1] * q[i + 2]), 1 - 2 * (q[i + 2] ** 2 + q[i + 3] ** 2));
  }
  speed(a: Agent) {
    const [vx, vy] = this.vel(a);
    const h = this.yaw(a);
    return vx * Math.cos(h) + vy * Math.sin(h);
  }
  /** z of the root's up axis (1 upright). */
  upZ(a: Agent) {
    const q = this.data.qpos, i = a.fq + 3;
    return 1 - 2 * (q[i + 1] ** 2 + q[i + 2] ** 2);
  }

  isActive(i: number) {
    return this.agents[i].active;
  }

  lightFor(axis: Axis) {
    return axis === 'NS' ? this.signal.ns : this.signal.ew;
  }
  arrowFor(axis: Axis) {
    return axis === 'NS' ? this.signal.nsArrow : this.signal.ewArrow;
  }

  // ── Perception ──────────────────────────────────────────

  private eyeOf(a: Agent): [number, number, number] {
    const [x, y] = this.pos(a);
    const h = this.yaw(a);
    if (a.ego) {
      const d = this.data, cam = this.mj.mj_name2id(this.model, this.mj.mjtObj.mjOBJ_CAMERA.value, 'driver_cam');
      return [d.cam_xpos[3 * cam], d.cam_xpos[3 * cam + 1], d.cam_xpos[3 * cam + 2]];
    }
    const fwd = a.kind === 'truck' ? 2.6 : a.kind === 'car' ? 0 : -0.2;
    return [x + Math.cos(h) * fwd, y + Math.sin(h) * fwd, a.spec.eye];
  }

  /** Probe points on a target (centre, and the ends of long vehicles). */
  private targetPoints(b: Agent): [number, number, number][] {
    const [x, y, z] = this.pos(b);
    const h = this.yaw(b);
    const up = b.kind === 'ped' ? 0.35 : b.kind === 'car' || b.kind === 'truck' ? 0.45 : 0.75;
    const pts: [number, number, number][] = [[x, y, z + up]];
    if (b.kind === 'car' || b.kind === 'truck') {
      const e = b.spec.len * 0.35;
      pts.push([x + Math.cos(h) * e, y + Math.sin(h) * e, z + up], [x - Math.cos(h) * e, y - Math.sin(h) * e, z + up]);
    }
    return pts;
  }

  /**
   * Line of sight from a's eyes to any probe point of b, using MuJoCo ray casting.
   * mirrors = false: the ego driver is looking out of the windows only (not checking the mirrors).
   */
  canSee(a: Agent, b: Agent, mirrors = true): boolean {
    const eye = this.eyeOf(a);
    const exclude = a.rider >= 0 ? a.rider : a.root;
    const yaw = a.ego ? this.yaw(a) : 0;
    for (const p of this.targetPoints(b)) {
      if (a.ego) {
        // The ego driver sees through the windows and in the mirrors, not over the rear quarters.
        const bearing = angleDiff(Math.atan2(p[1] - eye[1], p[0] - eye[0]), yaw);
        const inView = Math.abs(bearing) <= VIEW_FRONT || (mirrors && (bearing >= VIEW_LEFT_MIRROR || bearing <= VIEW_RIGHT_MIRROR));
        if (!inView) continue;
      }
      const vec = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
      const dist = this.mj.mj_ray(this.model, this.data, eye, vec, null as unknown as number[], true, exclude, this.rayGeom, this.rayNormal);
      const g = this.rayGeom.GetView()[0];
      if (dist < 0 || dist >= 1 || g < 0) return true;
      const hit = this.geomAgent[g];
      if (hit === b.idx) return true;
      if (hit === a.idx) {
        // Own body part (mirror, pillar) — ignore it and treat as visible.
        return true;
      }
    }
    return false;
  }

  /**
   * Crossing conflicts: the paths meet at an angle (not followers in the same lane). With `merge`,
   * also the point where pa turns into pb's lane (a side street joining a main road).
   */
  private conflictsOf(pa: Path, pb: Path, dist: number, merge = false) {
    const key = `${pa.id}|${pb.id}|${dist}|${merge ? 'm' : ''}`;
    let c = this.conflictCache.get(key);
    if (!c) {
      const ang = (sa: number, sb: number) => Math.abs(angleDiff(pa.heading[pa.kOf(sa)], pb.heading[pb.kOf(sb)]));
      c = conflicts(pa, pb, dist).flatMap(({ sa, sb, sa0, sb0 }) => {
        if (ang(sa, sb) > 0.5) return [{ sa, sb }];
        if (merge && ang(sa0, sb0) > 0.35) return [{ sa: sa0, sb: sb0 }];
        return [];
      });
      this.conflictCache.set(key, c);
    }
    return c;
  }

  /**
   * Nearest agent inside the corridor ahead of `a` along its path.
   * Pedestrians and bicycles are also checked where they will be in one second.
   */
  private obstacleAhead(a: Agent, look: number, visibleOnly: boolean) {
    const p = a.path!;
    const k0 = p.kOf(a.s);
    const kEnd = p.kOf(a.s + look);
    const [ax, ay] = this.pos(a);
    const found: { gap: number; vLead: number; b: Agent }[] = [];
    const walker = (x: Agent) => x.kind === 'ped' || x.kind === 'bicycle';
    for (const b of this.agents) {
      if (!b.active || b === a) continue;
      const [bx, by] = this.pos(b);
      if ((bx - ax) ** 2 + (by - ay) ** 2 > (look + 12) ** 2) continue;
      // Someone standing on the sidewalk waiting to cross is not in a vehicle's way.
      if (!walker(a) && walker(b) && !b.crashed && b.v < 0.2 && b.path && b.s + b.spec.len / 2 < b.path.crossS) continue;
      // Walkers step around each other; they only queue behind someone going their way.
      if (walker(a) && walker(b) && !b.crashed && (!b.path || Math.cos(angleDiff(b.path.heading[0], p.heading[0])) < 0.7)) continue;
      const [bvx, bvy] = this.vel(b);
      const hb = this.yaw(b);
      // Probe points: the body outline (corners and edge midpoints), plus where walkers will be.
      const human = b.kind === 'ped';
      const rad = human ? 0.3 : 0.25;
      // Cheap pre-filter: is b's centre anywhere near the corridor?
      const reach = a.spec.wid / 2 + (human ? 0.3 : b.spec.len / 2) + 2.5;
      let near = false;
      for (let k = k0; k <= kEnd && !near; k += 4) near = (p.x[k] - bx) ** 2 + (p.y[k] - by) ** 2 < reach * reach;
      if (!near && !(walker(b) && !b.crashed)) continue;
      const probes: [number, number][] = [[bx, by]];
      if (!human) {
        const L = b.spec.len / 2, W = b.spec.wid / 2 - 0.1, c = Math.cos(hb), sn = Math.sin(hb);
        const n = Math.max(2, Math.ceil((2 * L) / 1.0));
        for (let i = 0; i <= n; i++) {
          const u = -L + (2 * L * i) / n;
          probes.push([bx + c * u - sn * W, by + sn * u + c * W], [bx + c * u + sn * W, by + sn * u - c * W]);
        }
        probes.push([bx + c * L, by + sn * L], [bx - c * L, by - sn * L]);
      }
      if ((b.kind === 'ped' || b.kind === 'bicycle') && !b.crashed) {
        probes.push([bx + bvx, by + bvy], [bx + 2 * bvx, by + 2 * bvy]);
      }
      let best: { gap: number; vLead: number; b: Agent } | null = null;
      for (const [px, py] of probes) {
        let bk = -1, bd = Infinity;
        for (let k = k0; k <= kEnd; k += 2) {
          const d = (p.x[k] - px) ** 2 + (p.y[k] - py) ** 2;
          if (d < bd) { bd = d; bk = k; }
        }
        if (bk < 0) continue;
        for (let k = Math.max(k0, bk - 1); k <= Math.min(kEnd, bk + 1); k++) {
          const d = (p.x[k] - px) ** 2 + (p.y[k] - py) ** 2;
          if (d < bd) { bd = d; bk = k; }
        }
        // Motorcycles squeeze through narrow gaps (filtering past cars on their left) — but not past
        // a car signalling left and pulling over to the kerb.
        const squeeze = a.kind === 'moto' && !(b.path?.turn === 'left' && b.path.keepsLeft);
        if (Math.sqrt(bd) > a.spec.wid / 2 + rad + (squeeze ? 0.05 : 0.35)) continue;
        const gap = (bk - k0) * PATH_STEP - a.spec.len / 2 - rad;
        if (gap < -a.spec.len / 2) continue; // beside or behind
        if (!best || gap < best.gap) {
          const hk = p.heading[bk];
          best = { gap, vLead: bvx * Math.cos(hk) + bvy * Math.sin(hk), b };
        }
      }
      if (best) found.push(best);
    }
    found.sort((u, v) => u.gap - v.gap);
    for (const o of found.slice(0, 3)) {
      if (!visibleOnly || o.gap < 2 || this.canSee(a, o.b)) return o;
    }
    return null;
  }

  /**
   * b is stopped and will stay stopped until a moves: it is waiting for a, held by a red, waiting for its
   * own turn, or queued behind such a vehicle.
   */
  private notComing(b: Agent, a: Agent, depth = 0): boolean {
    if (depth > 4 || !b.path || b.crashed || Math.abs(this.speed(b)) > 0.5) return false;
    if (b.reactId === a.idx || this.heldByRed(b) || (b.path.waitS > 0 && !b.committed)) return true;
    return b.reactId >= 0 && this.notComing(this.agents[b.reactId], a, depth + 1);
  }

  /** b is behind a, in a track that runs into a — no room to come past it. */
  private stuckBehind(b: Agent, a: Agent) {
    if (!b.path) return false;
    const [ax, ay] = this.pos(a), [bx, by] = this.pos(b);
    const h = this.yaw(a);
    if ((bx - ax) * Math.cos(h) + (by - ay) * Math.sin(h) > 0) return false;
    const pr = b.path.project(ax, ay, b.path.kOf(b.s), 0, 120);
    // Same clearance as obstacleAhead uses to decide whether b has to stop for a.
    const room = a.spec.wid / 2 - 0.1 + b.spec.wid / 2 + 0.25 + (b.kind === 'moto' ? 0.05 : 0.35);
    return pr.d2 < room * room;
  }

  /** b faces a red (or is stopping for a yellow) before its stop line and can stop in time. */
  private heldByRed(b: Agent) {
    const p = b.path!;
    if (p.stopS < 0 || p.control !== 'signal') return false;
    const d = p.stopS - (b.s + b.spec.len / 2);
    if (d < -0.5) return false;
    const light = this.lightFor(p.axis);
    if (light === 'G' || (p.turn === 'right' && this.arrowFor(p.axis))) return false;
    if (light === 'Y' && b.yellowGo) return false;
    const v = Math.max(0, this.speed(b));
    return v * v / (2 * 4) < d + 0.5;
  }

  /** Is it safe for a right-turning agent to go now? */
  private gapOk(a: Agent, gapT: number, visibleOnly: boolean, sight?: Sight[]): boolean {
    const p = a.path!;
    let ok = true;
    for (const b of this.agents) {
      if (!b.active || b === a || !b.path || b.crashed) continue;
      const human = b.kind === 'ped' || b.kind === 'bicycle';
      const cs = this.conflictsOf(p, b.path, human ? 2.5 : 3.2, p.control === 'stop');
      const c = cs.find((c) => c.sa > a.s - 1 && c.sa < a.s + 40 && c.sb > b.s - b.spec.len / 2 - 1.5);
      if (!c) continue;
      const dB = c.sb - b.s - (human ? 0 : b.spec.len / 2);
      const vB = Math.max(0, human ? b.v : this.speed(b));
      if (!human && dB > 90) continue;
      // Turning left: whatever is queued behind us in our own track has to wait for us. A motorcycle
      // with room to come up on our left does not.
      if (!human && p.turn === 'left' && this.stuckBehind(b, a)) continue;
      // Facing a red before its stop line and able to stop for it: not coming (if we meet beyond that line).
      if (!human && c.sb > b.path.stopS && this.heldByRed(b)) continue;
      // An opposing turner waiting for its own gap is not coming either — unless it has decided to go.
      if (!human && vB < 0.8 && dB > 3 && b.path.waitS > 0 && !b.committed) continue;
      if (!human && this.notComing(b, a)) continue;
      if (human && (dB > 12 || (vB < 0.2 && Math.abs(dB) > 2.5))) continue;
      // Standing at the kerb waiting for their own green: not stepping out.
      if (human && vB < 0.2 && b.s < b.path.crossS && (b.path.axis === 'NS' ? this.signal.walkNS : this.signal.walkEW) !== 'G') continue;
      let seen = true;
      if (visibleOnly) {
        // At a left turn the driver's eyes are on the corner and the crosswalk, not the mirrors: the
        // mirror check was made earlier, when deciding whether to pull over.
        if (this.canSee(a, b, p.turn !== 'left')) this.egoSeen.set(b.idx, this.time);
        const last = this.egoSeen.get(b.idx);
        seen = last !== undefined && this.time - last < 0.6;
        if (sight) {
          const [x, y] = this.pos(b);
          sight.push({ x, y, visible: seen });
        }
      }
      if (!seen) continue;
      // Time until b reaches the conflict point, accelerating if it is below its cruising speed.
      const acc = vB < b.desired * 0.9 ? b.spec.accel : 0;
      const tB = acc > 0 ? (-vB + Math.sqrt(vB * vB + 2 * acc * Math.max(0, dB))) / acc : dB / Math.max(vB, 0.1);
      const threat = human ? Math.abs(dB) < 2.5 || dB / Math.max(vB, 0.3) < 3 : dB < 3 || tB < gapT;
      if (threat) ok = false;
    }
    return ok;
  }

  // ── Planning (50 Hz) ────────────────────────────────────

  private plan() {
    this.signal = signalAt(this.time);
    this.spawnTraffic();
    for (const a of this.agents) {
      if (!a.active) continue;
      this.housekeeping(a);
      if (!a.active || a.crashed) continue;
      if (a.ego) this.planEgo(a);
      else if (a.kind === 'ped' || a.kind === 'bicycle') this.planWalker(a);
      else this.planVehicle(a);
    }
  }

  /** Progress along the path, towing wrecks, removing agents that left the stage. */
  private housekeeping(a: Agent) {
    const [x, y, z] = this.pos(a);
    const t = this.time;
    if (!a.rolled && (a.kind === 'car' || a.kind === 'truck') && this.upZ(a) < ROLLED_UP) this.onRollover(a);
    const moving = Math.hypot(...this.vel(a)) > 0.3;
    if (moving) a.stillSince = t;
    if (a.path && (a.kind === 'car' || a.kind === 'truck') && !a.crashed) {
      a.s = a.path.project(x, y, a.path.kOf(a.s), 12, 30).k * PATH_STEP;
    }
    const out = Math.abs(x) > ARM + 20 || Math.abs(y) > ARM + 20 || z < -3;
    if (a.ego) {
      if (a.crashed && (t - a.crashT > RESPAWN_AFTER_CRASH)) this.spawnEgo();
      else if (out) this.spawnEgo();
      return;
    }
    if (out) return this.park(a);
    if (a.crashed) {
      // Once a thrown walker has landed, others step around it again.
      if ((a.kind === 'ped' || a.kind === 'bicycle') && t - a.crashT > 2.5) this.setBodiesFloating(a, false);
      if (t - a.stillSince > TOW_AFTER || t - a.crashT > 25) this.park(a);
      return;
    }
    if (a.path && a.s >= a.path.length - 2) return this.park(a);
    if (t - a.stillSince > STUCK_LIMIT) this.park(a);
  }

  private stopLimit(a: Agent, turn: boolean): number {
    const p = a.path!;
    if (p.stopS < 0 || p.control !== 'signal') return Infinity; // stop signs: see waitLimit
    const d = p.stopS - (a.s + a.spec.len / 2);
    const light = this.lightFor(p.axis);
    if (d < -0.5) {
      // Past the stop line: clear the intersection — unless stopped with the nose barely over it (short of the crosswalk),
      // in which case wait for the next green rather than go on a red.
      const stranded = d > -1.2 && light === 'R' && Math.max(0, this.speed(a)) < 1 && !(turn && p.turn === 'right' && this.arrowFor(p.axis));
      return stranded ? 0 : Infinity;
    }
    if (light === 'G') {
      a.yellowGo = null;
      return Infinity;
    }
    if (turn && p.waitS > 0 && p.turn === 'right' && this.arrowFor(p.axis)) return Infinity;
    if (light === 'Y') {
      if (a.yellowGo === null) {
        const v = Math.max(0, this.speed(a));
        a.yellowGo = v * v / (2 * 3.5) > d || (a.runner && d < v * 2.5);
      }
      if (a.yellowGo) return Infinity;
    }
    return Math.sqrt(2 * 3.0 * Math.max(0, d - 0.3));
  }

  private obstacleLimit(a: Agent, visibleOnly = true) {
    const v = Math.max(0, a.kind === 'moto' ? a.v : this.speed(a));
    const o = this.obstacleAhead(a, Math.max(14, v * 3.5), visibleOnly);
    if (!o) {
      a.reactId = -1;
      return Infinity;
    }
    if (o.b.idx !== a.reactId) {
      a.reactId = o.b.idx;
      a.reactSince = this.time;
    }
    // Reaction time before the driver brakes for something new.
    const reaction = a.ego && this.egoMode !== 'auto' ? 0 : this.settings.reaction;
    if (this.time - a.reactSince < reaction && o.gap > 1) return Infinity;
    const g0 = o.b.kind === 'ped' || o.b.kind === 'bicycle' ? 2.5 : 2;
    const b = a.spec.decel * 0.75;
    const vl = Math.max(0, o.vLead);
    // Queueing behind a car stopped in the intersection: wait at the stop line, not on the crosswalk.
    const p = a.path!;
    const front = a.s + a.spec.len / 2;
    if (p.control === 'signal' && p.stopS > 0 && front < p.stopS + 0.5 && vl < 0.5 && a.kind !== 'bicycle') {
      const stopAt = front + o.gap - g0;
      if (stopAt > p.stopS + 0.5 && stopAt < p.stopS + 1.5 + (WALK_OUT - ROAD_HALF) + 1) {
        return Math.sqrt(2 * 3.0 * Math.max(0, p.stopS - 0.3 - front));
      }
    }
    return o.gap <= g0 ? Math.min(vl, 0.0) : Math.sqrt(vl * vl + 2 * b * (o.gap - g0));
  }

  private waitLimit(a: Agent, gapT: number, visibleOnly: boolean, sight?: Sight[]) {
    const p = a.path!;
    if (p.waitS < 0 || a.committed) return Infinity;
    if (a.s > p.waitS + 2) {
      a.committed = true;
      return Infinity;
    }
    if (a.s > p.waitS - 20) {
      let mayEnter = true;
      if (p.control === 'signal') {
        const pastStop = a.s + a.spec.len / 2 > p.stopS;
        const light = this.lightFor(p.axis);
        mayEnter = pastStop || light === 'G' || (p.turn === 'right' && this.arrowFor(p.axis));
      } else if (p.control === 'stop') {
        // A stop sign means a complete stop at the line, whatever the traffic.
        if (this.speed(a) < 0.3 && a.s > p.waitS - 1.5) a.fullStop = true;
        mayEnter = a.fullStop;
      }
      // Left turns: the last look along the left side is just before turning in.
      const near = p.turn === 'left' ? a.s > p.waitS - 3 : a.s > p.waitS - 10 || this.speed(a) < 3;
      if (mayEnter && near && this.gapOk(a, gapT, visibleOnly, sight)) {
        a.committed = true;
        return Infinity;
      }
    }
    return Math.sqrt(2 * 2.5 * Math.max(0, p.waitS - a.s));
  }

  /**
   * Turning vehicles give way to pedestrians and bicycles on the crosswalk they are about to cross:
   * stop before it if a walker will be there around the time the vehicle gets there.
   */
  private walkerYieldLimit(a: Agent, visibleOnly: boolean) {
    const p = a.path!;
    if (!p.turn || p.turn === 'straight' || p.xwalkS < 0) return Infinity;
    const front = a.s + a.spec.len / 2;
    const stopAt = p.xwalkS - 1.5;
    if (front > p.xwalkS + 1) return Infinity; // already on it
    const v = Math.max(3, this.speed(a));
    const tIn = Math.max(0, stopAt - front) / v, tOut = (p.xwalkS + 4 + a.spec.len - front) / v + 1.2;
    for (const b of this.agents) {
      if (!b.active || b.crashed || !b.path || (b.kind !== 'ped' && b.kind !== 'bicycle')) continue;
      const c = this.conflictsOf(p, b.path, 2.5).find((c) => c.sa > p.xwalkS - 3 && c.sa < p.xwalkS + 6);
      if (!c) continue;
      const dB = c.sb - b.s;
      if (dB < -2) continue; // already past our path
      const vB = b.v;
      const walking = vB > 0.2;
      if (!walking && Math.abs(dB) > 2) continue; // waiting at the kerb
      // Standing at the kerb for their own green: not stepping out.
      if (!walking && b.s < b.path.crossS && (b.path.axis === 'NS' ? this.signal.walkNS : this.signal.walkEW) !== 'G') continue;
      const t0 = Math.max(0, (dB - 2.2) / Math.max(vB, 0.2)), t1 = (dB + 2.2) / Math.max(vB, 0.2);
      if (t1 < tIn || t0 > tOut) continue;
      if (visibleOnly && !this.canSee(a, b)) continue;
      return Math.sqrt(2 * 3.0 * Math.max(0, stopAt - front));
    }
    return Infinity;
  }

  private curveLimit(a: Agent) {
    const p = a.path!;
    return p.vCurve[p.kOf(a.s + a.spec.len / 2)];
  }

  private pursuit(a: Agent, x: number, y: number, h: number, v: number) {
    const p = a.path!;
    const wb = a.spec.wb;
    const ld = 3 + 0.35 * Math.abs(v);
    const rx = x - Math.cos(h) * wb / 2, ry = y - Math.sin(h) * wb / 2;
    const T = p.at(a.s - wb / 2 + ld);
    const alpha = angleDiff(Math.atan2(T.y - ry, T.x - rx), h);
    const ldReal = Math.hypot(T.x - rx, T.y - ry);
    return Math.atan2(2 * wb * Math.sin(alpha), ldReal);
  }

  /** A motorcycle passing cars on their left keeps its speed within FILTER_DV of theirs. */
  private filterLimit(a: Agent, x: number, y: number, h: number) {
    const dx = Math.cos(h), dy = Math.sin(h);
    let lim = Infinity;
    for (const b of this.agents) {
      if (!b.active || b === a || (b.kind !== 'car' && b.kind !== 'truck')) continue;
      const [bx, by] = this.pos(b);
      const lon = (bx - x) * dx + (by - y) * dy, lat = -(bx - x) * dy + (by - y) * dx;
      if (lat > -0.5 || lat < -2.6 || lon < -b.spec.len / 2 - 1 || lon > b.spec.len / 2 + 8) continue;
      lim = Math.min(lim, Math.max(0, this.speed(b)) + FILTER_DV);
    }
    return lim;
  }

  private planVehicle(a: Agent) {
    const [x, y] = this.pos(a);
    const h = this.yaw(a);
    const turn = a.path!.waitS > 0;
    const v = Math.min(
      a.desired, this.curveLimit(a), this.stopLimit(a, turn), this.obstacleLimit(a),
      this.waitLimit(a, 3.5, false), this.walkerYieldLimit(a, false),
      a.kind === 'moto' ? this.filterLimit(a, x, y, h) : Infinity,
    );
    a.targetV = Math.max(0, v);
    if (a.kind === 'moto') {
      const p = a.path!;
      a.steerCmd = Math.atan(a.spec.wb * p.curv[p.kOf(a.s)]);
    } else {
      a.steerCmd = Math.max(-a.spec.maxSteer, Math.min(a.spec.maxSteer, this.pursuit(a, x, y, h, this.speed(a))));
    }
  }

  private planWalker(a: Agent) {
    const p = a.path!;
    const light = p.axis === 'NS' ? this.signal.walkNS : this.signal.walkEW;
    let v = a.desired;
    const front = a.s + a.spec.len / 2;
    if (front < p.crossS - 0.2) {
      if (light !== 'G') v = Math.min(v, Math.sqrt(2 * 1.5 * Math.max(0, p.crossS - 0.6 - front)));
    } else if (a.s < p.crossEndS && light !== 'G') {
      v *= a.kind === 'ped' ? 1.35 : 1.1; // hurry across
    }
    if (a.kind === 'ped') {
      const o = this.obstacleAhead(a, 2.5, false);
      if (o && o.gap < 1.2) v = Math.min(v, Math.max(0, o.vLead));
    } else {
      v = Math.min(v, this.obstacleLimit(a));
    }
    a.targetV = Math.max(0, v);
  }

  private planEgo(a: Agent) {
    const [x, y] = this.pos(a);
    const h = this.yaw(a);
    const v = this.speed(a);
    this.followLap(a, x, y);
    const p = a.path!;
    const L = LAP[this.egoLeg];
    const lcLimit = this.egoLcLocked || this.egoOff ? Infinity : this.laneChange(a, x, y, v);
    if (L.hug && this.egoHug === null && this.egoLcLocked && !this.egoOff) this.keepLeft(a, x, y, v);
    const sight: Sight[] = [];
    // Perception runs in every mode so the overhead map can show what the driver sees.
    this.gapOk(a, this.settings.gap, true, sight);
    this.egoSight = sight;
    if (this.egoMode === 'auto') {
      const lim = [
        a.desired, this.curveLimit(a), this.stopLimit(a, true), this.obstacleLimit(a),
        this.waitLimit(a, this.settings.gap, true), this.walkerYieldLimit(a, true), lcLimit,
      ];
      a.targetV = Math.max(0, Math.min(...lim));
      a.steerCmd = this.pursuit(a, x, y, h, v);
    } else {
      a.steerCmd = this.egoMode === 'pedal' ? this.pursuit(a, x, y, h, v) : this.manualSteer * steerLimit(Math.abs(v));
      if (p.waitS > 0 && a.s > p.waitS + 2) a.committed = true;
    }
    a.steerCmd = Math.max(-a.spec.maxSteer, Math.min(a.spec.maxSteer, a.steerCmd));

    const front = a.s + a.spec.len / 2;
    const inLane = p.lcS0 >= 0 && a.s > p.lcS0 - 1 && a.s < p.lcS1;
    const turning = p.waitS > 0 ? a.committed || a.s > p.waitS + 2 : a.s > p.arcS0 - 1;
    if (this.egoOff) this.egoPhase = 'free';
    else if (turning) this.egoPhase = L.turn;
    else if (!this.egoLcLocked || inLane) this.egoPhase = 'lane';
    else if (p.control === 'signal' && v < 1 && Math.abs(front - p.stopS) < 1.5 && this.lightFor(p.axis) !== 'G') this.egoPhase = 'red';
    else if (p.control === 'stop' && a.s > p.waitS - 12) this.egoPhase = 'stop';
    else if (p.waitS > 0 && a.s > p.waitS - 12 && v < 2) this.egoPhase = 'wait';
    else this.egoPhase = 'approach';
  }

  /** Move on to the next leg once the turn is done; in manual mode, find the lap again after straying. */
  private followLap(a: Agent, x: number, y: number) {
    const p = a.path!;
    if (p.project(x, y, p.kOf(a.s), 12, 30).d2 > 6 * 6) {
      this.egoOff = true;
      this.rejoinLap(a, x, y);
      return;
    }
    this.egoOff = false;
    if (a.s < p.arcS1 + 1) return;
    const next = (this.egoLeg + 1) % LAP.length;
    const N = LAP[next];
    const [cx, cy] = JUNCTION_POS[N.j];
    if ((x - cx) * Math.cos(N.h1) + (y - cy) * Math.sin(N.h1) < -LEG_START + 1.5) return;
    const done = LAP[this.egoLeg];
    this.stats.turns++;
    if (done.turn === 'right') this.stats.rights++;
    else this.stats.lefts++;
    if (next === 0) this.stats.laps++;
    this.setLeg(next, N.from !== undefined ? LC_LATEST : null);
  }

  /** Off the route (manual driving): pick up whichever leg the car is now driving along. */
  private rejoinLap(a: Agent, x: number, y: number) {
    const h = this.yaw(a);
    for (let i = 0; i < LAP.length; i++) {
      for (const t0 of LAP[i].from !== undefined ? [LC_LATEST, null] : [null]) {
        const p = this.legPath(i, t0);
        const pr = p.project(x, y, p.n >> 1, p.n, p.n);
        if (pr.d2 < 2.5 * 2.5 && pr.k * PATH_STEP < p.arcS1 && Math.cos(angleDiff(p.heading[pr.k], h)) > 0.8) {
          this.setLeg(i, t0);
          this.egoLcLocked = true;
          this.egoOff = false;
          return;
        }
      }
    }
  }

  /**
   * Lane change on the south arm before the main intersection: move over at the first safe moment
   * (a gap beside and behind in the other lane), or slow down and wait for one before the no-change zone.
   * Returns a speed limit.
   */
  private laneChange(a: Agent, x: number, y: number, v: number): number {
    const L = LAP[this.egoLeg];
    const [cx, cy] = JUNCTION_POS[L.j];
    const dx = Math.cos(L.h1), dy = Math.sin(L.h1);
    const t = (x - cx) * dx + (y - cy) * dy;
    const lock = (t0: number) => {
      this.egoLcLocked = true;
      this.stats.laneChanges++;
      if (t0 !== this.egoLcT0) this.setEgoPath(this.legPath(this.egoLeg, t0), a.s);
      this.egoLcT0 = t0;
    };
    if (this.egoMode === 'manual') {
      lock(LC_LATEST); // the driver steers
      return Infinity;
    }
    const t0 = Math.max(LC_EARLIEST, Math.ceil((t + 3 + 0.5 * Math.max(0, v)) / 2) * 2);
    const clear = this.otherLaneClear(a, x, y, dx, dy, L.o1 - L.from!);
    if (clear || (this.egoMode === 'pedal' && t0 >= LC_LATEST)) {
      lock(Math.min(t0, LC_LATEST));
      return Infinity;
    }
    // Hold back until there is room: stop short of where the lane change must begin.
    const sLatest = a.path!.lcS0;
    return Math.sqrt(2 * 2.5 * Math.max(0, sLatest - 1.5 - a.s));
  }

  /**
   * Before a left turn from a two-lane road: pull over to the kerb (setting), after checking the left
   * mirror: nothing alongside, nothing coming up fast behind — otherwise let it by first.
   * Once over, a motorcycle behind has no room and waits.
   */
  private keepLeft(a: Agent, x: number, y: number, v: number) {
    const L = LAP[this.egoLeg];
    const [cx, cy] = JUNCTION_POS[L.j];
    const dx = Math.cos(L.h1), dy = Math.sin(L.h1);
    const t = (x - cx) * dx + (y - cy) * dy;
    if (t < HUG_T0 - 3 - 0.8 * Math.max(0, v)) return;
    const t0 = Math.max(HUG_T0, Math.ceil((t + 3) / 2) * 2);
    if (this.egoMode === 'manual' || !this.settings.keepLeft || t0 > HUG_LATEST) {
      this.egoHug = false;
      return;
    }
    const ve = Math.max(0, v);
    const inTheWay = this.agents.some((b) => {
      if (!b.active || b === a || b.kind === 'ped') return false;
      const [bx, by] = this.pos(b);
      const lon = (bx - x) * dx + (by - y) * dy, lat = -(bx - x) * dy + (by - y) * dx;
      const vb = Math.max(0, b.kind === 'moto' || b.kind === 'bicycle' ? b.v : this.speed(b));
      return lon > -(8 + Math.max(0, vb - ve) * 3) && lon < 3 && lat > 0.5 && lat < 3;
    });
    if (inTheWay) return; // look again in a moment
    this.egoHug = true;
    this.setEgoPath(this.legPath(this.egoLeg, this.egoLcT0, LEG_START, t0), a.s);
  }

  /** Room in the lane `dlat` metres to the side: nothing alongside, nothing closing in from behind. */
  private otherLaneClear(a: Agent, x: number, y: number, dx: number, dy: number, dlat: number) {
    const ve = Math.max(0, this.speed(a));
    const h = Math.atan2(dy, dx);
    for (const b of this.agents) {
      if (!b.active || b === a || b.kind === 'ped' || b.kind === 'bicycle') continue;
      const [bx, by] = this.pos(b);
      const lon = (bx - x) * dx + (by - y) * dy, lat = -(bx - x) * dy + (by - y) * dx;
      if (Math.abs(lat - dlat) > 2.1 || lon > 40 || lon < -60) continue;
      if (!b.crashed && Math.cos(angleDiff(this.yaw(b), h)) < 0.5) continue;
      const vb = b.crashed ? 0 : Math.max(0, b.kind === 'moto' ? b.v : this.speed(b));
      const half = (a.spec.len + b.spec.len) / 2;
      // Ahead: room to finish the move, so we don't end up stopped across both lanes behind a queue.
      const ahead = half + 3 + Math.max(0, ve - vb) * 1.5 + (vb < 3 ? 22 : 0);
      if (lon > -(half + 4 + Math.max(0, vb - ve) * 2.5) && lon < ahead) return false;
    }
    return true;
  }

  // ── Per-step actuation ──────────────────────────────────

  private actuate() {
    const d = this.data;
    const ctrl = d.ctrl;
    for (const a of this.agents) {
      if (!a.active) continue;
      if (a.crashed) {
        for (const id of a.driveAct) ctrl[id] = 0;
        continue;
      }
      if (a.kind === 'car' || a.kind === 'truck') {
        const r = a.spec.r;
        let wv = a.wheelVel;
        if (a.ego && this.egoMode !== 'auto') {
          const p = this.pedal;
          if (p === 'accel') wv = Math.min(wv + (a.spec.accel * 1.3 / r) * SIM_DT, EGO_MAX_SPEED / r);
          else if (p === 'brake') {
            wv -= Math.sign(wv) * (a.spec.decel * 0.85 / r) * SIM_DT;
            if (Math.abs(wv) < 0.3) wv = 0;
          } else if (p === 'none' && wv > 0) wv = Math.max(0, wv - (0.3 / r) * SIM_DT);
        } else {
          const target = a.targetV / r;
          const up = (a.spec.accel / r) * SIM_DT, down = (a.spec.decel / r) * SIM_DT;
          wv += Math.max(-down, Math.min(up, target - wv));
        }
        a.wheelVel = wv;
        for (const id of a.steerAct) ctrl[id] = a.steerCmd;
        for (const id of a.driveAct) ctrl[id] = wv;
      } else {
        this.driveForced(a);
      }
    }
  }

  /** Force-driven agents (motorcycles, bicycles, pedestrians): track the path with a PD on the root. */
  private driveForced(a: Agent) {
    const d = this.data;
    const q = d.qpos, qv = d.qvel, xf = d.xfrc_applied, ctrl = d.ctrl;
    const up = a.spec.accel * SIM_DT, down = a.spec.decel * SIM_DT;
    a.v += Math.max(-down, Math.min(up, a.targetV - a.v));
    a.s += a.v * SIM_DT;
    const P = a.path!.at(a.s);
    const M = a.mass;
    const ped = a.kind === 'ped';
    const KP = 30, KD = 11;
    const c = Math.cos(P.h), sn = Math.sin(P.h);
    const fx = M * (KP * (P.x - q[a.fq]) + KD * (a.v * c - qv[a.fv]));
    const fy = M * (KP * (P.y - q[a.fq + 1]) + KD * (a.v * sn - qv[a.fv + 1]));
    const fz = ped ? M * (100 * (PELVIS_H + 0.03 - q[a.fq + 2]) - 20 * qv[a.fv + 2] + 9.81) : 0;
    // Orientation: upright, facing along the path
    const qc = [q[a.fq + 3], q[a.fq + 4], q[a.fq + 5], q[a.fq + 6]];
    let e = quatMul([Math.cos(P.h / 2), 0, 0, Math.sin(P.h / 2)], [qc[0], -qc[1], -qc[2], -qc[3]]);
    if (e[0] < 0) e = e.map((u) => -u);
    const [w, x, y, z] = qc;
    const wl = [qv[a.fv + 3], qv[a.fv + 4], qv[a.fv + 5]];
    // Local → world angular velocity
    const R = [
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    ];
    const ww = [0, 1, 2].map((i) => R[3 * i] * wl[0] + R[3 * i + 1] * wl[1] + R[3 * i + 2] * wl[2]);
    const Irp = ped ? 8 : a.kind === 'moto' ? 60 : 25;
    const Iy = ped ? 1.5 : a.kind === 'moto' ? 40 : 12;
    const KR = 150, KDR = 25;
    const b = 6 * a.root;
    xf[b] = fx; xf[b + 1] = fy; xf[b + 2] = fz;
    xf[b + 3] = Irp * (KR * 2 * e[1] - KDR * ww[0]);
    xf[b + 4] = Irp * (KR * 2 * e[2] - KDR * ww[1]);
    xf[b + 5] = Iy * (KR * 2 * e[3] - KDR * ww[2]);
    if (ped) {
      // Walking gait
      const amp = Math.min(1, a.v / 1.2);
      a.gait += (2 * Math.PI * a.v / 1.4) * SIM_DT;
      const s = Math.sin(a.gait);
      const [hl, hr] = a.humanAct.hip, [kl, kr] = a.humanAct.knee, [sl, sr] = a.humanAct.sh;
      ctrl[hl] = -0.42 * s * amp; ctrl[hr] = 0.42 * s * amp;
      ctrl[kl] = 0.05 + 0.55 * Math.max(0, Math.cos(a.gait)) * amp;
      ctrl[kr] = 0.05 + 0.55 * Math.max(0, -Math.cos(a.gait)) * amp;
      ctrl[sl] = 0.35 * s * amp; ctrl[sr] = -0.35 * s * amp;
    } else {
      for (const id of a.steerAct) ctrl[id] = a.steerCmd;
      for (const id of a.driveAct) ctrl[id] = a.v / a.spec.r;
    }
  }

  private clearForces(a: Agent) {
    const xf = this.data.xfrc_applied;
    for (const b of a.bodies) for (let i = 0; i < 6; i++) xf[6 * b + i] = 0;
  }

  // ── Collisions ──────────────────────────────────────────

  private crash(a: Agent) {
    if (a.crashed) return;
    a.crashed = true;
    a.crashT = this.time;
    a.stillSince = this.time;
    this.clearForces(a);
    this.setWeld(a, false); // rider thrown off
    this.setLimp(a, true);
    if (a.kind === 'ped' || a.kind === 'bicycle') this.setFullyCollidable(a);
    for (const id of a.driveAct) this.data.ctrl[id] = 0; // driver stamps on the brake
    if (a.ego) {
      this.egoPhase = 'crash';
      this.stats.egoCrashes++;
    }
  }

  private detectContacts() {
    const d = this.data;
    const ncon = d.ncon;
    if (!ncon) return;
    const contacts = d.contact;
    try {
      for (let i = 0; i < ncon; i++) {
        const c = contacts.get(i);
        if (!c) continue;
        const { geom1, geom2 } = c;
        c.delete();
        const a1 = this.geomAgent[geom1], a2 = this.geomAgent[geom2];
        if (a1 >= 0 && a2 >= 0 && a1 !== a2) this.onHit(a1, a2);
        else if (a1 >= 0 && a2 < 0 && this.objectGeom[geom2]) this.onHit(a1, -1);
        else if (a2 >= 0 && a1 < 0 && this.objectGeom[geom1]) this.onHit(a2, -1);
      }
    } finally {
      contacts.delete();
    }
  }

  private onHit(i: number, j: number) {
    const key = j < 0 ? `${i}|obj` : i < j ? `${i}|${j}` : `${j}|${i}`;
    if (this.seenPairs.has(key)) return;
    const A = this.agents[i], B = j >= 0 ? this.agents[j] : null;
    if (!A.active || (B && !B.active)) return;
    // Brushing contact is not an accident: people bumping into each other (or into someone lying
    // on the ground), or walking into a car at walking pace. A thrown body flying into another is.
    const walker = (x: Agent) => x.kind === 'ped' || x.kind === 'bicycle';
    if (B) {
      const [ax, ay, az] = this.vel(A), [bx, by, bz] = this.vel(B);
      const closing = Math.hypot(ax - bx, ay - by, az - bz);
      if (walker(A) && walker(B) && closing < 2) return;
      if (!A.crashed && !B.crashed && closing < 1) return;
    }
    this.seenPairs.add(key);
    const va = this.vel(A), vb = B ? this.vel(B) : [0, 0, 0];
    const speed = Math.hypot(va[0] - vb[0], va[1] - vb[1], va[2] - vb[2]);
    const [x, y] = this.pos(A);
    const ev: CrashEvent = { t: this.time, a: i, b: j, speed, ego: A.ego || !!B?.ego, x, y };
    // An agent brushing a pole at walking pace isn't an accident.
    if (!B && speed < 1.5 && !A.crashed) {
      this.seenPairs.delete(key);
      return;
    }
    this.events.push(ev);
    this.fresh.push(ev);
    this.stats.crashes++;
    this.crash(A);
    if (B) this.crash(B);
  }

  /** A car or truck on its side or roof: an accident in itself, alarm included, whether or not it hit anything. */
  private onRollover(a: Agent) {
    a.rolled = true;
    const [x, y] = this.pos(a);
    const ev: CrashEvent = { t: this.time, a: a.idx, b: -1, speed: Math.hypot(...this.vel(a)), ego: a.ego, x, y, rollover: true };
    this.events.push(ev);
    this.fresh.push(ev);
    this.stats.rollovers++;
    if (!a.crashed) this.stats.crashes++;
    this.crash(a);
  }

  /** One physics step. */
  step() {
    if (this.stepCount % PLAN_EVERY === 0) this.plan();
    this.actuate();
    this.mj.mj_step(this.model, this.data);
    this.stepCount++;
    this.detectContacts();
  }

  /** Forget old crash pairs so the same two bodies can crash again after a respawn. */
  clearPairsFor(i: number) {
    for (const k of [...this.seenPairs]) if (k.split('|').includes(String(i))) this.seenPairs.delete(k);
  }

  activeCount(kind: Kind) {
    return this.agents.filter((a) => a.active && a.kind === kind && !a.ego).length;
  }

  agentName(i: number) {
    return i < 0 ? 'object' : this.agents[i].ego ? 'ego' : this.agents[i].kind;
  }

  egoSpeed() {
    return this.speed(this.ego);
  }

  egoInfo() {
    const e = this.ego;
    const [x, y, z] = this.pos(e);
    return { x, y, z, yaw: this.yaw(e), speed: this.speed(e), steer: e.steerCmd, s: e.s };
  }
}

/** Steering authority shrinks with speed (manual mode). */
export function steerLimit(speed: number) {
  return Math.max(0.15, 1 / (1 + (speed / 12) ** 2));
}

