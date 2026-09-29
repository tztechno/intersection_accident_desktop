// Headless checks in Node: the world builds, the ego drives the figure-of-eight lap on its own
// (right and left turns, lane changes), traffic flows safely with default settings, and an aggressive
// setup produces collisions that MuJoCo keeps simulating.
import loadMujoco from '@mujoco/mujoco';
import { IntersectionSim } from '../src/sim.ts';
import { SIM_DT } from '../src/world.ts';
import { LAP, OUTER, PATH_STEP } from '../src/layout.ts';

const mj = await loadMujoco();
let failed = false;
const check = (ok: boolean, msg: string) => {
  console.log(`${ok ? 'OK  ' : 'FAIL'} ${msg}`);
  if (!ok) failed = true;
};
const finite = (sim: IntersectionSim) => sim.data.qpos.every((v: number) => Number.isFinite(v));

// 1. Empty roads: the ego drives a whole lap on its own, on its wheels, without touching anything.
{
  const sim = new IntersectionSim(mj);
  const m = sim.model;
  console.log('nbody', m.nbody, 'nv', m.nv, 'ngeom', m.ngeom, 'nu', m.nu);
  Object.assign(sim.settings, { oncoming: 0, cross: 0, peds: 0, bikes: 0 });
  sim.reset();
  let maxZ = 0, maxOff = 0;
  const phases = new Set<string>();
  for (let i = 0; i < 240 / SIM_DT && sim.stats.laps === 0; i++) {
    sim.step();
    const e = sim.egoInfo();
    maxZ = Math.max(maxZ, e.z);
    phases.add(sim.egoPhase);
    if (i % 25 === 0) {
      const p = sim.ego.path!;
      maxOff = Math.max(maxOff, Math.sqrt(p.project(e.x, e.y, p.kOf(e.s), 12, 30).d2));
    }
  }
  console.log(`lap: ${sim.time.toFixed(1)} s, stats ${JSON.stringify(sim.stats)}, phases ${[...phases].join(' ')}`);
  check(sim.stats.laps === 1, `ego completes a figure-of-eight lap on empty roads (${sim.time.toFixed(1)} s)`);
  check(sim.stats.rights === 4 && sim.stats.lefts === 4, `four right and four left turns (${sim.stats.rights} / ${sim.stats.lefts})`);
  check(sim.stats.laneChanges >= 1, `changes lanes on the south arm (${sim.stats.laneChanges})`);
  check(sim.events.length === 0, 'no collisions on empty roads');
  check(maxZ < 0.6, `ego stays on its wheels (max z ${maxZ.toFixed(2)} m)`);
  check(maxOff < 1.5,`ego tracks its lane (max ${maxOff.toFixed(2)} m off)`);
  check(!phases.has('free'), 'never leaves the loop');
  check(LAP.length === 8, 'lap has eight turns');
}

// 2. Default settings, busy intersection, 3 simulated minutes: flows, and the ego drives safely.
{
  const sim = new IntersectionSim(mj);
  Object.assign(sim.settings, { oncoming: 24, cross: 12, peds: 20, bikes: 8 });
  sim.reset();
  const t0 = performance.now();
  let maxActive = 0;
  for (let i = 0; i < 180 / SIM_DT; i++) {
    sim.step();
    if (i % 250 === 0) maxActive = Math.max(maxActive, sim.agents.filter((a) => a.active).length);
  }
  const wall = (performance.now() - t0) / 1000;
  console.log(`busy: ${sim.time.toFixed(0)} s in ${wall.toFixed(1)} s wall (${(sim.time / wall).toFixed(1)}× real time), ` +
    `max on stage ${maxActive}, stats ${JSON.stringify(sim.stats)}`);
  check(finite(sim), 'simulation stays finite');
  check(sim.stats.spawned > 60, `traffic spawns (${sim.stats.spawned})`);
  check(sim.stats.turns >= 5, `ego keeps going round (${sim.stats.rights} right, ${sim.stats.lefts} left, ${sim.stats.laneChanges} lane changes)`);
  check(sim.stats.egoCrashes === 0, `careful ego (4.5 s gap) does not crash (${sim.stats.egoCrashes})`);
  check(sim.stats.crashes <= 3, `collisions are rare with default settings (${sim.stats.crashes})`);
}

// 3. Aggressive driver, fast traffic, slow reactions: accidents happen and play out physically.
//    Two runs pooled, so one quiet stretch of traffic doesn't decide the result.
{
  const moved: number[] = [];
  let collisions = 0, fast = 0, egoCrashes = 0, turns = 0, ok = true;
  for (const seed of [7, 8]) {
    const sim = new IntersectionSim(mj, undefined, seed);
    Object.assign(sim.settings, { oncoming: 24, cross: 8, peds: 16, bikes: 8, speed: 65, motoPct: 40, gap: 1.5, reaction: 1.4 });
    sim.reset();
    let seen = 0;
    const watch: { i: number; t: number; x: number; y: number; d: number }[] = [];
    for (let i = 0; i < 300 / SIM_DT; i++) {
      sim.step();
      while (seen < sim.events.length) {
        const e = sim.events[seen++];
        for (const k of [e.a, e.b]) {
          if (k < 0) continue;
          const [x, y] = sim.pos(sim.agents[k]);
          watch.push({ i: k, t: e.t, x, y, d: 0 });
        }
      }
      for (const w of watch) {
        const a = sim.agents[w.i];
        if (!a.active || sim.time - w.t > 3) continue;
        const [x, y] = sim.pos(a);
        w.d = Math.max(w.d, Math.hypot(x - w.x, y - w.y));
      }
    }
    for (const w of watch) moved.push(w.d);
    collisions += sim.events.length;
    fast += sim.events.filter((e) => e.speed > 20 / 3.6).length;
    egoCrashes += sim.stats.egoCrashes;
    turns += sim.stats.turns;
    ok &&= finite(sim);
    for (const e of sim.events.slice(0, 4)) {
      console.log(`  seed ${seed} t=${e.t.toFixed(1)} ${sim.agentName(e.a)} × ${sim.agentName(e.b)} ${(e.speed * 3.6).toFixed(0)} km/h`);
    }
  }
  console.log(`aggressive (2 runs): ${collisions} collisions (${fast} over 20 km/h), ego ${egoCrashes}, turns ${turns}`);
  check(ok, 'simulation stays finite through crashes');
  check(egoCrashes >= 1, 'an aggressive ego crashes');
  check(fast >= 1, 'at least one collision above 20 km/h');
  check(moved.some((d) => d > 1), `bodies keep moving after impact (max ${Math.max(...moved).toFixed(1)} m in 3 s)`);
}

// 4. Left turns and motorcycles: the ego heads for the left turn at the main intersection on a green and a
//    motorcycle comes up behind it by the kerb. Pulled over, the motorcycle has no room and waits behind;
//    turning from the middle of the lane leaves room on the left, and the car hooks it at the corner.
{
  const run = (keepLeft: number) => {
    const sim = new IntersectionSim(mj);
    Object.assign(sim.settings, { oncoming: 0, cross: 0, peds: 0, bikes: 0, keepLeft });
    sim.reset();
    const S = sim as any; // test access to the spawner
    let moto: any = null;
    let passedOnLeft = false;
    for (let i = 0; i < 400 / SIM_DT && !sim.events.length; i++) {
      sim.step();
      const e = sim.egoInfo();
      if (!moto && sim.egoLeg === 4 && e.y > -60 && sim.signal.ns === 'G' && sim.signal.remaining > 12) {
        moto = S.free('moto');
        const path = S.route('S', 'straight', OUTER + 1.7, OUTER + 1.7);
        const k = path.project(e.x, e.y, path.n >> 1, path.n, path.n).k;
        S.place(moto, path, k * PATH_STEP - 20, 35 / 3.6);
        moto.desired = 35 / 3.6;
      }
      if (moto?.active && sim.egoLeg === 4) {
        const [mx, my] = sim.pos(moto);
        const lon = (mx - e.x) * Math.cos(e.yaw) + (my - e.y) * Math.sin(e.yaw);
        if (Math.abs(lon) < 1.5) passedOnLeft = true;
      }
      if (moto && sim.egoLeg !== 4) break;
    }
    const hook = sim.events.find((e) => e.ego && [e.a, e.b].includes(moto?.idx));
    return { hook, passedOnLeft };
  };
  const on = run(1), off = run(0);
  console.log(`motorcycle behind a left-turning car: pulled over ${on.hook ? 'HOOKED' : 'safe'}, ` +
    `not pulled over ${off.hook ? `HOOKED at ${off.hook.x.toFixed(1)},${off.hook.y.toFixed(1)} ${(off.hook.speed * 3.6).toFixed(0)} km/h` : 'safe'}`);
  check(!on.hook && !on.passedOnLeft, 'pulled over: the motorcycle waits behind');
  check(off.passedOnLeft, 'not pulled over: the motorcycle comes up on the left');
  check(!!off.hook && Math.abs(off.hook.y) < 14, 'not pulled over: the car hooks it at the corner');
}

// 5. Rollover: a car on its side raises an accident (alarm) and stops driving.
{
  const sim = new IntersectionSim(mj);
  Object.assign(sim.settings, { oncoming: 0, cross: 0, peds: 0, bikes: 0 });
  sim.reset();
  for (let i = 0; i < 1 / SIM_DT; i++) sim.step();
  const q = sim.data.qpos, i0 = sim.ego.fq;
  const h = sim.egoInfo().yaw;
  // Lay the car on its left side where it is
  // yaw h, then a 90° roll: q = qz(h) · qx(90°)
  const cz = Math.cos(h / 2), sz = Math.sin(h / 2), c = Math.cos(Math.PI / 4), s = Math.sin(Math.PI / 4);
  const qm = [cz * c, cz * s, sz * s, sz * c];
  q[i0 + 2] = 1.2;
  for (let k = 0; k < 4; k++) q[i0 + 3 + k] = qm[k];
  for (let i = 0; i < 2 / SIM_DT; i++) sim.step();
  const ev = sim.events.find((e) => e.rollover);
  check(!!ev && ev.ego, 'a rollover is reported as an accident');
  check(sim.egoPhase === 'crash' && sim.ego.crashed, 'the rolled car stops driving');
  check(sim.stats.rollovers === 1, `counted once (${sim.stats.rollovers})`);
}

process.exit(failed ? 1 : 0);
