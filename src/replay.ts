// Drive recorder: keeps the last few minutes of the simulation as joint positions (qpos) so a run
// can be watched again from any camera. Playback rebuilds body and camera poses in a separate MjData
// with forward kinematics; the live simulation is not touched. Recordings save to / load from a file.
import type { MainModule, MjData, MjModel } from '@mujoco/mujoco';
import type { CrashEvent, IntersectionSim } from './sim.ts';

const RATE = 30; // frames per simulated second
const MAX_SECONDS = 300;
const MAGIC = 'N55R';

export class Recording {
  readonly nq: number;
  readonly nAgents: number;
  times: number[] = [];
  qpos: Float32Array[] = [];
  active: Uint8Array[] = [];
  events: CrashEvent[] = [];

  constructor(nq: number, nAgents: number) {
    this.nq = nq;
    this.nAgents = nAgents;
  }

  get length() {
    return this.times.length;
  }
  get start() {
    return this.times[0] ?? 0;
  }
  get end() {
    return this.times[this.times.length - 1] ?? 0;
  }

  clear() {
    this.times = [];
    this.qpos = [];
    this.active = [];
    this.events = [];
  }

  /** Record the current state if a frame is due; drop frames older than MAX_SECONDS. */
  capture(sim: IntersectionSim) {
    const t = sim.time;
    if (this.length && t - this.end < 1 / RATE - 1e-6) return;
    if (this.length && t < this.end) this.clear(); // the simulation was reset
    this.times.push(t);
    this.qpos.push(Float32Array.from(sim.data.qpos as Float64Array));
    const act = new Uint8Array(this.nAgents);
    for (let i = 0; i < this.nAgents; i++) act[i] = sim.isActive(i) ? 1 : 0;
    this.active.push(act);
    while (this.length > 2 && this.end - this.start > MAX_SECONDS) {
      this.times.shift();
      this.qpos.shift();
      this.active.shift();
    }
    this.events = sim.events.filter((e) => e.t >= this.start);
  }

  /** Index of the last frame at or before t. */
  frameAt(t: number) {
    let lo = 0, hi = this.length - 1;
    if (hi < 0) return -1;
    if (t <= this.times[0]) return 0;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.times[mid] <= t) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  }

  /** A single file: magic, header length, JSON header, then times (f64), qpos (f32) and active flags (u8). */
  toBlob(): Blob {
    const n = this.length;
    const header = new TextEncoder().encode(JSON.stringify({
      version: 1, nq: this.nq, nAgents: this.nAgents, frames: n, events: this.events,
    }));
    const pad = (8 - ((8 + header.length) % 8)) % 8;
    const head = new Uint8Array(8 + header.length + pad);
    head.set(new TextEncoder().encode(MAGIC), 0);
    new DataView(head.buffer).setUint32(4, header.length + pad, true);
    head.set(header, 8);
    head.fill(32, 8 + header.length); // pad the JSON with spaces
    const times = Float64Array.from(this.times);
    const q = new Float32Array(n * this.nq);
    this.qpos.forEach((f, i) => q.set(f, i * this.nq));
    const a = new Uint8Array(n * this.nAgents);
    this.active.forEach((f, i) => a.set(f, i * this.nAgents));
    return new Blob([head, times, q, a], { type: 'application/octet-stream' });
  }

  static fromBuffer(buf: ArrayBuffer, nq: number, nAgents: number): Recording {
    const bytes = new Uint8Array(buf);
    if (new TextDecoder().decode(bytes.subarray(0, 4)) !== MAGIC) throw new Error('not a replay file');
    const hlen = new DataView(buf).getUint32(4, true);
    const h = JSON.parse(new TextDecoder().decode(bytes.subarray(8, 8 + hlen)));
    if (h.version !== 1 || h.nq !== nq || h.nAgents !== nAgents) throw new Error('recorded with a different world');
    const n: number = h.frames;
    let off = 8 + hlen;
    const times = new Float64Array(buf, off, n);
    off += n * 8;
    const q = new Float32Array(buf, off, n * nq);
    off += n * nq * 4;
    const a = new Uint8Array(buf, off, n * nAgents);
    const r = new Recording(nq, nAgents);
    r.times = Array.from(times);
    for (let i = 0; i < n; i++) {
      r.qpos.push(q.slice(i * nq, (i + 1) * nq));
      r.active.push(a.slice(i * nAgents, (i + 1) * nAgents));
    }
    r.events = h.events ?? [];
    return r;
  }
}

/** Poses a separate MjData from a recording, interpolating between frames. */
export class Player {
  readonly data: MjData;
  private mj: MainModule;
  private model: MjModel;
  private quatAdr: number[] = []; // qpos addresses of free-joint quaternions (re-normalised after blending)
  private hingeAdr: number[] = [];
  active: Uint8Array = new Uint8Array(0);

  constructor(mj: MainModule, model: MjModel) {
    this.mj = mj;
    this.model = model;
    this.data = new mj.MjData(model);
    for (let j = 0; j < model.njnt; j++) {
      if (model.jnt_type[j] === 0) this.quatAdr.push(model.jnt_qposadr[j] + 3);
      else this.hingeAdr.push(model.jnt_qposadr[j]);
    }
  }

  /** Set the pose at time t and run forward kinematics (bodies and cameras). */
  seek(rec: Recording, t: number) {
    const i = rec.frameAt(t);
    if (i < 0) return;
    const j = Math.min(i + 1, rec.length - 1);
    const t0 = rec.times[i], t1 = rec.times[j];
    // Agents that appear or vanish between the two frames were teleported: don't blend them.
    const u = j > i && t1 > t0 && rec.active[i].every((v, k) => v === rec.active[j][k]) ? Math.max(0, Math.min(1, (t - t0) / (t1 - t0))) : 0;
    const a = rec.qpos[i], b = rec.qpos[j], q = this.data.qpos;
    for (let k = 0; k < a.length; k++) q[k] = a[k] + (b[k] - a[k]) * u;
    for (const k of this.quatAdr) {
      // Blend along the shorter way round, then normalise
      if (a[k] * b[k] + a[k + 1] * b[k + 1] + a[k + 2] * b[k + 2] + a[k + 3] * b[k + 3] < 0) {
        for (let m = 0; m < 4; m++) q[k + m] = a[k + m] - (b[k + m] + a[k + m]) * u;
      }
      const n = Math.hypot(q[k], q[k + 1], q[k + 2], q[k + 3]) || 1;
      for (let m = 0; m < 4; m++) q[k + m] /= n;
    }
    this.active = rec.active[i];
    this.data.time = t;
    this.mj.mj_kinematics(this.model, this.data);
    this.mj.mj_comPos(this.model, this.data);
    this.mj.mj_camlight(this.model, this.data);
  }

  /** Ego speed around t, from the recorded positions [m/s]. */
  egoSpeed(rec: Recording, t: number, fq: number) {
    const i = rec.frameAt(t), j = Math.min(i + 1, rec.length - 1);
    if (i < 0 || j === i) return 0;
    const a = rec.qpos[i], b = rec.qpos[j];
    return Math.hypot(b[fq] - a[fq], b[fq + 1] - a[fq + 1]) / (rec.times[j] - rec.times[i]);
  }
}
