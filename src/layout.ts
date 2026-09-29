// Intersection geometry, vehicle / pedestrian routes and the signal cycle.
// World frame: x east, y north, z up, origin at the centre of the intersection.
// Traffic keeps left (Japan). Each road has two lanes per direction.

export const LANE_W = 3.5;
export const INNER = 1.75; // inner lane centre, from the road centreline
export const OUTER = 5.25; // outer lane centre
export const ROAD_HALF = 7.5; // centreline to kerb
export const WALK_OUT = 12.5; // outer edge of sidewalk (= crosswalk far edge)
export const WALK_MID = (ROAD_HALF + WALK_OUT) / 2; // pedestrian / bicycle line
export const STOP_LINE = 14; // distance of stop lines from the centre
export const CORNER_R = 2.5; // kerb radius at the corners
export const ARM = 150; // road length from the centre
export const BUILDING_LINE = 14.5;

export type Arm = 'N' | 'S' | 'E' | 'W';
export type Turn = 'straight' | 'left' | 'right';
export type Axis = 'NS' | 'EW';
/** How a junction is controlled for a path: traffic signal, stop sign, give way to oncoming, or nothing. */
export type Control = 'signal' | 'stop' | 'yield' | 'none';

/** Travel heading of a vehicle arriving from an arm (arriving from S means heading north). */
export const ARM_HEADING: Record<Arm, number> = { S: Math.PI / 2, N: -Math.PI / 2, W: 0, E: Math.PI };
export const axisOf = (a: Arm): Axis => (a === 'N' || a === 'S' ? 'NS' : 'EW');

export function angleDiff(a: number, b: number) {
  let d = a - b;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

// ── Paths ─────────────────────────────────────────────────

export const PATH_STEP = 0.5;
let pathSerial = 0;

/** A polyline sampled every PATH_STEP metres, with heading and curvature. */
export class Path {
  readonly id = pathSerial++;
  readonly x: Float64Array;
  readonly y: Float64Array;
  readonly heading: Float64Array;
  readonly curv: Float64Array;
  /** Comfortable speed profile from curvature, braking ahead of bends [m/s]. */
  readonly vCurve: Float64Array;
  readonly n: number;
  readonly length: number;
  /** Arc lengths of interesting points (-1 when not applicable). */
  stopS = -1; // stop line
  waitS = -1; // right-turn waiting point inside the intersection
  crossS = -1; // pedestrian: entering the crosswalk
  crossEndS = -1; // pedestrian: leaving the crosswalk
  exitS = -1; // leaving the intersection box
  xwalkS = -1; // entering the crosswalk on the exit arm (turning routes)
  arcS0 = -1; // start / end of the turning arc
  arcS1 = -1;
  lcS0 = -1; // start / end of a lane change on the approach
  lcS1 = -1;
  turn: Turn | null = null;
  axis: Axis = 'NS'; // signal that governs this path
  control: Control = 'signal';
  keepsLeft = false; // pulls over to the kerb before its left turn (motorcycles don't filter past)
  label = '';

  constructor(pts: [number, number][], latAccel = 2.2) {
    // Resample at a fixed step
    const xs: number[] = [pts[0][0]], ys: number[] = [pts[0][1]];
    let carry = 0;
    for (let i = 1; i < pts.length; i++) {
      const [x0, y0] = pts[i - 1], [x1, y1] = pts[i];
      const seg = Math.hypot(x1 - x0, y1 - y0);
      let d = PATH_STEP - carry;
      while (d <= seg) {
        xs.push(x0 + ((x1 - x0) * d) / seg);
        ys.push(y0 + ((y1 - y0) * d) / seg);
        d += PATH_STEP;
      }
      carry = seg - (d - PATH_STEP);
    }
    const n = xs.length;
    this.n = n;
    this.length = (n - 1) * PATH_STEP;
    this.x = Float64Array.from(xs);
    this.y = Float64Array.from(ys);
    this.heading = new Float64Array(n);
    this.curv = new Float64Array(n);
    for (let k = 0; k < n; k++) {
      const a = Math.max(0, k - 1), b = Math.min(n - 1, k + 1);
      this.heading[k] = Math.atan2(ys[b] - ys[a], xs[b] - xs[a]);
    }
    for (let k = 0; k < n; k++) {
      const a = Math.max(0, k - 2), b = Math.min(n - 1, k + 2);
      this.curv[k] = b > a ? angleDiff(this.heading[b], this.heading[a]) / ((b - a) * PATH_STEP) : 0;
    }
    const v = new Float64Array(n);
    for (let k = 0; k < n; k++) v[k] = Math.min(40, Math.sqrt(latAccel / Math.max(Math.abs(this.curv[k]), 1e-4)));
    for (let k = n - 2; k >= 0; k--) v[k] = Math.min(v[k], Math.sqrt(v[k + 1] ** 2 + 2 * 2.0 * PATH_STEP));
    this.vCurve = v;
  }

  clampK(k: number) {
    return Math.max(0, Math.min(this.n - 1, k));
  }
  kOf(s: number) {
    return this.clampK(Math.round(s / PATH_STEP));
  }

  /** Interpolated point at arc length s. */
  at(s: number): { x: number; y: number; h: number } {
    const f = Math.max(0, Math.min(this.n - 1.001, s / PATH_STEP));
    const k = Math.floor(f), t = f - k;
    return {
      x: this.x[k] + (this.x[k + 1] - this.x[k]) * t,
      y: this.y[k] + (this.y[k + 1] - this.y[k]) * t,
      h: this.heading[k] + angleDiff(this.heading[k + 1], this.heading[k]) * t,
    };
  }

  /** Nearest sample to (x, y) within [k0 - back, k0 + ahead]; lat > 0 means left of the path. */
  project(px: number, py: number, k0: number, back = 20, ahead = 40): { k: number; lat: number; d2: number } {
    let best = -1, bd = Infinity;
    const a = this.clampK(k0 - back), b = this.clampK(k0 + ahead);
    for (let k = a; k <= b; k++) {
      const d = (this.x[k] - px) ** 2 + (this.y[k] - py) ** 2;
      if (d < bd) { bd = d; best = k; }
    }
    const h = this.heading[best];
    const lat = -Math.sin(h) * (px - this.x[best]) + Math.cos(h) * (py - this.y[best]);
    return { k: best, lat, d2: bd };
  }

  /** First arc length where the signed distance along `dir` (measured from (cx, cy)) passes `t`. */
  sWhere(dirx: number, diry: number, t: number, cx = 0, cy = 0) {
    for (let k = 0; k < this.n; k++) if ((this.x[k] - cx) * dirx + (this.y[k] - cy) * diry >= t) return k * PATH_STEP;
    return -1;
  }
}

const hd = (h: number): [number, number] => [Math.cos(h), Math.sin(h)];
const leftN = (h: number): [number, number] => [-Math.sin(h), Math.cos(h)];

export function exitArm(approach: Arm, turn: Turn): Arm {
  const order: Arm[] = ['S', 'W', 'N', 'E']; // counter-clockwise, seen as approaches
  const i = order.indexOf(approach);
  if (turn === 'straight') return order[(i + 2) % 4];
  // Arriving from S (heading north), a left turn leaves towards the west arm.
  return turn === 'left' ? order[(i + 1) % 4] : order[(i + 3) % 4];
}

export const TURN_RADIUS: Record<Turn, number> = { straight: 0, left: 5.5, right: 9.25 };

export interface TurnSpec {
  cx: number; cy: number; // junction centre, where the two road centrelines cross
  h1: number; // approach heading
  turn: Turn;
  o1: number; o2: number; // entry / exit lane offset from the road centreline, to the left of travel
  R: number; // turning radius
  startDist: number; endDist: number; // route start before / end after the junction centre
  /** Lane change on the approach: from offset `from`, starting `t0` (signed, < 0) from the centre, over `len` m. */
  lc?: { from: number; t0: number; len: number };
  /** Keeping left before a left turn: move `d` m towards the kerb within the lane, from `t0` over `len` m. */
  hug?: { d: number; t0: number; len: number };
}

/** Before a left turn, drivers pull over this far to the kerb side of the lane, so nothing fits past on the left. */
export const HUG = 0.9;
export const HUG_T0 = -36; // earliest start
export const HUG_LATEST = -28; // latest start, to be over before the turn
export const HUG_LEN = 15;
export const HUG_TIGHTEN = 0.5; // pulled over, turn this much tighter so the inside of the car clears the corner
const ease = (u: number) => (1 - Math.cos(Math.PI * Math.max(0, Math.min(1, u)))) / 2;

/** A lane route through any junction: straight approach (with an optional lane change), arc, exit lane. */
export function buildTurnPath(sp: TurnSpec): Path {
  const { cx, cy, h1, turn, o2, lc, hug } = sp;
  const o1 = sp.o1 + (hug ? hug.d : 0); // where the turn starts from
  // Pulled over to the kerb, the turn centre moves out towards the sidewalk corner; turning a little
  // tighter brings it back so the inside of the car clears the corner.
  const R = sp.R - (hug ? HUG_TIGHTEN : 0);
  const [d1x, d1y] = hd(h1), [n1x, n1y] = leftN(h1);
  const pts: [number, number][] = [];
  const line = (h: number, o: number, t: number): [number, number] => {
    const [dx, dy] = hd(h), [nx, ny] = leftN(h);
    return [cx + nx * o + dx * t, cy + ny * o + dy * t];
  };
  // Lateral offset along the approach: the lane change (lc.from → sp.o1), then pulling over (hug)
  const off = (t: number) => {
    const lane = lc ? lc.from + (sp.o1 - lc.from) * ease((t - lc.t0) / lc.len) : sp.o1;
    return lane + (hug ? hug.d * ease((t - hug.t0) / hug.len) : 0);
  };
  const shifting = (t: number) =>
    (lc && t >= lc.t0 - 2 && t < lc.t0 + lc.len + 2) || (hug && t >= hug.t0 - 2 && t < hug.t0 + hug.len + 2);
  const approach = (tEnd: number, inclusive: boolean) => {
    for (let t = -sp.startDist; inclusive ? t <= tEnd : t < tEnd;) {
      pts.push(line(h1, off(t), t));
      t += shifting(t) ? 1 : 2;
    }
  };
  let tArc = 0;
  if (turn === 'straight') {
    approach(sp.endDist, true);
  } else {
    const sign = turn === 'left' ? 1 : -1;
    const h2 = h1 + (sign * Math.PI) / 2;
    const [d2x, d2y] = hd(h2), [n2x, n2y] = leftN(h2);
    // Corner of the two lane lines: along line 1, t* = projection of the exit lane offset.
    const tStar = n2x * o2 * d1x + n2y * o2 * d1y;
    const kx = n1x * o1 + d1x * tStar, ky = n1y * o1 + d1y * tStar;
    const t1x = kx - d1x * R, t1y = ky - d1y * R;
    tArc = tStar - R;
    approach(tArc, false);
    pts.push([cx + t1x, cy + t1y]);
    const ox = t1x + sign * n1x * R, oy = t1y + sign * n1y * R;
    const a0 = Math.atan2(t1y - oy, t1x - ox);
    for (let i = 1; i <= 24; i++) {
      const a = a0 + (sign * (Math.PI / 2) * i) / 24;
      pts.push([cx + ox + R * Math.cos(a), cy + oy + R * Math.sin(a)]);
    }
    // Continue along the exit lane
    const t2x = kx + d2x * R, t2y = ky + d2y * R;
    const u0 = t2x * d2x + t2y * d2y;
    for (let u = u0 + 2; u <= sp.endDist; u += 2) pts.push(line(h2, o2, u));
  }
  const p = new Path(pts);
  p.turn = turn;
  p.keepsLeft = !!hug;
  p.axis = Math.abs(d1x) > 0.5 ? 'EW' : 'NS';
  if (turn !== 'straight') {
    p.arcS0 = p.sWhere(d1x, d1y, tArc - 0.01, cx, cy);
    p.arcS1 = p.arcS0 + (R * Math.PI) / 2;
  }
  if (lc) {
    p.lcS0 = p.sWhere(d1x, d1y, lc.t0, cx, cy);
    p.lcS1 = p.sWhere(d1x, d1y, lc.t0 + lc.len, cx, cy);
  }
  return p;
}

/** Stop line, right-turn waiting point, crosswalk and box exit for a route through the main intersection. */
function markCentre(p: Path, h1: number, turn: Turn, R: number) {
  const [d1x, d1y] = hd(h1);
  p.control = 'signal';
  p.stopS = p.sWhere(d1x, d1y, -STOP_LINE);
  if (turn === 'right') {
    const sArc0 = p.sWhere(d1x, d1y, -ROAD_HALF);
    // Wait just into the turn, nose angled but still clear of the opposing inner lane.
    p.waitS = sArc0 + 0.1 * (R * Math.PI) / 2;
  }
  // Leaving the box: the first point outside |x|,|y| < ROAD_HALF once inside it (the exit crosswalk)
  let inBox = false;
  for (let k = p.kOf(p.stopS) + 4; k < p.n; k++) {
    const r = Math.max(Math.abs(p.x[k]), Math.abs(p.y[k]));
    if (r < ROAD_HALF) inBox = true;
    if (!inBox) continue;
    if (p.xwalkS < 0 && r > ROAD_HALF) p.xwalkS = k * PATH_STEP;
    if (r > ROAD_HALF + 4.5) { p.exitS = k * PATH_STEP; break; }
  }
}

/**
 * Lane route through the main intersection. o1 / o2: lateral offset of the entry / exit lane from the
 * road centreline (to the left of travel). startDist: distance from the centre where the route starts.
 */
export function buildRoute(approach: Arm, turn: Turn, o1: number, o2 = o1, startDist = ARM, endDist = ARM, keepLeft = false): Path {
  const h1 = ARM_HEADING[approach];
  const R = TURN_RADIUS[turn];
  const hug = keepLeft && turn === 'left' ? { d: HUG, t0: HUG_T0, len: HUG_LEN } : undefined;
  const p = buildTurnPath({ cx: 0, cy: 0, h1, turn, o1, o2, R, startDist, endDist, hug });
  p.axis = axisOf(approach);
  p.label = `${approach}-${turn}`;
  markCentre(p, h1, turn, R);
  return p;
}

/**
 * Pedestrian / bicycle line along a sidewalk that crosses one road on a crosswalk.
 * along: 'y' walks north-south on x = side·WALK_MID (crossing the east-west road).
 */
export function buildWalk(along: 'x' | 'y', side: 1 | -1, dir: 1 | -1, lat: number, from = 45, to = 45): Path {
  const c = side * WALK_MID + lat;
  const pts: [number, number][] = [];
  for (let t = -from; t <= to; t += 1) pts.push(along === 'y' ? [c, dir * t] : [dir * t, c]);
  const p = new Path(pts, 100);
  // Crossing the east-west road (walking along y) goes with the north-south signal.
  p.axis = along === 'y' ? 'NS' : 'EW';
  p.crossS = from - ROAD_HALF;
  p.crossEndS = from + ROAD_HALF;
  p.label = `walk-${along}${side > 0 ? '+' : '-'}${dir > 0 ? '>' : '<'}`;
  return p;
}

export interface Conflict {
  sa: number; sb: number; // closest approach
  sa0: number; sb0: number; // where the paths first come within range (the merge point when one joins the other's lane)
}

/** Arc-length pairs where two paths come within `dist` metres of each other. */
export function conflicts(a: Path, b: Path, dist: number): Conflict[] {
  const out: Conflict[] = [];
  const d2 = dist * dist;
  let inside = false;
  let best = { sa: 0, sb: 0, d: Infinity };
  let entry = { sa: 0, sb: 0 };
  for (let i = 0; i < a.n; i += 2) {
    let md = Infinity, mj = -1;
    for (let j = 0; j < b.n; j += 2) {
      const d = (a.x[i] - b.x[j]) ** 2 + (a.y[i] - b.y[j]) ** 2;
      if (d < md) { md = d; mj = j; }
    }
    if (md < d2) {
      if (!inside) entry = { sa: i * PATH_STEP, sb: mj * PATH_STEP };
      if (!inside || md < best.d) best = { sa: i * PATH_STEP, sb: mj * PATH_STEP, d: md };
      inside = true;
    } else if (inside) {
      out.push({ sa: best.sa, sb: best.sb, sa0: entry.sa, sb0: entry.sb });
      inside = false;
      best = { sa: 0, sb: 0, d: Infinity };
    }
  }
  if (inside) out.push({ sa: best.sa, sb: best.sb, sa0: entry.sa, sb0: entry.sb });
  return out;
}

// ── Signals ───────────────────────────────────────────────

export type Light = 'G' | 'Y' | 'R';
export type Walk = 'G' | 'F' | 'R'; // F = flashing green

interface Phase { ns: Light; ew: Light; nsArrow?: boolean; ewArrow?: boolean; dur: number }

export const PHASES: Phase[] = [
  { ns: 'G', ew: 'R', dur: 26 },
  { ns: 'Y', ew: 'R', dur: 3 },
  { ns: 'R', ew: 'R', nsArrow: true, dur: 6 },
  { ns: 'R', ew: 'R', dur: 2 },
  { ns: 'R', ew: 'G', dur: 20 },
  { ns: 'R', ew: 'Y', dur: 3 },
  { ns: 'R', ew: 'R', ewArrow: true, dur: 5 },
  { ns: 'R', ew: 'R', dur: 2 },
];
export const CYCLE = PHASES.reduce((a, p) => a + p.dur, 0);
const WALK_FLASH = 6; // pedestrian green flashes this long before it ends

export interface SignalState {
  ns: Light; ew: Light; nsArrow: boolean; ewArrow: boolean;
  walkNS: Walk; walkEW: Walk; // walkNS: crosswalks used while north-south traffic has green
  phase: number; remaining: number;
}

export function signalAt(t: number): SignalState {
  let u = ((t % CYCLE) + CYCLE) % CYCLE;
  let i = 0;
  while (u >= PHASES[i].dur) u -= PHASES[i++].dur;
  const p = PHASES[i];
  const walk = (on: boolean): Walk => (!on ? 'R' : p.dur - u > WALK_FLASH ? 'G' : 'F');
  return {
    ns: p.ns, ew: p.ew, nsArrow: !!p.nsArrow, ewArrow: !!p.ewArrow,
    walkNS: walk(p.ns === 'G'), walkEW: walk(p.ew === 'G'),
    phase: i, remaining: p.dur - u,
  };
}

// ── Town loop ─────────────────────────────────────────────
//
//   W ─────┬───── C ─────┬───── E     C: the signalised main intersection
//          │ west  │ east │
//          │ block │ block│           side streets: one lane each way, stop signs
//          └────── S ─────┘           where they meet the main roads
//
// The ego drives a figure of eight: the east block clockwise (right turns), then the west block
// anticlockwise (left turns), coming up the south arm into the main intersection every time.
// Right turns leave it in the inner lane and left turns in the outer lane, so on the south arm it
// always has to change lanes before the next turn at the main intersection.

export const BLOCK = 90; // block size: distance between junctions
export const SIDE_LANE = 1.5; // side-street lane centre from its centreline
export const SIDE_HALF = 3; // side-street centreline to kerb
export const SIDE_WALK = 5; // side-street centreline to the outer sidewalk edge
export const SIDE_STOP = 10; // side-street stop line, from the junction centre

export type JunctionId = 'C' | 'E' | 'W' | 'S' | 'SE' | 'SW';
export const JUNCTION_POS: Record<JunctionId, [number, number]> = {
  C: [0, 0], E: [BLOCK, 0], W: [-BLOCK, 0], S: [0, -BLOCK], SE: [BLOCK, -BLOCK], SW: [-BLOCK, -BLOCK],
};

/** Side-street centrelines [x0, y0, x1, y1]. */
export const SIDE_STREETS: [number, number, number, number][] = [
  [-BLOCK, -BLOCK, BLOCK, -BLOCK], // south street
  [BLOCK, -ROAD_HALF, BLOCK, -BLOCK], // east street
  [-BLOCK, -ROAD_HALF, -BLOCK, -BLOCK], // west street
];

/** Does the rectangle [x0, x1] × [y0, y1] come within `margin` of a side street or its sidewalks? */
export function nearSideStreet(x0: number, y0: number, x1: number, y1: number, margin = 0) {
  const m = SIDE_WALK + margin;
  return SIDE_STREETS.some(([ax, ay, bx, by]) =>
    x1 > Math.min(ax, bx) - m && x0 < Math.max(ax, bx) + m && y1 > Math.min(ay, by) - m && y0 < Math.max(ay, by) + m);
}

const HN = Math.PI / 2, HS = -Math.PI / 2, HE = 0, HW = Math.PI;

/** Side-street approaches with a stop sign: junction and travel heading. */
export const STOP_APPROACHES: { j: JunctionId; h: number }[] = [
  { j: 'E', h: HN }, { j: 'W', h: HN }, { j: 'S', h: HW }, { j: 'S', h: HE },
];

/** Stop-sign posts: on the driver's left, just before the stop line; `face` is the direction the sign faces. */
export function stopSigns(): { x: number; y: number; face: number }[] {
  return STOP_APPROACHES.map(({ j, h }) => {
    const [cx, cy] = JUNCTION_POS[j];
    const [dx, dy] = hd(h), [nx, ny] = leftN(h);
    const t = -SIDE_STOP - 0.6, o = SIDE_HALF + 0.5;
    return { x: cx + dx * t + nx * o, y: cy + dy * t + ny * o, face: h + Math.PI };
  });
}

export interface LegSpec {
  j: JunctionId;
  h1: number; // approach heading
  turn: 'left' | 'right';
  o1: number; o2: number; // entry / exit lane offsets
  R: number;
  control: Control;
  from?: number; // lane the car arrives in, when it has to change lanes before this turn
  hug?: boolean; // left turn from a two-lane road: room on the left for a motorcycle unless the driver pulls over
}

/** One lap of the figure of eight, one leg per turn. */
export const LAP: LegSpec[] = [
  { j: 'C', h1: HN, turn: 'right', o1: INNER, o2: INNER, R: TURN_RADIUS.right, control: 'signal', from: OUTER },
  { j: 'E', h1: HE, turn: 'right', o1: INNER, o2: SIDE_LANE, R: 7, control: 'yield' },
  { j: 'SE', h1: HS, turn: 'right', o1: SIDE_LANE, o2: SIDE_LANE, R: 6, control: 'none' },
  { j: 'S', h1: HW, turn: 'right', o1: SIDE_LANE, o2: INNER, R: 7, control: 'stop' },
  { j: 'C', h1: HN, turn: 'left', o1: OUTER, o2: OUTER, R: TURN_RADIUS.left, control: 'signal', from: INNER, hug: true },
  { j: 'W', h1: HW, turn: 'left', o1: OUTER, o2: SIDE_LANE, R: 5.5, control: 'yield', hug: true },
  { j: 'SW', h1: HS, turn: 'left', o1: SIDE_LANE, o2: SIDE_LANE, R: 5, control: 'none' },
  { j: 'S', h1: HE, turn: 'left', o1: SIDE_LANE, o2: OUTER, R: 5.5, control: 'stop' },
];
/** A leg starts this far before its junction (just past the previous one) and runs this far past it. */
export const LEG_START = 77;
export const LEG_END = 40;
/** Lane change on the south arm: length, and the window for its start (the yellow no-change line begins at -44). */
export const LC_LEN = 20;
export const LC_EARLIEST = -74;
export const LC_LATEST = -64;

/**
 * Route of one leg. lcT0: where the lane change starts (null: already in the right lane).
 * hugT0: where pulling over to the kerb before a left turn starts (null: not pulling over).
 */
export function buildLeg(i: number, lcT0: number | null, startDist = LEG_START, hugT0: number | null = null): Path {
  const L = LAP[i];
  const [cx, cy] = JUNCTION_POS[L.j];
  const lc = lcT0 !== null && L.from !== undefined ? { from: L.from, t0: lcT0, len: LC_LEN } : undefined;
  const h = hugT0 !== null && L.hug ? { d: HUG, t0: hugT0, len: HUG_LEN } : undefined;
  const p = buildTurnPath({ cx, cy, h1: L.h1, turn: L.turn, o1: L.o1, o2: L.o2, R: L.R, startDist, endDist: LEG_END, lc, hug: h });
  if (L.j === 'C') {
    markCentre(p, L.h1, L.turn, L.R);
    // Left turn: last look with the nose at the stop line (not past it, so a red still holds the car).
    if (L.turn === 'left') p.waitS = p.stopS - 2.6;
  } else if (L.hug) {
    p.control = L.control;
    p.waitS = p.arcS0 - 1.5; // last look along the left side before turning in
    p.exitS = p.arcS1 + 3;
  } else {
    p.control = L.control;
    const [dx, dy] = hd(L.h1);
    if (L.control === 'stop') {
      p.stopS = p.sWhere(dx, dy, -SIDE_STOP, cx, cy);
      p.waitS = p.stopS - 2.6; // car centre when its nose is at the stop line
    } else if (L.control === 'yield') {
      p.waitS = p.arcS0 + 0.08 * (L.R * Math.PI) / 2;
    }
    p.exitS = p.arcS1 + 3;
  }
  p.label = `lap${i}:${L.j}-${L.turn}`;
  return p;
}

/** Kerb segments [x0, y0, x1, y1] along the main arms and side streets, open at the side-street mouths. */
export function kerbSegments(): [number, number, number, number][] {
  const out: [number, number, number, number][] = [];
  const cut = (a: number, b: number, gaps: number[], w: number) => {
    let segs: [number, number][] = [[a, b]];
    for (const g of gaps) {
      segs = segs.flatMap(([u, v]): [number, number][] =>
        g + w <= u || g - w >= v ? [[u, v]] : [[u, g - w], [g + w, v]]);
    }
    return segs.filter(([u, v]) => v - u > 0.5);
  };
  const alongY = (x: number, y0: number, y1: number, gaps: number[] = [], w = 0) => {
    for (const [a, b] of cut(y0, y1, gaps, w)) out.push([x, a, x, b]);
  };
  const alongX = (y: number, x0: number, x1: number, gaps: number[] = [], w = 0) => {
    for (const [a, b] of cut(x0, x1, gaps, w)) out.push([a, y, b, y]);
  };
  const k = ROAD_HALF + 0.1, sk = SIDE_HALF + 0.1;
  const mouth = SIDE_HALF + CORNER_R, cross = ROAD_HALF + CORNER_R;
  const B = BLOCK;
  for (const s of [1, -1]) alongY(s * k, WALK_OUT, ARM); // north arm
  for (const s of [1, -1]) alongY(s * k, -ARM, -WALK_OUT, [-B], mouth); // south arm
  alongX(k, WALK_OUT, ARM); alongX(-k, WALK_OUT, ARM, [B], mouth); // east arm
  alongX(k, -ARM, -WALK_OUT); alongX(-k, -ARM, -WALK_OUT, [-B], mouth); // west arm
  for (const s of [1, -1]) {
    alongY(s * (B - sk), -B + mouth, -cross); // inner kerbs of the east / west streets
    alongY(s * (B + sk), -B - sk, -cross); // outer kerbs
  }
  alongX(-B + sk, -B + mouth, B - mouth, [0], cross); // south street, block side
  alongX(-B - sk, -B - sk, B + sk, [0], cross); // south street, outer side
  return out;
}

// ── Scenery (deterministic pseudo-random) ─────────────────

export function rng(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export interface Building { x: number; y: number; hx: number; hy: number; h: number; color: number; kind: 'office' | 'shop' | 'house' }

/** Blocks of buildings lining the four arms, lower near the corners, taller further out. */
export function buildings(): Building[] {
  const r = rng(55);
  const out: Building[] = [];
  const kinds: Building['kind'][] = ['office', 'shop', 'house'];
  for (const q of [[1, 1], [1, -1], [-1, 1], [-1, -1]] as const) {
    // Two rows along each road facing into the quadrant
    for (const alongX of [true, false]) {
      let t = BUILDING_LINE + 1;
      while (t < 120) {
        const w = 8 + r() * 14;
        const depth = 8 + r() * 10;
        const gap = 1 + r() * 4;
        const far = t > 50;
        const h = far ? 8 + r() * 30 : 5 + r() * 16;
        const kind = kinds[Math.floor(r() * 3)];
        const c = t + w / 2, off = BUILDING_LINE + depth / 2 + r() * 1.5;
        const [x, y] = alongX ? [q[0] * c, q[1] * off] : [q[0] * off, q[1] * c];
        const [hx, hy] = alongX ? [w / 2, depth / 2] : [depth / 2, w / 2];
        // Skip the lot right at the corner (both rows would overlap): keep one row there.
        if (!alongX && t < BUILDING_LINE + 20) { t += w + gap; continue; }
        const color = Math.floor(r() * 6);
        if (!nearSideStreet(x - hx, y - hy, x + hx, y + hy, 1)) out.push({ x, y, hx, hy, h, color, kind });
        t += w + gap;
      }
    }
  }
  // Lower rows along the side streets, leaving the corners open so drivers can see along the main road.
  const r2 = rng(56);
  const overlaps = (b: Building) => out.some((o) => Math.abs(o.x - b.x) < o.hx + b.hx + 1 && Math.abs(o.y - b.y) < o.hy + b.hy + 1);
  const onMainRoad = (b: Building) => Math.abs(b.x) < b.hx + BUILDING_LINE || Math.abs(b.y) < b.hy + BUILDING_LINE;
  const nearJunction = (b: Building) =>
    Object.values(JUNCTION_POS).some(([jx, jy]) => Math.abs(b.x - jx) < b.hx + 12 && Math.abs(b.y - jy) < b.hy + 12);
  for (const [ax, ay, bx, by] of SIDE_STREETS) {
    const alongX = ay === by;
    const lo = alongX ? Math.min(ax, bx) : Math.min(ay, by), hi = alongX ? Math.max(ax, bx) : Math.max(ay, by);
    for (const side of [1, -1]) {
      for (let t = lo - 30; t < hi + 30;) {
        const w = 7 + r2() * 10, depth = 7 + r2() * 8, gap = 1 + r2() * 3, h = 4 + r2() * 12;
        const off = SIDE_WALK + 1 + depth / 2 + r2();
        const c = t + w / 2;
        const [x, y] = alongX ? [c, ay + side * off] : [ax + side * off, c];
        const [hx, hy] = alongX ? [w / 2, depth / 2] : [depth / 2, w / 2];
        const b: Building = { x, y, hx, hy, h, color: Math.floor(r2() * 6), kind: r2() < 0.75 ? 'house' : 'shop' };
        t += w + gap;
        if (nearSideStreet(x - hx, y - hy, x + hx, y + hy, 0.5) || onMainRoad(b) || nearJunction(b) || overlaps(b)) continue;
        out.push(b);
      }
    }
  }
  return out;
}

/** Street tree positions on the sidewalks (away from the corners). */
export function streetTrees(): [number, number][] {
  const out: [number, number][] = [];
  const off = ROAD_HALF + 0.45; // kerb side, clear of the walking lines
  for (let t = 24; t < 110; t += 11) {
    for (const s of [1, -1]) {
      out.push([s * off, t], [s * off, -t], [t, s * off], [-t, s * off]);
    }
  }
  return out.filter(([x, y]) => !nearSideStreet(x, y, x, y, 2));
}

/** Utility poles (Japanese streets): along the outer sidewalk edge of each arm. */
export function utilityPoles(): [number, number][] {
  const out: [number, number][] = [];
  const off = WALK_OUT - 0.4;
  for (let t = 17; t < 140; t += 30) {
    out.push([off, t + 5], [-off, -t], [t, -off], [-t - 5, off]);
  }
  return out.filter(([x, y]) => !nearSideStreet(x, y, x, y, 1));
}

/** Signal mast positions: one at each corner, arm reaching over the lanes it controls. */
export interface SignalMast { x: number; y: number; face: Arm; armDir: [number, number] }
export function signalMasts(): SignalMast[] {
  const c = WALK_OUT - 0.3;
  // The mast for traffic arriving from S stands on the far-left (NW) corner and faces south.
  return [
    { x: -c, y: c, face: 'S', armDir: [1, 0] },
    { x: c, y: c, face: 'W', armDir: [0, -1] },
    { x: c, y: -c, face: 'N', armDir: [-1, 0] },
    { x: -c, y: -c, face: 'E', armDir: [0, 1] },
  ];
}
