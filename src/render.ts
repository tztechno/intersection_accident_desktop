// Draws the MuJoCo scene with three.js, plus the visual-only parts of the intersection:
// road surface and markings, buildings, tree crowns, wires, signal heads, sight lines.
// Everything stays in MuJoCo's Z-up world frame; cameras use up = +Z.
import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import type { MainModule, MjData } from '@mujoco/mujoco';
import type { IntersectionSim, Sight } from './sim.ts';
import {
  ARM, ARM_HEADING, BLOCK, CORNER_R, INNER, OUTER, ROAD_HALF, SIDE_HALF, SIDE_STOP, SIDE_WALK, STOP_LINE,
  WALK_OUT, buildRoute, buildings, nearSideStreet, rng, signalMasts, stopSigns, streetTrees, utilityPoles,
  type Arm, type SignalState,
} from './layout.ts';

THREE.ColorManagement.enabled = false;

const GEOM_PLANE = 0;
const GEOM_SPHERE = 2;
const GEOM_CAPSULE = 3;
const GEOM_CYLINDER = 5;
const GEOM_BOX = 6;
const SKY = 0xbcd6ee;
export const OVERLAY_LAYER = 1;

// ── Small helpers ─────────────────────────────────────────

function canvasTexture(w: number, h: number, draw: (g: CanvasRenderingContext2D) => void, repeat = true) {
  const c = document.createElement('canvas');
  c.width = w;
  c.height = h;
  draw(c.getContext('2d')!);
  const t = new THREE.CanvasTexture(c);
  if (repeat) t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 8;
  return t;
}

function noise(g: CanvasRenderingContext2D, w: number, h: number, amp: number, seed = 1) {
  const r = rng(seed);
  const img = g.getImageData(0, 0, w, h);
  for (let i = 0; i < img.data.length; i += 4) {
    const n = (r() - 0.5) * amp;
    img.data[i] += n; img.data[i + 1] += n; img.data[i + 2] += n;
  }
  g.putImageData(img, 0, 0);
}

function colorize(geo: THREE.BufferGeometry, c: THREE.Color) {
  const n = geo.getAttribute('position').count;
  const col = new Float32Array(3 * n);
  for (let i = 0; i < n; i++) { col[3 * i] = c.r; col[3 * i + 1] = c.g; col[3 * i + 2] = c.b; }
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  return geo;
}

// ── Scene ─────────────────────────────────────────────────

export class SceneView {
  readonly scene = new THREE.Scene();
  readonly sun: THREE.DirectionalLight;
  private bodyMeshes: { mesh: THREE.Mesh; body: number; agent: number }[] = [];
  private sim: IntersectionSim;
  private lamps: { mesh: THREE.Mesh; on: THREE.Material; off: THREE.Material; state: (s: SignalState, t: number) => boolean }[] = [];
  private sightLines: THREE.LineSegments;
  private sightGeom: THREE.BufferGeometry;
  private rings: { mesh: THREE.Mesh; t0: number }[] = [];
  private tmp = new THREE.Matrix4();

  constructor(mj: MainModule, sim: IntersectionSim) {
    this.sim = sim;
    const model = sim.model;
    this.scene.background = new THREE.Color(SKY);
    this.scene.fog = new THREE.Fog(SKY, 250, 900);
    this.scene.add(new THREE.HemisphereLight(0xe4efff, 0x8a8070, 0.55 * Math.PI));
    this.sun = new THREE.DirectionalLight(0xfff4e0, 0.75 * Math.PI);
    this.sun.position.set(-60, -45, 110);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = -70; sc.right = 70; sc.top = 70; sc.bottom = -70; sc.near = 1; sc.far = 400;
    this.sun.shadow.bias = -0.0005;
    this.scene.add(this.sun, this.sun.target);

    // MuJoCo geoms: static ones merged into one mesh, each moving body into its own mesh.
    const staticParts: THREE.BufferGeometry[] = [];
    const perBody = new Map<number, THREE.BufferGeometry[]>();
    const GEOM = mj.mjtObj.mjOBJ_GEOM.value;
    const size = model.geom_size, rgba = model.geom_rgba, type = model.geom_type, bodyOf = model.geom_bodyid;
    for (let i = 0; i < model.ngeom; i++) {
      const a = rgba[4 * i + 3];
      if (a === 0 || type[i] === GEOM_PLANE) continue;
      const name = mj.mj_id2name(model, GEOM, i);
      if (name.startsWith('bld_')) continue; // drawn with facades below
      const s = [size[3 * i], size[3 * i + 1], size[3 * i + 2]];
      let geo: THREE.BufferGeometry;
      switch (type[i]) {
        case GEOM_BOX: geo = new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]); break;
        case GEOM_CYLINDER: geo = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 20).rotateX(Math.PI / 2); break;
        case GEOM_CAPSULE: geo = new THREE.CapsuleGeometry(s[0], 2 * s[1], 4, 10).rotateX(Math.PI / 2); break;
        case GEOM_SPHERE: geo = new THREE.SphereGeometry(s[0], 16, 12); break;
        default: continue;
      }
      geo = geo.toNonIndexed();
      geo.deleteAttribute('uv');
      colorize(geo, new THREE.Color(rgba[4 * i], rgba[4 * i + 1], rgba[4 * i + 2]));
      const gp = model.geom_pos, gq = model.geom_quat;
      const local = new THREE.Matrix4().compose(
        new THREE.Vector3(gp[3 * i], gp[3 * i + 1], gp[3 * i + 2]),
        new THREE.Quaternion(gq[4 * i + 1], gq[4 * i + 2], gq[4 * i + 3], gq[4 * i]),
        new THREE.Vector3(1, 1, 1),
      );
      geo.applyMatrix4(local);
      const b = bodyOf[i];
      if (b === 0) staticParts.push(geo);
      else (perBody.get(b) ?? perBody.set(b, []).get(b)!).push(geo);
    }
    const bodyMat = new THREE.MeshPhongMaterial({ vertexColors: true, shininess: 60, specular: 0x333333 });
    for (const [b, parts] of perBody) {
      const mesh = new THREE.Mesh(mergeGeometries(parts), bodyMat);
      mesh.matrixAutoUpdate = false;
      mesh.castShadow = true;
      const agent = sim.geomAgent[model.body_geomadr[b]] ?? -1;
      this.scene.add(mesh);
      this.bodyMeshes.push({ mesh, body: b, agent });
    }
    if (staticParts.length) {
      const st = new THREE.Mesh(mergeGeometries(staticParts), new THREE.MeshLambertMaterial({ vertexColors: true }));
      st.castShadow = true;
      st.receiveShadow = true;
      this.scene.add(st);
    }

    this.scene.add(groundMeshes());
    this.scene.add(buildingMeshes());
    this.scene.add(treeCrowns());
    this.scene.add(wires());
    this.scene.add(streetLights());
    this.scene.add(stopSignMeshes());
    this.scene.add(mountains());
    this.scene.add(this.signalHeads());

    this.sightGeom = new THREE.BufferGeometry();
    this.sightGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6 * 64), 3));
    this.sightGeom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(6 * 64), 3));
    this.sightLines = new THREE.LineSegments(this.sightGeom, new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, transparent: true, opacity: 0.9 }));
    this.sightLines.layers.set(OVERLAY_LAYER);
    this.sightLines.renderOrder = 10;
    this.sightLines.frustumCulled = false;
    this.scene.add(this.sightLines);
    this.sync();
  }

  /**
   * Copy body poses from MjData into the meshes; hide parked agents. By default the live simulation;
   * a replay passes its own data and active flags.
   */
  sync(d: MjData = this.sim.data, active?: Uint8Array) {
    const xp = d.xpos, xm = d.xmat;
    for (const { mesh, body: b, agent } of this.bodyMeshes) {
      const vis = agent < 0 || (active ? active[agent] === 1 : this.sim.isActive(agent));
      mesh.visible = vis;
      if (!vis) continue;
      const r = 9 * b, p = 3 * b;
      mesh.matrix.set(
        xm[r], xm[r + 1], xm[r + 2], xp[p],
        xm[r + 3], xm[r + 4], xm[r + 5], xp[p + 1],
        xm[r + 6], xm[r + 7], xm[r + 8], xp[p + 2],
        0, 0, 0, 1,
      );
      mesh.matrixWorldNeedsUpdate = true;
    }
  }

  /** Keep the sun's shadow box around what the camera looks at. */
  followShadow(x: number, y: number) {
    this.sun.position.set(x - 60, y - 45, 110);
    this.sun.target.position.set(x, y, 0);
    this.sun.target.updateMatrixWorld();
  }

  updateSignals(s: SignalState, t: number) {
    for (const l of this.lamps) {
      const on = l.state(s, t);
      l.mesh.material = on ? l.on : l.off;
    }
  }

  /** Sight lines from the ego driver's eyes: green = seen, red = hidden. */
  updateSight(eye: [number, number, number], sight: Sight[]) {
    const pos = this.sightGeom.getAttribute('position') as THREE.BufferAttribute;
    const col = this.sightGeom.getAttribute('color') as THREE.BufferAttribute;
    const n = Math.min(sight.length, 64);
    for (let i = 0; i < n; i++) {
      const s = sight[i];
      pos.setXYZ(2 * i, eye[0], eye[1], eye[2]);
      pos.setXYZ(2 * i + 1, s.x, s.y, 1.0);
      const c = s.visible ? [0.2, 1, 0.3] : [1, 0.15, 0.1];
      col.setXYZ(2 * i, c[0], c[1], c[2]);
      col.setXYZ(2 * i + 1, c[0], c[1], c[2]);
    }
    this.sightGeom.setDrawRange(0, 2 * n);
    pos.needsUpdate = true;
    col.needsUpdate = true;
  }

  /** Expanding ring where a crash happened. */
  addCrashMarker(x: number, y: number, t: number) {
    const mesh = new THREE.Mesh(
      new THREE.RingGeometry(0.8, 1.1, 40),
      new THREE.MeshBasicMaterial({ color: 0xff3020, transparent: true, opacity: 0.9, depthWrite: false, side: THREE.DoubleSide }),
    );
    mesh.position.set(x, y, 0.15);
    mesh.renderOrder = 5;
    mesh.layers.enable(OVERLAY_LAYER);
    this.scene.add(mesh);
    this.rings.push({ mesh, t0: t });
  }

  updateMarkers(t: number) {
    for (const r of [...this.rings]) {
      const age = t - r.t0;
      if (age > 4 || age < 0) {
        this.scene.remove(r.mesh);
        r.mesh.geometry.dispose();
        this.rings.splice(this.rings.indexOf(r), 1);
        continue;
      }
      const s = 1 + age * 2.5;
      r.mesh.scale.set(s, s, 1);
      (r.mesh.material as THREE.MeshBasicMaterial).opacity = 0.9 * (1 - age / 4);
    }
  }

  clearMarkers() {
    for (const r of this.rings) this.scene.remove(r.mesh);
    this.rings.length = 0;
  }

  /** Place a three.js camera at a MuJoCo model camera. */
  setFromModelCamera(cam: THREE.PerspectiveCamera, camId: number, d: MjData = this.sim.data) {
    const p = d.cam_xpos, r = d.cam_xmat;
    const i = 9 * camId, j = 3 * camId;
    this.tmp.set(
      r[i], r[i + 1], r[i + 2], p[j],
      r[i + 3], r[i + 4], r[i + 5], p[j + 1],
      r[i + 6], r[i + 7], r[i + 8], p[j + 2],
      0, 0, 0, 1,
    );
    this.tmp.decompose(cam.position, cam.quaternion, cam.scale);
    cam.up.set(0, 0, 1);
    cam.fov = this.sim.model.cam_fovy[camId];
    cam.updateProjectionMatrix();
    cam.updateMatrixWorld();
  }

  // ── Signal heads ────────────────────────────────────────

  private signalHeads() {
    const group = new THREE.Group();
    const housing = new THREE.MeshLambertMaterial({ color: 0x3a3d40 });
    const lampGeo = new THREE.CircleGeometry(0.15, 24);
    const mk = (color: number) => new THREE.MeshBasicMaterial({ color });
    const dim = (color: number) => new THREE.MeshLambertMaterial({ color: new THREE.Color(color).multiplyScalar(0.18) });
    const G = 0x30ffb0, Y = 0xffc020, R = 0xff2a20;
    const arrowTex = (on: boolean) => canvasTexture(64, 64, (g) => {
      g.fillStyle = '#111'; g.fillRect(0, 0, 64, 64);
      g.fillStyle = on ? '#3dffae' : '#16221c';
      g.beginPath(); g.moveTo(14, 26); g.lineTo(36, 26); g.lineTo(36, 14); g.lineTo(54, 32);
      g.lineTo(36, 50); g.lineTo(36, 38); g.lineTo(14, 38); g.closePath(); g.fill();
    }, false);
    const arrowOn = new THREE.MeshBasicMaterial({ map: arrowTex(true) });
    const arrowOff = new THREE.MeshBasicMaterial({ map: arrowTex(false) });

    for (const m of signalMasts()) {
      const h = ARM_HEADING[m.face];
      const fx = Math.cos(h), fy = Math.sin(h); // approaching traffic travels along f; the head faces -f
      const axis = m.face === 'N' || m.face === 'S' ? 'NS' : 'EW';
      const dist = Math.abs(m.armDir[0] ? m.x : m.y) - 3.5;
      const hx = m.x + m.armDir[0] * dist, hy = m.y + m.armDir[1] * dist;
      const head = new THREE.Group();
      head.position.set(hx, hy, 5.6);
      head.rotation.z = Math.atan2(-fy, -fx); // local +x faces the drivers
      // Horizontal Japanese head: green, yellow, red from the driver's left to right.
      const box = new THREE.Mesh(new THREE.BoxGeometry(0.3, 1.25, 0.42), housing);
      box.castShadow = true;
      head.add(box);
      // Sun hood over the lamps (a roof sticking out, not a plate in front of them)
      const visor = new THREE.Mesh(new THREE.BoxGeometry(0.22, 1.3, 0.03), housing);
      visor.position.set(0.25, 0, 0.22);
      head.add(visor);
      const cols: [number, (s: SignalState) => boolean][] = [
        [G, (s) => (axis === 'NS' ? s.ns : s.ew) === 'G'],
        [Y, (s) => (axis === 'NS' ? s.ns : s.ew) === 'Y'],
        [R, (s) => (axis === 'NS' ? s.ns : s.ew) === 'R'],
      ];
      cols.forEach(([c, st], i) => {
        const lamp = new THREE.Mesh(lampGeo, mk(c));
        lamp.position.set(0.17, -0.4 + 0.4 * i, 0); // the driver's left is local -y; just proud of the housing
        lamp.rotation.y = Math.PI / 2;
        head.add(lamp);
        this.lamps.push({ mesh: lamp, on: mk(c), off: dim(c), state: st });
      });
      const arrowBox = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.4, 0.36), housing);
      arrowBox.position.set(0, 0.4, -0.42); // right-turn arrow under the red lamp
      head.add(arrowBox);
      const arrow = new THREE.Mesh(new THREE.PlaneGeometry(0.3, 0.3), arrowOff);
      arrow.position.set(0.15, 0.4, -0.42);
      arrow.rotation.set(Math.PI / 2, Math.PI / 2, 0);
      head.add(arrow);
      this.lamps.push({ mesh: arrow, on: arrowOn, off: arrowOff, state: (s) => (axis === 'NS' ? s.nsArrow : s.ewArrow) });
      group.add(head);

      // Pedestrian heads on the pole: one facing across each crosswalk from this corner.
      const sx = Math.sign(m.x), sy = Math.sign(m.y);
      for (const [nx, ny, walkAxis] of [[0, -sy, 'NS'], [-sx, 0, 'EW']] as const) {
        const ph = new THREE.Group();
        ph.position.set(m.x + nx * 0.3, m.y + ny * 0.3, 2.7);
        ph.rotation.z = Math.atan2(ny, nx);
        const pb = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.42, 0.78), housing);
        ph.add(pb);
        for (const [z, color, isGreen] of [[0.18, R, false], [-0.18, G, true]] as const) {
          const tex = (on: boolean) => new THREE.MeshBasicMaterial({ map: pedIcon(isGreen, on) });
          const lamp = new THREE.Mesh(new THREE.PlaneGeometry(0.34, 0.34), tex(false));
          lamp.position.set(0.115, 0, z);
          lamp.rotation.set(Math.PI / 2, Math.PI / 2, 0);
          ph.add(lamp);
          void color;
          this.lamps.push({
            mesh: lamp, on: tex(true), off: tex(false),
            state: (s, t) => {
              const w = walkAxis === 'NS' ? s.walkNS : s.walkEW;
              return isGreen ? w === 'G' || (w === 'F' && Math.floor(t * 2) % 2 === 0) : w === 'R';
            },
          });
        }
        group.add(ph);
      }
    }
    // Intersection name plate on the north-west mast
    const plate = new THREE.Mesh(new THREE.PlaneGeometry(2.6, 0.7), new THREE.MeshBasicMaterial({
      map: canvasTexture(512, 138, (g) => {
        g.fillStyle = '#1d4fa0'; g.fillRect(0, 0, 512, 138);
        g.strokeStyle = '#fff'; g.lineWidth = 6; g.strokeRect(6, 6, 500, 126);
        g.fillStyle = '#fff'; g.textAlign = 'center';
        g.font = 'bold 60px sans-serif'; g.fillText('中央一丁目', 256, 72);
        g.font = 'bold 30px sans-serif'; g.fillText('Chuo 1-chome', 256, 115);
      }, false),
      side: THREE.DoubleSide,
    }));
    const nw = signalMasts()[0];
    plate.position.set(nw.x + 5.5, nw.y, 6.35);
    plate.rotation.set(Math.PI / 2, 0, 0);
    group.add(plate);
    return group;
  }
}

function pedIcon(green: boolean, on: boolean) {
  return canvasTexture(64, 64, (g) => {
    g.fillStyle = '#0c0c0c'; g.fillRect(0, 0, 64, 64);
    g.fillStyle = on ? (green ? '#3dffae' : '#ff3322') : (green ? '#16301f' : '#301410');
    g.beginPath(); g.arc(32, 13, 7, 0, Math.PI * 2); g.fill();
    g.lineWidth = 8; g.lineCap = 'round'; g.strokeStyle = g.fillStyle;
    g.beginPath();
    if (green) { // walking
      g.moveTo(32, 24); g.lineTo(28, 40); g.lineTo(20, 56); g.moveTo(28, 40); g.lineTo(40, 55);
      g.moveTo(31, 28); g.lineTo(42, 36); g.moveTo(31, 28); g.lineTo(22, 36);
    } else { // standing
      g.moveTo(32, 24); g.lineTo(32, 42); g.lineTo(27, 58); g.moveTo(32, 42); g.lineTo(37, 58);
      g.moveTo(32, 27); g.lineTo(23, 42); g.moveTo(32, 27); g.lineTo(41, 42);
    }
    g.stroke();
  }, false);
}

// ── Ground, road surface and markings ─────────────────────

const INNER_R = 44; // detailed texture covers [-INNER_R, INNER_R]²
const PX = 24; // texture pixels per metre

function groundMeshes() {
  const group = new THREE.Group();
  // Far ground: grass / lots, tinted
  const grass = canvasTexture(256, 256, (g) => {
    g.fillStyle = '#7d8a64'; g.fillRect(0, 0, 256, 256);
    noise(g, 256, 256, 26, 3);
  });
  grass.repeat.set(200, 200);
  const far = new THREE.Mesh(new THREE.PlaneGeometry(2400, 2400), new THREE.MeshLambertMaterial({ map: grass }));
  far.position.z = -0.03;
  far.receiveShadow = true;
  group.add(far);

  // Paved lots behind the sidewalks along the arms
  const lot = new THREE.MeshLambertMaterial({ color: 0x8e8c86 });
  const lotGeo: THREE.BufferGeometry[] = [];
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const g = new THREE.PlaneGeometry(ARM + 20 - WALK_OUT, ARM + 20 - WALK_OUT);
    g.translate(sx * (WALK_OUT + (ARM + 20 - WALK_OUT) / 2), sy * (WALK_OUT + (ARM + 20 - WALK_OUT) / 2), -0.02);
    lotGeo.push(g);
  }
  const lots = new THREE.Mesh(mergeGeometries(lotGeo), lot);
  lots.receiveShadow = true;
  group.add(lots);

  // Detailed centre square
  const N = INNER_R * 2 * PX;
  const tex = canvasTexture(N, N, (g) => drawCentre(g, N), false);
  tex.generateMipmaps = true;
  const centre = new THREE.Mesh(new THREE.PlaneGeometry(2 * INNER_R, 2 * INNER_R), new THREE.MeshLambertMaterial({ map: tex }));
  centre.position.z = 0.005;
  centre.receiveShadow = true;
  group.add(centre);

  // Arms beyond the centre square: a repeating cross-section
  const W = 2 * WALK_OUT;
  const cross = canvasTexture(W * 20, 200, (g) => drawArmSection(g, W * 20, 200));
  const armMat = new THREE.MeshLambertMaterial({ map: cross });
  const armLen = ARM + 10 - INNER_R;
  for (const [arm, rot] of [['N', 0], ['S', Math.PI], ['E', -Math.PI / 2], ['W', Math.PI / 2]] as const) {
    const geo = new THREE.PlaneGeometry(W, armLen);
    const uv = geo.getAttribute('uv') as THREE.BufferAttribute;
    for (let i = 0; i < uv.count; i++) uv.setY(i, uv.getY(i) * armLen / 10);
    geo.translate(0, INNER_R + armLen / 2, 0.005);
    geo.rotateZ(rot);
    const m = new THREE.Mesh(geo, armMat);
    m.receiveShadow = true;
    void arm;
    group.add(m);
  }

  // Side streets of the town loop: straight stretches, then the junctions drawn in detail on top.
  const sideTex = canvasTexture(SIDE_WALK * 2 * 24, 240, (g) => drawSideSection(g, SIDE_WALK * 2 * 24, 240));
  const sideMat = new THREE.MeshLambertMaterial({ map: sideTex });
  const B = BLOCK, J = JUNCTION_HALF;
  const strip = (x0: number, y0: number, x1: number, y1: number) => {
    const alongX = y0 === y1;
    const len = Math.abs(alongX ? x1 - x0 : y1 - y0);
    const geo = new THREE.PlaneGeometry(2 * SIDE_WALK, len);
    const uv = geo.getAttribute('uv') as THREE.BufferAttribute;
    for (let i = 0; i < uv.count; i++) uv.setY(i, uv.getY(i) * len / 10);
    if (alongX) geo.rotateZ(Math.PI / 2);
    geo.translate((x0 + x1) / 2, (y0 + y1) / 2, 0.005);
    const m = new THREE.Mesh(geo, sideMat);
    m.receiveShadow = true;
    group.add(m);
  };
  strip(-B + J, -B, -J, -B); strip(J, -B, B - J, -B);
  for (const s of [1, -1]) strip(s * B, -B + J, s * B, -TEE_Y1);
  group.add(patch(0, -B, J, J, (p) => drawCrossJunction(p)));
  for (const s of [1, -1] as const) {
    group.add(patch(s * B, -(TEE_Y0 + TEE_Y1) / 2, J, (TEE_Y1 - TEE_Y0) / 2, (p) => drawTeeMouth(p, s * B)));
    group.add(patch(s * B, -B, J, J, (p) => drawBend(p, s * B)));
  }
  return group;
}

// ── Town loop junctions ───────────────────────────────────

const JUNCTION_HALF = 14; // detailed patches cover this far either side of a junction
const TEE_Y0 = ROAD_HALF - 0.6, TEE_Y1 = WALK_OUT + 2; // patch over the main-road kerb where a side street joins

interface Painter {
  g: CanvasRenderingContext2D;
  P: (x: number) => number; // world x → canvas x
  Q: (y: number) => number; // world y → canvas y
  rect: (x0: number, y0: number, x1: number, y1: number) => void;
  disc: (x: number, y: number, r: number) => void;
  x0: number; y0: number; x1: number; y1: number; // world extent
}

/** A detailed piece of road surface, painted in world coordinates. */
function patch(cx: number, cy: number, hw: number, hh: number, draw: (p: Painter) => void) {
  const W = Math.round(2 * hw * PX), H = Math.round(2 * hh * PX);
  const tex = canvasTexture(W, H, (g) => {
    const P = (x: number) => (x - (cx - hw)) * PX, Q = (y: number) => (cy + hh - y) * PX;
    draw({
      g, P, Q,
      rect: (x0, y0, x1, y1) => g.fillRect(P(Math.min(x0, x1)), Q(Math.max(y0, y1)), Math.abs(x1 - x0) * PX, Math.abs(y1 - y0) * PX),
      disc: (x, y, r) => { g.beginPath(); g.arc(P(x), Q(y), r * PX, 0, Math.PI * 2); g.fill(); },
      x0: cx - hw, y0: cy - hh, x1: cx + hw, y1: cy + hh,
    });
    noise(g, W, H, 12, 21);
  }, false);
  tex.generateMipmaps = true;
  const m = new THREE.Mesh(new THREE.PlaneGeometry(2 * hw, 2 * hh), new THREE.MeshLambertMaterial({ map: tex }));
  m.position.set(cx, cy, 0.008);
  m.receiveShadow = true;
  return m;
}

/** Sidewalk tiles over the whole patch, then paved lot outside the sidewalk bands. */
function paintGround(p: Painter, walks: [number, number, number, number][]) {
  const { g } = p;
  g.fillStyle = '#8e8c86'; p.rect(p.x0, p.y0, p.x1, p.y1);
  g.fillStyle = SIDEWALK;
  for (const w of walks) p.rect(...w);
  // Tiles only on the sidewalks
  g.save();
  g.beginPath();
  for (const [x0, y0, x1, y1] of walks) {
    g.rect(p.P(Math.min(x0, x1)), p.Q(Math.max(y0, y1)), Math.abs(x1 - x0) * PX, Math.abs(y1 - y0) * PX);
  }
  g.clip();
  sidewalkTiles(g, 0, 0, g.canvas.width, g.canvas.height, PX * 0.5);
  g.restore();
}

/** Rounded kerb at an inside corner (cx, cy), with the sidewalk towards (sx, sy). */
function roundCorner(p: Painter, cx: number, cy: number, sx: number, sy: number) {
  p.g.fillStyle = ASPHALT;
  p.rect(cx, cy, cx + sx * CORNER_R, cy + sy * CORNER_R);
  p.g.fillStyle = SIDEWALK;
  p.disc(cx + sx * CORNER_R, cy + sy * CORNER_R, CORNER_R);
}

/**
 * Lane divider along an arm, between `from` and `to` (world coordinate along the arm), in phase with the
 * arm texture: dashes where the distance d from the centre has (d − 4) mod 10 ≥ 5, i.e. d ∈ [9, 14) + 10k.
 */
function dashes(p: Painter, alongX: boolean, c: number, from: number, to: number, wid: number) {
  const lo = Math.min(from, to), hi = Math.max(from, to);
  for (let d0 = 9; d0 < ARM + 20; d0 += 10) {
    for (const sgn of [1, -1]) {
      const u0 = Math.max(lo, Math.min(sgn * d0, sgn * (d0 + 5))), u1 = Math.min(hi, Math.max(sgn * d0, sgn * (d0 + 5)));
      if (u1 <= u0) continue;
      if (alongX) p.rect(u0, c - wid / 2, u1, c + wid / 2);
      else p.rect(c - wid / 2, u0, c + wid / 2, u1);
    }
  }
}

/** 止まれ painted in the lane, readable by a driver heading h. */
function stopText(p: Painter, x: number, y: number, h: number) {
  const g = p.g;
  g.save();
  g.translate(p.P(x), p.Q(y));
  g.rotate(Math.atan2(Math.cos(h), Math.sin(h)));
  g.scale(1, 2.2);
  g.fillStyle = WHITE;
  g.font = `bold ${0.8 * PX}px sans-serif`;
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText('止まれ', 0, 0);
  g.restore();
}

/** The south street crossing the south arm (stop signs on the side street). */
function drawCrossJunction(p: Painter) {
  const { g } = p;
  const B = BLOCK, y = -B;
  paintGround(p, [[-WALK_OUT, p.y0, WALK_OUT, p.y1], [p.x0, y - SIDE_WALK, p.x1, y + SIDE_WALK]]);
  g.fillStyle = ASPHALT;
  p.rect(-ROAD_HALF, p.y0, ROAD_HALF, p.y1);
  p.rect(p.x0, y - SIDE_HALF, p.x1, y + SIDE_HALF);
  for (const sx of [1, -1]) for (const sy of [1, -1]) roundCorner(p, sx * ROAD_HALF, y + sy * SIDE_HALF, sx, sy);
  g.fillStyle = WHITE;
  // Main road (priority): centre and lane lines run through, edge lines break at the mouths
  p.rect(-0.1, p.y0, 0.1, p.y1);
  for (const s of [1, -1]) {
    dashes(p, false, s * 3.5, p.y0, p.y1, 0.15);
    const m = SIDE_HALF + CORNER_R;
    p.rect(s * 7.2 - 0.075, p.y0, s * 7.2 + 0.075, y - m);
    p.rect(s * 7.2 - 0.075, y + m, s * 7.2 + 0.075, p.y1);
  }
  // Side street: centre line, stop lines, 止まれ
  for (const s of [1, -1]) {
    const x0 = s * (ROAD_HALF + 0.5), x1 = s * JUNCTION_HALF;
    p.rect(x0, y - 0.06, x1, y + 0.06);
    // Westbound traffic (from the east) keeps to the south half, eastbound to the north half.
    const lane = s > 0 ? [y - SIDE_HALF, y] : [y, y + SIDE_HALF];
    p.rect(s * SIDE_STOP, lane[0], s * (SIDE_STOP + 0.45), lane[1]);
    stopText(p, s * (SIDE_STOP + 2.4), (lane[0] + lane[1]) / 2, s > 0 ? Math.PI : 0);
  }
}

/** Where the east / west street meets the main road: the kerb and sidewalk open up, stop line. */
function drawTeeMouth(p: Painter, x: number) {
  const { g } = p;
  paintGround(p, [[p.x0, -WALK_OUT, p.x1, -ROAD_HALF], [x - SIDE_WALK, p.y0, x + SIDE_WALK, -ROAD_HALF]]);
  g.fillStyle = ASPHALT;
  p.rect(p.x0, -ROAD_HALF, p.x1, p.y1);
  p.rect(x - SIDE_HALF, p.y0, x + SIDE_HALF, -ROAD_HALF);
  for (const s of [1, -1]) roundCorner(p, x + s * SIDE_HALF, -ROAD_HALF, s, -1);
  g.fillStyle = WHITE;
  const m = SIDE_HALF + CORNER_R;
  p.rect(p.x0, -7.2 - 0.075, x - m, -7.2 + 0.075);
  p.rect(x + m, -7.2 - 0.075, p.x1, -7.2 + 0.075);
  p.rect(x - 0.06, p.y0, x + 0.06, -ROAD_HALF - 0.5);
  // Northbound traffic keeps to the west half
  p.rect(x - SIDE_HALF, -SIDE_STOP - 0.45, x, -SIDE_STOP);
}

/** A corner of the loop where two side streets meet. */
function drawBend(p: Painter, x: number) {
  const { g } = p;
  const y = -BLOCK, sx = Math.sign(x); // the bend opens to the north and towards the centre (-sx)
  paintGround(p, [[x - SIDE_WALK, y - SIDE_WALK, x + SIDE_WALK, p.y1], [x + sx * SIDE_WALK, y - SIDE_WALK, x - sx * JUNCTION_HALF, y + SIDE_WALK]]);
  g.fillStyle = ASPHALT;
  p.rect(x - SIDE_HALF, y - SIDE_HALF, x + SIDE_HALF, p.y1);
  p.rect(x + sx * SIDE_HALF, y - SIDE_HALF, x - sx * JUNCTION_HALF, y + SIDE_HALF);
  roundCorner(p, x - sx * SIDE_HALF, y + SIDE_HALF, -sx, 1);
  g.fillStyle = WHITE;
  p.rect(x - 0.06, y, x + 0.06, p.y1);
  p.rect(x, y - 0.06, x - sx * JUNCTION_HALF, y + 0.06);
}

/** Cross-section of a side street (one lane each way, sidewalks), 10 m along. */
function drawSideSection(g: CanvasRenderingContext2D, w: number, h: number) {
  const s = w / (2 * SIDE_WALK);
  const X = (x: number) => (x + SIDE_WALK) * s;
  g.fillStyle = SIDEWALK; g.fillRect(0, 0, w, h);
  sidewalkTiles(g, 0, 0, w, h, s * 0.5);
  g.fillStyle = ASPHALT; g.fillRect(X(-SIDE_HALF), 0, 2 * SIDE_HALF * s, h);
  noise(g, w, h, 14, 6);
  g.fillStyle = WHITE;
  g.fillRect(X(0) - 0.06 * s, 0, 0.12 * s, h / 2); // dashed centre line
  for (const x of [-SIDE_HALF + 0.2, SIDE_HALF - 0.2]) g.fillRect(X(x) - 0.06 * s, 0, 0.12 * s, h);
}

/** Inverted-triangle 止まれ signs on posts at the side-street stop lines. */
function stopSignMeshes() {
  const group = new THREE.Group();
  const tex = canvasTexture(128, 116, (g) => {
    g.clearRect(0, 0, 128, 116);
    const tri = (inset: number) => {
      g.beginPath(); g.moveTo(inset * 1.7, inset); g.lineTo(128 - inset * 1.7, inset); g.lineTo(64, 116 - inset * 2); g.closePath();
    };
    g.fillStyle = '#ffffff'; tri(0); g.fill();
    g.fillStyle = '#d0201c'; tri(6); g.fill();
    g.fillStyle = '#ffffff'; g.font = 'bold 26px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText('止まれ', 64, 36);
  }, false);
  const signMat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, side: THREE.DoubleSide });
  const poleMat = new THREE.MeshLambertMaterial({ color: 0x9a9da0 });
  for (const s of stopSigns()) {
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 2.4, 8).rotateX(Math.PI / 2), poleMat);
    pole.position.set(s.x, s.y, 1.2);
    pole.castShadow = true;
    const sign = new THREE.Mesh(new THREE.PlaneGeometry(0.8, 0.72), signMat);
    sign.position.set(s.x + Math.cos(s.face) * 0.05, s.y + Math.sin(s.face) * 0.05, 2.1);
    sign.rotation.set(Math.PI / 2, 0, 0); // upright, facing -y
    sign.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), s.face + Math.PI / 2);
    group.add(pole, sign);
  }
  return group;
}

const ASPHALT = '#4b4d51';
const SIDEWALK = '#b8b3aa';
const WHITE = '#ececea';
const YELLOW = '#e8b52a';

/** Cross-section of an arm (x across, y along 10 m), used beyond the centre square. */
function drawArmSection(g: CanvasRenderingContext2D, w: number, h: number) {
  const s = w / (2 * WALK_OUT); // px per metre across
  const X = (x: number) => (x + WALK_OUT) * s;
  g.fillStyle = SIDEWALK; g.fillRect(0, 0, w, h);
  sidewalkTiles(g, 0, 0, w, h, s * 0.5);
  g.fillStyle = ASPHALT; g.fillRect(X(-ROAD_HALF), 0, 2 * ROAD_HALF * s, h);
  noise(g, w, h, 14, 5);
  g.fillStyle = WHITE;
  const line = (x: number, wid: number, dash = false) => {
    if (dash) g.fillRect(X(x) - (wid * s) / 2, 0, wid * s, h / 2);
    else g.fillRect(X(x) - (wid * s) / 2, 0, wid * s, h);
  };
  line(0, 0.2); line(-7.2, 0.15); line(7.2, 0.15);
  line(-3.5, 0.15, true); line(3.5, 0.15, true);
}

function sidewalkTiles(g: CanvasRenderingContext2D, x0: number, y0: number, w: number, h: number, step: number) {
  g.strokeStyle = 'rgba(90,85,78,0.25)';
  g.lineWidth = 1;
  g.beginPath();
  for (let x = x0; x <= x0 + w; x += step) { g.moveTo(x, y0); g.lineTo(x, y0 + h); }
  for (let y = y0; y <= y0 + h; y += step) { g.moveTo(x0, y); g.lineTo(x0 + w, y); }
  g.stroke();
}

/** The intersection itself, as seen from above (north up). */
function drawCentre(g: CanvasRenderingContext2D, N: number) {
  const P = (x: number) => (x + INNER_R) * PX; // world x → canvas x
  const Q = (y: number) => (INNER_R - y) * PX; // world y → canvas y
  const rect = (x0: number, y0: number, x1: number, y1: number) =>
    g.fillRect(P(Math.min(x0, x1)), Q(Math.max(y0, y1)), Math.abs(x1 - x0) * PX, Math.abs(y1 - y0) * PX);

  // Lots, sidewalks, asphalt
  g.fillStyle = '#8e8c86'; g.fillRect(0, 0, N, N);
  g.fillStyle = SIDEWALK;
  rect(-WALK_OUT, -INNER_R, WALK_OUT, INNER_R);
  rect(-INNER_R, -WALK_OUT, INNER_R, WALK_OUT);
  sidewalkTiles(g, 0, 0, N, N, PX * 0.5);
  g.fillStyle = '#8e8c86';
  // Blank out tiles outside the sidewalks
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) rect(sx * WALK_OUT, sy * WALK_OUT, sx * INNER_R, sy * INNER_R);
  g.fillStyle = ASPHALT;
  rect(-ROAD_HALF, -INNER_R, ROAD_HALF, INNER_R);
  rect(-INNER_R, -ROAD_HALF, INNER_R, ROAD_HALF);
  // Rounded kerbs at the corners
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    g.fillStyle = ASPHALT;
    rect(sx * ROAD_HALF, sy * ROAD_HALF, sx * (ROAD_HALF + CORNER_R), sy * (ROAD_HALF + CORNER_R));
    g.fillStyle = SIDEWALK;
    g.beginPath();
    g.arc(P(sx * (ROAD_HALF + CORNER_R)), Q(sy * (ROAD_HALF + CORNER_R)), CORNER_R * PX, 0, Math.PI * 2);
    g.fill();
  }
  noise(g, N, N, 12, 9);

  g.fillStyle = WHITE;
  for (const arm of ['N', 'S', 'E', 'W'] as Arm[]) {
    // Work in a frame where the arm points "up": u across (+ = east for N), v outwards.
    const h = ARM_HEADING[arm]; // travel heading of traffic arriving from this arm
    const ox = -Math.cos(h), oy = -Math.sin(h); // outward along the arm
    const ux = -oy, uy = ox; // across, to the left when looking outwards
    const quad = (u0: number, v0: number, u1: number, v1: number) => {
      const pts = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]].map(([u, v]) => [P(u * ux + v * ox), Q(u * uy + v * oy)]);
      g.beginPath();
      g.moveTo(pts[0][0], pts[0][1]);
      for (const p of pts.slice(1)) g.lineTo(p[0], p[1]);
      g.closePath();
      g.fill();
    };
    // Arriving traffic keeps left: looking outwards along the arm, its lanes are on the right (u < 0).
    const vIn = STOP_LINE;
    // Crosswalk zebra (stripes parallel to traffic)
    for (let u = -ROAD_HALF + 0.35; u < ROAD_HALF - 0.3; u += 0.9) quad(u, ROAD_HALF + 0.5, u + 0.45, WALK_OUT - 0.5);
    // Stop line across the arriving lanes
    quad(-ROAD_HALF + 0.2, vIn, -0.2, vIn + 0.45);
    // Centre line: solid white, lane dividers: yellow (no lane change) near the stop line then dashed white
    g.fillStyle = WHITE;
    quad(-0.1, WALK_OUT + 0.3, 0.1, INNER_R);
    quad(-7.25, WALK_OUT + 0.3, -7.1, INNER_R); quad(7.1, WALK_OUT + 0.3, 7.25, INNER_R);
    g.fillStyle = YELLOW;
    quad(-3.58, vIn + 0.5, -3.42, vIn + 30);
    g.fillStyle = WHITE;
    quad(3.42, WALK_OUT + 0.3, 3.58, vIn + 8);
    for (let v = vIn + 30; v < INNER_R; v += 10) quad(-3.58, v + 2, -3.42, v + 7);
    for (let v = vIn + 8; v < INNER_R; v += 10) quad(3.42, v + 2, 3.58, v + 7);
    // Crosswalk ahead diamonds (横断歩道予告) in each arriving lane
    for (const u of [-INNER, -OUTER]) diamond(u, vIn + 22);
    // Lane arrows: inner = straight + right, outer = straight + left
    laneArrow(g, P, Q, ux, uy, ox, oy, -INNER, vIn + 6, 'right');
    laneArrow(g, P, Q, ux, uy, ox, oy, -OUTER, vIn + 6, 'left');
    function diamond(u: number, v: number) {
      const pts = [[u, v - 1.5], [u + 0.75, v], [u, v + 1.5], [u - 0.75, v]].map(([a, b]) => [P(a * ux + b * ox), Q(a * uy + b * oy)]);
      g.strokeStyle = WHITE; g.lineWidth = 0.15 * PX;
      g.beginPath(); g.moveTo(pts[0][0], pts[0][1]);
      for (const p of pts.slice(1)) g.lineTo(p[0], p[1]);
      g.closePath(); g.stroke();
    }
  }
  // Centre diamond
  g.strokeStyle = WHITE; g.lineWidth = 0.2 * PX;
  g.beginPath();
  g.moveTo(P(0), Q(1.2)); g.lineTo(P(1.2), Q(0)); g.lineTo(P(0), Q(-1.2)); g.lineTo(P(-1.2), Q(0));
  g.closePath(); g.stroke();
  // Right-turn guide dots through the intersection
  g.fillStyle = 'rgba(236,236,234,0.85)';
  for (const arm of ['N', 'S', 'E', 'W'] as Arm[]) {
    const p = buildRoute(arm, 'right', INNER, INNER, 20, 20);
    for (let s = p.stopS + 5; s < p.exitS - 3; s += 1.2) {
      const q = p.at(s);
      g.fillRect(P(q.x) - 0.12 * PX, Q(q.y) - 0.12 * PX, 0.24 * PX, 0.24 * PX);
    }
  }
}

function laneArrow(
  g: CanvasRenderingContext2D, P: (x: number) => number, Q: (y: number) => number,
  ux: number, uy: number, ox: number, oy: number, u: number, v: number, turn: 'left' | 'right',
) {
  // Arrow points against the outward direction (towards the intersection): "forward" = -v
  const pt = (a: number, b: number): [number, number] => [P(a * ux + b * ox), Q(a * uy + b * oy)];
  g.strokeStyle = WHITE; g.fillStyle = WHITE;
  g.lineWidth = 0.18 * PX; g.lineCap = 'butt';
  const line = (pts: [number, number][]) => {
    g.beginPath(); g.moveTo(...pts[0]);
    for (const p of pts.slice(1)) g.lineTo(...p);
    g.stroke();
  };
  const head = (tip: [number, number], back1: [number, number], back2: [number, number]) => {
    g.beginPath(); g.moveTo(...tip); g.lineTo(...back1); g.lineTo(...back2); g.closePath(); g.fill();
  };
  // Straight
  line([pt(u, v + 4), pt(u, v - 0.6)]);
  head(pt(u, v - 1.6), pt(u - 0.45, v - 0.5), pt(u + 0.45, v - 0.5));
  // Turn branch: the driver faces -v, so the driver's right is +u.
  const side = turn === 'right' ? 1 : -1;
  line([pt(u, v + 1.8), pt(u + side * 0.9, v + 1.0)]);
  head(pt(u + side * 1.3, v + 0.6), pt(u + side * 0.55, v + 0.55), pt(u + side * 1.05, v + 1.55));
}

// ── Buildings ─────────────────────────────────────────────

const FACADES = [
  [0.86, 0.84, 0.8], [0.78, 0.74, 0.68], [0.93, 0.91, 0.86], [0.66, 0.68, 0.7], [0.82, 0.7, 0.58], [0.72, 0.76, 0.8],
];
const SHOPS = [
  ['コンビニ', '#1f8f4a'], ['ドラッグ', '#1f5fb0'], ['カフェ', '#6b4226'], ['ラーメン', '#c0282a'],
  ['不動産', '#2d6a8a'], ['歯科', '#3f9aa8'], ['書店', '#7a5a2a'], ['銀行', '#20407a'], ['花屋', '#d05a8a'], ['弁当', '#d07a10'],
];

function buildingMeshes() {
  const r = rng(77);
  const group = new THREE.Group();
  const facadeTex = canvasTexture(64, 64, (g) => {
    g.fillStyle = '#ffffff'; g.fillRect(0, 0, 64, 64);
    g.fillStyle = '#56616d'; g.fillRect(10, 14, 44, 30);
    g.fillStyle = '#87929c'; g.fillRect(10, 14, 44, 4);
    g.fillStyle = '#d6d2ca'; g.fillRect(4, 48, 56, 5);
  });
  const wallPos: number[] = [], wallUv: number[] = [], wallCol: number[] = [];
  const roofGeos: THREE.BufferGeometry[] = [];
  const signs: THREE.Mesh[] = [];
  const signCache = new Map<number, THREE.Material>();
  for (const b of buildings()) {
    const col = FACADES[b.color];
    const z0 = 0, z1 = b.h;
    const corners = [[b.x - b.hx, b.y - b.hy], [b.x + b.hx, b.y - b.hy], [b.x + b.hx, b.y + b.hy], [b.x - b.hx, b.y + b.hy]];
    for (let i = 0; i < 4; i++) {
      const [x0, y0] = corners[i], [x1, y1] = corners[(i + 1) % 4];
      const len = Math.hypot(x1 - x0, y1 - y0);
      const u1 = len / 4, vTop = (z1 - z0) / 3.3;
      wallPos.push(x0, y0, z0, x1, y1, z0, x1, y1, z1, x0, y0, z0, x1, y1, z1, x0, y0, z1);
      wallUv.push(0, 0, u1, 0, u1, vTop, 0, 0, u1, vTop, 0, vTop);
      for (let k = 0; k < 6; k++) wallCol.push(col[0], col[1], col[2]);
    }
    const roof = new THREE.BoxGeometry(2 * b.hx + 0.3, 2 * b.hy + 0.3, 0.4).toNonIndexed();
    roof.translate(b.x, b.y, z1 + 0.2);
    roof.deleteAttribute('uv');
    colorize(roof, new THREE.Color(0.45 + r() * 0.1, 0.45, 0.44));
    roofGeos.push(roof);
    // Shops: an awning and a sign on the face towards the nearer road
    if (b.kind === 'shop' && Math.min(Math.abs(b.x) - b.hx, Math.abs(b.y) - b.hy) < 22) {
      const towardsEW = Math.abs(b.y) - b.hy < Math.abs(b.x) - b.hx;
      const si = Math.floor(r() * SHOPS.length);
      let mat = signCache.get(si);
      if (!mat) {
        const [text, bg] = SHOPS[si];
        mat = new THREE.MeshBasicMaterial({
          map: canvasTexture(256, 64, (g) => {
            g.fillStyle = bg; g.fillRect(0, 0, 256, 64);
            g.fillStyle = '#fff'; g.font = 'bold 40px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
            g.fillText(text, 128, 34);
          }, false),
        });
        signCache.set(si, mat);
      }
      const w = Math.min(8, 2 * (towardsEW ? b.hx : b.hy) - 1);
      const sign = new THREE.Mesh(new THREE.PlaneGeometry(w, w / 4), mat);
      const nx = towardsEW ? 0 : -Math.sign(b.x), ny = towardsEW ? -Math.sign(b.y) : 0;
      sign.position.set(b.x + nx * ((towardsEW ? 0 : b.hx) + 0.05), b.y + ny * ((towardsEW ? b.hy : 0) + 0.05), 3.4);
      sign.rotation.set(Math.PI / 2, 0, 0); // upright, facing -y
      sign.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), Math.atan2(ny, nx) + Math.PI / 2);
      signs.push(sign);
      const awning = new THREE.Mesh(new THREE.BoxGeometry(towardsEW ? w : 1.2, towardsEW ? 1.2 : w, 0.12),
        new THREE.MeshLambertMaterial({ color: new THREE.Color(SHOPS[si][1]) }));
      awning.position.set(b.x + nx * ((towardsEW ? 0 : b.hx) + 0.6), b.y + ny * ((towardsEW ? b.hy : 0) + 0.6), 2.7);
      awning.castShadow = true;
      signs.push(awning);
    }
  }
  const walls = new THREE.BufferGeometry();
  walls.setAttribute('position', new THREE.Float32BufferAttribute(wallPos, 3));
  walls.setAttribute('uv', new THREE.Float32BufferAttribute(wallUv, 2));
  walls.setAttribute('color', new THREE.Float32BufferAttribute(wallCol, 3));
  walls.computeVertexNormals();
  const wm = new THREE.Mesh(walls, new THREE.MeshLambertMaterial({ map: facadeTex, vertexColors: true, side: THREE.DoubleSide }));
  wm.castShadow = true;
  wm.receiveShadow = true;
  group.add(wm);
  const roofs = new THREE.Mesh(mergeGeometries(roofGeos), new THREE.MeshLambertMaterial({ vertexColors: true }));
  roofs.castShadow = true;
  group.add(roofs);
  for (const s of signs) group.add(s);
  return group;
}

// ── Vegetation, wires, lights, mountains ──────────────────

function treeCrowns() {
  const pts = streetTrees();
  const crown = new THREE.InstancedMesh(new THREE.IcosahedronGeometry(1.8, 1), new THREE.MeshLambertMaterial({ color: 0x4b7a3c }), pts.length);
  const m = new THREE.Matrix4(), q = new THREE.Quaternion(), sc = new THREE.Vector3(), p = new THREE.Vector3();
  const tint = new THREE.Color();
  const r = rng(12);
  pts.forEach(([x, y], i) => {
    const s = 0.85 + r() * 0.35;
    crown.setMatrixAt(i, m.compose(p.set(x, y, 3.2 + 1.6 * s), q.identity(), sc.set(s, s, s * 1.1)));
    crown.setColorAt(i, tint.setHSL(0.27 + r() * 0.04, 0.4, 0.3 + r() * 0.06));
  });
  crown.castShadow = true;
  return crown;
}

function wires() {
  const pts: number[] = [];
  const poles = utilityPoles();
  // Connect consecutive poles on the same line (same x or same y side)
  const lines = new Map<string, [number, number][]>();
  for (const [x, y] of poles) {
    const key = Math.abs(x) > Math.abs(y) && Math.abs(y) < WALK_OUT ? `y${Math.sign(y)}` : `x${Math.sign(x)}`;
    (lines.get(key) ?? lines.set(key, []).get(key)!).push([x, y]);
  }
  for (const [key, ps] of lines) {
    const along = key.startsWith('x') ? 1 : 0;
    ps.sort((a, b) => a[along] - b[along]);
    for (let i = 0; i + 1 < ps.length; i++) {
      const [x0, y0] = ps[i], [x1, y1] = ps[i + 1];
      for (const h of [9.2, 9.8, 10.4]) {
        const n = 12;
        for (let k = 0; k < n; k++) {
          const t0 = k / n, t1 = (k + 1) / n;
          const sag = (t: number) => h - 0.8 * 4 * t * (1 - t);
          pts.push(x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0, sag(t0), x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1, sag(t1));
        }
      }
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  const group = new THREE.Group();
  group.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0x202020 })));
  // Cross arms on the poles
  const arm = new THREE.InstancedMesh(new THREE.BoxGeometry(1.6, 0.12, 0.12), new THREE.MeshLambertMaterial({ color: 0x555555 }), poles.length);
  const m = new THREE.Matrix4();
  poles.forEach(([x, y], i) => {
    // Cross arms run across the wires
    const rot = Math.abs(x) > Math.abs(y) && Math.abs(y) < WALK_OUT ? Math.PI / 2 : 0;
    arm.setMatrixAt(i, m.makeRotationZ(rot).setPosition(x, y, 10.4));
  });
  group.add(arm);
  return group;
}

function streetLights() {
  const group = new THREE.Group();
  const poleMat = new THREE.MeshLambertMaterial({ color: 0x8a8d90 });
  const lampMat = new THREE.MeshBasicMaterial({ color: 0xfff6dd });
  const pos: [number, number, number][] = [];
  for (let t = 30; t < 140; t += 34) {
    pos.push([ROAD_HALF + 0.4, t + 10, Math.PI], [-ROAD_HALF - 0.4, -t, 0], [t, -ROAD_HALF - 0.4, Math.PI / 2], [-t - 10, ROAD_HALF + 0.4, -Math.PI / 2]);
  }
  for (const [x, y, rot] of pos) {
    if (nearSideStreet(x, y, x, y, 1)) continue;
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.1, 8, 8).rotateX(Math.PI / 2), poleMat);
    pole.position.set(x, y, 4);
    const arm = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.08, 0.08), poleMat);
    arm.position.set(x + Math.cos(rot) * 0.9, y + Math.sin(rot) * 0.9, 7.95);
    arm.rotation.z = rot;
    const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.25, 0.1), lampMat);
    lamp.position.set(x + Math.cos(rot) * 1.7, y + Math.sin(rot) * 1.7, 7.88);
    lamp.rotation.z = rot;
    group.add(pole, arm, lamp);
  }
  return group;
}

function mountains() {
  const group = new THREE.Group();
  const r = rng(31);
  const mats = [0x7f93a3, 0x8b9eab, 0x74899a].map((c) => new THREE.MeshLambertMaterial({ color: c, flatShading: true }));
  for (let i = 0; i < 26; i++) {
    const a = (i / 26) * Math.PI * 2 + r() * 0.2;
    const d = 650 + r() * 200;
    const h = 60 + r() * 140;
    const cone = new THREE.Mesh(new THREE.ConeGeometry(120 + r() * 120, h, 7).rotateX(Math.PI / 2), mats[i % 3]);
    cone.position.set(Math.cos(a) * d, Math.sin(a) * d, h / 2 - 5);
    group.add(cone);
  }
  // Distant town silhouettes
  const far = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshLambertMaterial({ color: 0xa4a9ad }), 160);
  const m = new THREE.Matrix4();
  let n = 0;
  for (let i = 0; i < 160; i++) {
    const a = r() * Math.PI * 2, d = 190 + r() * 220;
    const w = 10 + r() * 22, h = 8 + r() * 45;
    const x = Math.cos(a) * d, y = Math.sin(a) * d;
    if (Math.abs(x) < 20 || Math.abs(y) < 20) continue; // keep the road corridors open
    m.makeScale(w, w * (0.6 + r() * 0.8), h).setPosition(x, y, h / 2);
    far.setMatrixAt(n++, m);
  }
  far.count = n;
  group.add(far);
  return group;
}
