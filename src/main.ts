// App shell: loading, main loop, cameras, HUD, settings, collision alarms, replay.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url';
import { DEFAULT_SETTINGS, IntersectionSim, type CrashEvent, type EgoMode, type Pedal, type Settings } from './sim.ts';
import { OVERLAY_LAYER, SceneView } from './render.ts';
import { SIM_DT } from './world.ts';
import { signalAt, type Light } from './layout.ts';
import { getLang, onLangChange, setLang, t, type Key } from './i18n.ts';
import { Player, Recording } from './replay.ts';

const MAX_STEPS_PER_FRAME = 60;
const ALERT_HOLD_MS = 1800;
const OVERHEAD_HALF = 46; // metres shown either side of the centre on the overhead map
const SETTINGS_KEY = 'intersection-settings';

type View = 'driver' | 'chase' | 'orbit' | 'corner';
const VIEWS: View[] = ['driver', 'chase', 'orbit', 'corner'];
const MODES: EgoMode[] = ['auto', 'pedal', 'manual'];

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const wStatus = $<HTMLDivElement>('status');
const hud = $<HTMLPreElement>('hud');
const logEl = $<HTMLOListElement>('log');
const controls = document.querySelector<HTMLDivElement>('.controls')!;
const settingsEl = $<HTMLElement>('settings');
const steerSlider = $<HTMLInputElement>('steer');
const steerReadout = $<HTMLOutputElement>('steer-readout');
const pedalButtons = [...document.querySelectorAll<HTMLButtonElement>('button[data-pedal]')];
const modeButtons = [...document.querySelectorAll<HTMLButtonElement>('button[data-mode]')];
const viewButtons = [...document.querySelectorAll<HTMLButtonElement>('button[data-view]')];
const speedButtons = [...document.querySelectorAll<HTMLButtonElement>('button[data-speed]')];
const sliders = [...document.querySelectorAll<HTMLInputElement>('input[data-key]')];
const alertEl = $<HTMLDivElement>('alert');
const alertTitle = $<HTMLElement>('alert-title');
const alertSub = $<HTMLSpanElement>('alert-sub');
const pedalSpeed = $<HTMLOutputElement>('pedal-speed');
const replayBtn = $<HTMLButtonElement>('replay-btn');
const replayPlayBtn = $<HTMLButtonElement>('replay-play');
const replaySeek = $<HTMLInputElement>('replay-seek');
const replayTime = $<HTMLOutputElement>('replay-time');
const replayVideoBtn = $<HTMLButtonElement>('replay-video');
const replayFile = $<HTMLInputElement>('replay-file');

let statusKey: Key = 'status.loading';
let statusColor = '#aaa';
let statusExtra = '';
function setStatus(key: Key, color: string, extra = '') {
  statusKey = key;
  statusColor = color;
  statusExtra = extra;
  renderStatus();
}
function renderStatus() {
  wStatus.innerHTML = `<span style="color:${statusColor}">${t(statusKey)}${statusExtra}</span>`;
}

setLang(getLang());
setStatus('status.loading', '#aaa');

// ── Load MuJoCo and build the world ──
const mj = await loadMujoco({ locateFile: (path: string) => (path.endsWith('.wasm') ? wasmUrl : path) }).catch((err) => {
  setStatus('status.error', '#f66', ` ${String(err)}`);
  throw err;
});
const sim = new IntersectionSim(mj, undefined, (Math.random() * 1e9) | 0);
loadSettings();
const view = new SceneView(mj, sim);
const CAM = mj.mjtObj.mjOBJ_CAMERA.value;
const driverCamId = mj.mj_name2id(sim.model, CAM, 'driver_cam');
const chaseCamId = mj.mj_name2id(sim.model, CAM, 'chase_cam');
// Drive recorder: the last minutes of the run, to watch again from any camera.
let rec = new Recording(sim.model.nq, sim.agents.length);
let recLoaded = false; // rec came from a file, not this run
const player = new Player(mj, sim.model);

function makeRenderer(canvas: HTMLCanvasElement, shadows: boolean) {
  const r = new THREE.WebGLRenderer({ canvas, antialias: true });
  r.outputColorSpace = THREE.LinearSRGBColorSpace;
  r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  r.shadowMap.enabled = shadows;
  r.shadowMap.type = THREE.PCFShadowMap;
  return r;
}
const mainRenderer = makeRenderer($('main-canvas'), true);
const overheadRenderer = makeRenderer($('overhead-canvas'), false);
const mainCam = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 3000);
const orbitCam = new THREE.PerspectiveCamera(50, 16 / 9, 0.5, 3000);
orbitCam.up.set(0, 0, 1);
orbitCam.position.set(-24, -33, 22);
const orbit = new OrbitControls(orbitCam, $('main-canvas'));
orbit.target.set(0, 2, 0);
orbit.maxPolarAngle = Math.PI * 0.49;
orbit.minDistance = 5;
orbit.maxDistance = 300;
orbit.update();
const overheadCam = new THREE.OrthographicCamera(-OVERHEAD_HALF, OVERHEAD_HALF, OVERHEAD_HALF, -OVERHEAD_HALF, 1, 500);
overheadCam.up.set(0, 1, 0);
overheadCam.position.set(0, 0, 200);
overheadCam.lookAt(0, 0, 0);
overheadCam.layers.enable(OVERLAY_LAYER);

function fitRenderer(r: THREE.WebGLRenderer, cam: THREE.PerspectiveCamera | null) {
  const el = r.domElement.parentElement!;
  const { clientWidth: w, clientHeight: h } = el;
  const size = r.getSize(new THREE.Vector2());
  if (size.x !== w || size.y !== h) r.setSize(w, h, false);
  if (cam) {
    cam.aspect = w / h;
    cam.updateProjectionMatrix();
  }
}

// ── Cameras ──
let currentView: View = 'driver';

function activeCamera(): THREE.PerspectiveCamera {
  if (currentView === 'orbit') return orbitCam;
  const d = replaying ? player.data : sim.data;
  if (currentView === 'driver') view.setFromModelCamera(mainCam, driverCamId, d);
  else if (currentView === 'chase') view.setFromModelCamera(mainCam, chaseCamId, d);
  else {
    // Traffic camera on the south-east corner: the ego comes up on the left and turns towards it
    mainCam.position.set(13.8, -10.5, 7.5);
    mainCam.up.set(0, 0, 1);
    mainCam.lookAt(-2.5, 3, 0);
    mainCam.fov = 58;
    mainCam.updateProjectionMatrix();
    mainCam.updateMatrixWorld();
  }
  return mainCam;
}

function drawFrame() {
  const d = replaying ? player.data : sim.data;
  if (replaying) {
    view.sync(player.data, player.active);
    view.updateSignals(signalAt(replayT), performance.now() / 1000);
    replayMarkers();
  } else {
    view.sync();
    view.updateSignals(sim.signal, performance.now() / 1000);
    view.updateMarkers(sim.time);
  }
  const eye: [number, number, number] = [d.cam_xpos[3 * driverCamId], d.cam_xpos[3 * driverCamId + 1], d.cam_xpos[3 * driverCamId + 2]];
  view.updateSight(eye, !replaying && sim.ego.active && !sim.ego.crashed ? sim.egoSight : []);

  const cam = activeCamera();
  fitRenderer(mainRenderer, cam);
  if (currentView === 'orbit') view.followShadow(orbit.target.x, orbit.target.y);
  else if (currentView === 'corner') view.followShadow(0, 0);
  else view.followShadow(eye[0], eye[1]);
  mainRenderer.render(view.scene, cam);

  // The overhead map follows the car around the town.
  const fq = sim.ego.fq;
  const ex = d.qpos[fq], ey = d.qpos[fq + 1];
  overheadCam.position.set(ex, ey, 200);
  overheadCam.lookAt(ex, ey, 0);
  fitRenderer(overheadRenderer, null);
  overheadRenderer.render(view.scene, overheadCam);
  if (replaying) updateReplayHud();
  else updateHud();
}

// ── HUD ──
const LAMP: Record<Light, string> = { G: '#2fe39a', Y: '#ffc020', R: '#ff3322' };
const PHASE_COLOR: Record<string, string> = {
  approach: '#ddd', lane: '#c9a0ff', red: '#ff8870', wait: '#ffd060', stop: '#ffb060',
  right: '#80c8ff', left: '#80e8d0', free: '#aaa', crash: '#ff5040',
};
const pad = (k: Key) => t(k).padEnd(getLang() === 'ja' ? 6 : 12);
function lamp(color: string, on: boolean) {
  return `<i class="lamp" style="background:${on ? color : '#2a2a2a'}"></i>`;
}
function updateHud() {
  const s = sim.signal;
  const e = sim.egoInfo();
  const kmh = Math.abs(e.speed * 3.6).toFixed(0);
  pedalSpeed.value = `${kmh} km/h`;
  const hidden = sim.egoSight.filter((x) => !x.visible).length;
  const sig =
    lamp(LAMP.G, s.ns === 'G') + lamp(LAMP.Y, s.ns === 'Y') + lamp(LAMP.R, s.ns === 'R') +
    (s.nsArrow ? ' <span style="color:#2fe39a">➡</span>' : '') + ` <span class="small">${s.remaining.toFixed(0)}s</span>`;
  const active = sim.agents.filter((a) => a.active && !a.ego).length;
  const next = sim.egoNext();
  const st = sim.stats;
  hud.innerHTML =
    `<span style="color:#9f9">${pad('hud.speed')}${kmh.padStart(3)} km/h</span>\n` +
    `${pad('hud.signal')}${sig}\n` +
    `${pad('hud.next')}${next.turn === 'right' ? '↱' : '↰'} ${t(`j.${next.j}` as Key)} · ${t(`turn.${next.turn}` as Key)}\n` +
    `${pad('hud.state')}<span style="color:${PHASE_COLOR[sim.egoPhase]}">${t(`phase.${sim.egoPhase}` as Key)}</span>` +
    (hidden && sim.egoPhase !== 'crash' ? ` <span style="color:#ff6a50">👁 ${hidden} ${t('hud.hidden')}</span>` : '') + '\n' +
    `<span style="color:#7dff90">${pad('hud.turns')}${t('turn.right')} ${st.rights} · ${t('turn.left')} ${st.lefts} · ${t('hud.lane')} ${st.laneChanges}</span>\n` +
    `<span style="color:#7dff90">${pad('hud.laps')}${String(st.laps).padStart(3)}</span>\n` +
    `<span style="color:#ff9070">${pad('hud.egocrash')}${String(sim.stats.egoCrashes).padStart(3)}</span>\n` +
    `<span style="color:#ffc070">${pad('hud.crashes')}${String(sim.stats.crashes).padStart(3)}` +
    (st.rollovers ? ` · ${t('log.rollover')} ${st.rollovers}` : '') + '</span>\n' +
    `<span style="color:#aaa">${pad('hud.traffic')}${String(active).padStart(3)} · ${pad('hud.time')}${fmtTime(sim.time)}</span>`;
}
const fmtTime = (s: number) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
const fmtTenths = (s: number) => `${Math.floor(s / 60)}:${(s % 60).toFixed(1).padStart(4, '0')}`;

// ── Replay: watch the recorded run again, from any camera ──
let replaying = false;
let replayPlaying = false;
let replayT = 0;
let markersUpTo = -Infinity; // crash markers already placed for events up to this time
let video: MediaRecorder | null = null;

function updateReplayHud() {
  const s = signalAt(replayT);
  const kmh = (player.egoSpeed(rec, replayT, sim.ego.fq) * 3.6).toFixed(0);
  const sig = lamp(LAMP.G, s.ns === 'G') + lamp(LAMP.Y, s.ns === 'Y') + lamp(LAMP.R, s.ns === 'R') +
    (s.nsArrow ? ' <span style="color:#2fe39a">➡</span>' : '');
  hud.innerHTML =
    `<span style="color:#c9a0ff">🎞 ${t('replay.hud')}  ${fmtTenths(replayT - rec.start)} / ${fmtTenths(rec.end - rec.start)}</span>\n` +
    `<span style="color:#9f9">${pad('hud.speed')}${kmh.padStart(3)} km/h</span>\n` +
    `${pad('hud.signal')}${sig}\n` +
    `<span style="color:#aaa">${pad('hud.time')}${fmtTime(replayT)}</span>`;
  pedalSpeed.value = `${kmh} km/h`;
  replayTime.value = `${fmtTenths(replayT - rec.start)} / ${fmtTenths(rec.end - rec.start)}`;
  if (document.activeElement !== replaySeek) replaySeek.value = String(replayT);
}

/** Crash rings for the recorded events as the replay passes them (re-placed after seeking back). */
function replayMarkers() {
  if (replayT < markersUpTo) {
    view.clearMarkers();
    markersUpTo = replayT - 4;
  }
  for (const e of rec.events) if (e.t > markersUpTo && e.t <= replayT) view.addCrashMarker(e.x, e.y, e.t);
  markersUpTo = replayT;
  view.updateMarkers(replayT);
}

function seekReplay(t: number) {
  replayT = Math.max(rec.start, Math.min(rec.end, t));
  player.seek(rec, replayT);
}

function setReplayPlaying(on: boolean) {
  replayPlaying = on;
  if (on && replayT >= rec.end - 1e-3) seekReplay(rec.start);
  replayPlayBtn.textContent = on ? '⏸' : '▶';
}

function enterReplay() {
  if (rec.length < 2) {
    setStatus('replay.empty', '#f96');
    return;
  }
  if (running) onStop();
  replaying = true;
  replaySeek.min = String(rec.start);
  replaySeek.max = String(rec.end);
  view.clearMarkers();
  markersUpTo = -Infinity;
  alertEl.hidden = true;
  seekReplay(Math.max(rec.start, rec.end - 20)); // the last 20 s: usually what you want to see again
  markersUpTo = replayT - 4;
  setReplayPlaying(true);
  setStatus('status.replay', '#c9a0ff');
  syncButtons();
}

function exitReplay() {
  if (!replaying) return;
  stopVideo();
  replaying = false;
  replayPlaying = false;
  view.clearMarkers();
  if (recLoaded) {
    // A replay opened from a file is not part of this run
    rec = new Recording(sim.model.nq, sim.agents.length);
    recLoaded = false;
  }
  setStatus('status.paused', '#f96');
  syncButtons();
}

function saveReplay() {
  if (rec.length < 2) return;
  const a = document.createElement('a');
  a.href = URL.createObjectURL(rec.toBlob());
  a.download = `intersection-replay-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.n55r`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}

async function openReplay(file: File) {
  try {
    const r = Recording.fromBuffer(await file.arrayBuffer(), sim.model.nq, sim.agents.length);
    if (r.length < 2) throw new Error('empty');
    stopVideo();
    replaying = false;
    rec = r;
    recLoaded = true;
    enterReplay();
  } catch {
    setStatus('replay.bad', '#f66');
  }
}

/** Record the main view as the replay plays (current camera) and save it as a WebM file. */
function startVideo() {
  const canvas = $<HTMLCanvasElement>('main-canvas');
  const type = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm'].find((m) => MediaRecorder.isTypeSupported(m));
  if (!type || !canvas.captureStream) return;
  const chunks: Blob[] = [];
  video = new MediaRecorder(canvas.captureStream(30), { mimeType: type, videoBitsPerSecond: 8e6 });
  video.ondataavailable = (e) => { if (e.data.size) chunks.push(e.data); };
  video.onstop = () => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob(chunks, { type: 'video/webm' }));
    a.download = `intersection-${currentView}-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.webm`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  };
  video.start(500);
  setReplayPlaying(true);
  setStatus('status.recording', '#ff6060');
  syncButtons();
}
function stopVideo() {
  if (!video) return;
  if (video.state !== 'inactive') video.stop();
  video = null;
  if (replaying) setStatus('status.replay', '#c9a0ff');
  syncButtons();
}

// ── Collisions: log, alarm, overlay ──
const kindKey = (i: number): Key => `kind.${sim.agentName(i)}` as Key;
function describe(e: CrashEvent) {
  if (e.rollover) return `${t(kindKey(e.a))} ${t('log.rollover')} · ${(e.speed * 3.6).toFixed(0)} km/h`;
  // Put the ego first when it is involved
  const [a, b] = sim.agents[e.b]?.ego ? [e.b, e.a] : [e.a, e.b];
  return `${t(kindKey(a))} × ${t(kindKey(b))} · ${(e.speed * 3.6).toFixed(0)} km/h`;
}
function renderLog() {
  if (!sim.events.length) {
    logEl.innerHTML = `<li class="empty">${t('log.empty')}</li>`;
    return;
  }
  logEl.innerHTML = sim.events.slice(-40).reverse()
    .map((e) => `<li class="${e.ego ? 'ego' : ''}">${fmtTime(e.t)} ${describe(e)}</li>`).join('');
}

let audio: AudioContext | null = null;
function unlockAudio() {
  try {
    audio ??= new AudioContext();
    if (audio.state === 'suspended') void audio.resume();
  } catch {
    audio = null;
  }
}
function beep(t0: number, f1: number, f2: number, len = 0.3) {
  if (!audio) return;
  const gain = audio.createGain();
  gain.connect(audio.destination);
  gain.gain.setValueAtTime(0.0001, t0);
  gain.gain.exponentialRampToValueAtTime(0.16, t0 + 0.01);
  gain.gain.setValueAtTime(0.16, t0 + len - 0.04);
  gain.gain.exponentialRampToValueAtTime(0.0001, t0 + len);
  const osc = audio.createOscillator();
  osc.type = 'square';
  osc.frequency.setValueAtTime(f1, t0);
  osc.frequency.setValueAtTime(f2, t0 + len / 2);
  osc.connect(gain);
  osc.start(t0);
  osc.stop(t0 + len + 0.01);
}
let lastAlarm = 0;
function alarm(ego: boolean) {
  if (!audio) return;
  const now = audio.currentTime;
  if (now - lastAlarm < 0.9) return;
  lastAlarm = now;
  const n = ego ? 3 : 2;
  for (let i = 0; i < n; i++) beep(now + i * 0.4, ego ? 960 : 740, ego ? 720 : 560);
}

let alertUntil = 0;
let alertEvent: CrashEvent | null = null;
function renderAlert() {
  if (!alertEvent) return;
  alertTitle.textContent = alertEvent.rollover
    ? t(alertEvent.ego ? 'alert.egoRollover' : 'alert.rollover')
    : t(alertEvent.ego ? 'alert.ego' : 'alert.crash');
  alertSub.textContent = describe(alertEvent);
  alertEl.classList.toggle('minor', !alertEvent.ego);
}
function drainEvents(now: number) {
  if (!sim.fresh.length) {
    if (!alertEl.hidden && now >= alertUntil) {
      alertEl.hidden = true;
      alertEvent = null;
    }
    return;
  }
  for (const e of sim.fresh) {
    view.addCrashMarker(e.x, e.y, e.t);
    // An ego crash takes over the banner; others only if nothing more important is showing.
    if (!alertEvent || e.ego || !alertEvent.ego || now >= alertUntil) alertEvent = e;
    alarm(e.ego);
  }
  sim.fresh.length = 0;
  renderAlert();
  alertEl.hidden = false;
  alertUntil = now + ALERT_HOLD_MS;
  renderLog();
}

// ── Main loop ──
let running = false;
let timeScale = 1;
let lastT = 0;
let acc = 0;

function tick(now: number) {
  if (running) {
    acc += Math.min(((now - lastT) / 1000) * timeScale, MAX_STEPS_PER_FRAME * SIM_DT);
    lastT = now;
    while (acc >= SIM_DT) {
      sim.step();
      rec.capture(sim);
      acc -= SIM_DT;
    }
    drainEvents(now);
    if (sim.pedal !== shownPedal) syncPedal(); // the sim resets it when the ego respawns
  } else if (replaying) {
    const dt = Math.min((now - lastT) / 1000, 0.1);
    lastT = now;
    if (replayPlaying) {
      seekReplay(replayT + dt * timeScale);
      if (replayT >= rec.end) {
        setReplayPlaying(false);
        stopVideo();
      }
    }
  } else {
    lastT = now;
  }
  if (currentView === 'orbit') orbit.update();
  drawFrame();
  requestAnimationFrame(tick);
}

function onStart() {
  unlockAudio();
  exitReplay();
  if (running) return;
  running = true;
  acc = 0;
  lastT = performance.now();
  setStatus('status.running', '#6f6');
}
function onStop() {
  running = false;
  setStatus('status.paused', '#f96');
}
function onReset() {
  exitReplay();
  sim.reset();
  rec.clear();
  view.clearMarkers();
  alertEl.hidden = true;
  alertEvent = null;
  setSteer(0);
  syncPedal();
  renderLog();
  setStatus(running ? 'status.running' : 'status.reset', running ? '#6f6' : '#aaf');
}

// ── Controls ──
function syncButtons() {
  for (const b of modeButtons) b.classList.toggle('selected', b.dataset.mode === sim.egoMode);
  for (const b of viewButtons) b.classList.toggle('selected', b.dataset.view === currentView);
  for (const b of speedButtons) b.classList.toggle('selected', Number(b.dataset.speed) === timeScale);
  controls.classList.toggle('auto', sim.egoMode === 'auto');
  controls.classList.toggle('manual', sim.egoMode === 'manual');
  controls.classList.toggle('replaying', replaying);
  replayBtn.setAttribute('aria-pressed', String(replaying));
  replayVideoBtn.setAttribute('aria-pressed', String(!!video));
  $('settings-btn').setAttribute('aria-pressed', String(!settingsEl.hidden));
}

function setMode(m: EgoMode) {
  if (m === sim.egoMode) return;
  sim.egoMode = m;
  if (m !== 'auto') {
    // Take over smoothly: keep rolling at the current speed.
    sim.pedal = sim.ego.wheelVel > 0.5 ? 'hold' : 'none';
    setSteer(0);
  }
  syncPedal();
  syncButtons();
}
for (const b of modeButtons) b.addEventListener('click', () => setMode(b.dataset.mode as EgoMode));

function setView(v: View) {
  currentView = v;
  syncButtons();
}
for (const b of viewButtons) b.addEventListener('click', () => setView(b.dataset.view as View));

function setSpeed(s: number) {
  timeScale = s;
  syncButtons();
}
for (const b of speedButtons) b.addEventListener('click', () => setSpeed(Number(b.dataset.speed)));

let shownPedal: Pedal | null = null;
function syncPedal() {
  shownPedal = sim.pedal;
  for (const b of pedalButtons) b.setAttribute('aria-pressed', String(b.dataset.pedal === sim.pedal));
}
const togglePedal = (p: Pedal) => {
  sim.pedal = sim.pedal === p ? 'none' : p;
  syncPedal();
};
for (const b of pedalButtons) b.addEventListener('click', () => togglePedal(b.dataset.pedal as Pedal));

function setSteer(v: number) {
  v = THREE.MathUtils.clamp(v, -0.6, 0.6);
  steerSlider.value = String(v);
  onSteerInput();
}
function onSteerInput() {
  const v = Number(steerSlider.value);
  steerReadout.value = v.toFixed(2);
  sim.manualSteer = -v; // slider right → steer right (negative yaw)
}
steerSlider.addEventListener('input', onSteerInput);

$('start').addEventListener('click', onStart);
$('stop').addEventListener('click', onStop);
$('reset').addEventListener('click', onReset);
const toggleSettings = () => {
  settingsEl.hidden = !settingsEl.hidden;
  syncButtons();
};
$('settings-btn').addEventListener('click', toggleSettings);
const toggleLang = () => setLang(getLang() === 'en' ? 'ja' : 'en');
$('lang').addEventListener('click', toggleLang);

const toggleReplay = () => (replaying ? exitReplay() : enterReplay());
replayBtn.addEventListener('click', toggleReplay);
replayPlayBtn.addEventListener('click', () => setReplayPlaying(!replayPlaying));
replaySeek.addEventListener('input', () => seekReplay(Number(replaySeek.value)));
$('replay-save').addEventListener('click', saveReplay);
$('replay-open').addEventListener('click', () => replayFile.click());
replayFile.addEventListener('change', () => {
  const f = replayFile.files?.[0];
  if (f) void openReplay(f);
  replayFile.value = '';
});
replayVideoBtn.addEventListener('click', () => (video ? stopVideo() : startVideo()));
$('replay-exit').addEventListener('click', exitReplay);

// ── Settings sliders ──
function renderSliders() {
  for (const s of sliders) {
    const k = s.dataset.key as keyof Settings;
    s.value = String(sim.settings[k]);
    const out = s.nextElementSibling as HTMLOutputElement;
    const unit = out.dataset.unit ? ` ${t(out.dataset.unit as Key)}` : out.dataset.suffix ?? '';
    out.value = out.dataset.onoff ? t(sim.settings[k] ? 'set.on' : 'set.off') : `${sim.settings[k]}${unit}`;
  }
}
for (const s of sliders) {
  s.addEventListener('input', () => {
    (sim.settings as unknown as Record<string, number>)[s.dataset.key!] = Number(s.value);
    renderSliders();
    saveSettings();
  });
}
$('defaults').addEventListener('click', () => {
  Object.assign(sim.settings, DEFAULT_SETTINGS);
  renderSliders();
  saveSettings();
});
function saveSettings() {
  try {
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(sim.settings));
  } catch {
    // storage unavailable
  }
}
function loadSettings() {
  try {
    const saved = JSON.parse(localStorage.getItem(SETTINGS_KEY) ?? 'null');
    if (saved && typeof saved === 'object') {
      for (const k of Object.keys(DEFAULT_SETTINGS) as (keyof Settings)[]) {
        if (typeof saved[k] === 'number' && Number.isFinite(saved[k])) sim.settings[k] = saved[k];
      }
    }
  } catch {
    // storage unavailable or corrupt: keep defaults
  }
}

onLangChange(() => {
  renderStatus();
  renderAlert();
  renderLog();
  renderSliders();
});

// ── Keyboard ──
window.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLInputElement && e.key.startsWith('Arrow')) return;
  if (replaying) {
    // Replay: Space play/pause, ←/→ 2 s back/forward, Esc or R to leave
    switch (e.key) {
      case ' ': setReplayPlaying(!replayPlaying); e.preventDefault(); return;
      case 'ArrowLeft': seekReplay(replayT - 2); e.preventDefault(); return;
      case 'ArrowRight': seekReplay(replayT + 2); e.preventDefault(); return;
      case 'Escape': case 'r': case 'R': exitReplay(); e.preventDefault(); return;
      case 'w': case 'W': case 'h': case 'H': case 'b': case 'B': case 'c': case 'C': case 'm': case 'M': return;
    }
  }
  const driving = sim.egoMode !== 'auto';
  switch (e.key) {
    case 'r': case 'R': toggleReplay(); break;
    case 'Enter': running ? onStop() : onStart(); break;
    case 'ArrowLeft': if (sim.egoMode === 'manual') setSteer(Number(steerSlider.value) - 0.04); break;
    case 'ArrowRight': if (sim.egoMode === 'manual') setSteer(Number(steerSlider.value) + 0.04); break;
    case 'c': case 'C': if (sim.egoMode === 'manual') setSteer(0); break;
    case ' ': case 'w': case 'W': if (driving) togglePedal('accel'); break;
    case 'h': case 'H': if (driving) togglePedal('hold'); break;
    case 'b': case 'B': if (driving) togglePedal('brake'); break;
    case 'm': case 'M': setMode(MODES[(MODES.indexOf(sim.egoMode) + 1) % MODES.length]); break;
    case 'v': case 'V': setView(VIEWS[(VIEWS.indexOf(currentView) + 1) % VIEWS.length]); break;
    case 's': case 'S': toggleSettings(); break;
    case 'l': case 'L': toggleLang(); break;
    case '1': setSpeed(0.25); break;
    case '2': setSpeed(0.5); break;
    case '3': setSpeed(1); break;
    case '4': setSpeed(2); break;
    default: return;
  }
  e.preventDefault();
});

// URL options, e.g. ?mode=auto&view=orbit&lang=ja&rate=0.5&gap=2&autostart
const params = new URLSearchParams(location.search);
const pLang = params.get('lang'), pMode = params.get('mode'), pView = params.get('view'), pRate = Number(params.get('rate'));
if (pLang === 'en' || pLang === 'ja') setLang(pLang);
if (pMode && (MODES as string[]).includes(pMode)) setMode(pMode as EgoMode);
if (pView && (VIEWS as string[]).includes(pView)) currentView = pView as View;
if ([0.25, 0.5, 1, 2].includes(pRate)) timeScale = pRate;
// Traffic settings can be given in the URL too, e.g. ?gap=2&motoPct=40 (not saved).
for (const k of Object.keys(DEFAULT_SETTINGS) as (keyof Settings)[]) {
  const v = params.get(k);
  if (v !== null && v !== '' && Number.isFinite(Number(v))) sim.settings[k] = Number(v);
}

renderSliders();
renderLog();
syncPedal();
syncButtons();
setStatus('status.ready', '#aaa');
if (params.has('autostart')) onStart();
requestAnimationFrame(tick);
