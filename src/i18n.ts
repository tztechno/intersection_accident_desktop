// English / Japanese UI strings. Static text uses data-i18n (textContent) and
// data-i18n-title (tooltip) attributes; dynamic text calls t().

export type Lang = 'en' | 'ja';

const DICT = {
  'title': ['Town Loop Intersection Sim', '交差点シミュレーター（8の字周回）'],
  'overhead': ['🛰 Overhead · sight lines', '🛰 上空 · 視線'],
  'mode.title': ['Who drives the orange car (M)', 'オレンジの自車を誰が運転するか (M)'],
  'mode.auto': ['🤖 Auto', '🤖 自動'],
  'mode.pedal': ['🦶 Pedal', '🦶 ペダル'],
  'mode.manual': ['🎮 Manual', '🎮 手動'],
  'view.title': ['Camera (V)', 'カメラ (V)'],
  'view.driver': ['🪟 Driver', '🪟 運転席'],
  'view.chase': ['🚗 Chase', '🚗 後方'],
  'view.orbit': ['🎥 Orbit', '🎥 自由'],
  'view.corner': ['📹 Corner', '📹 定点'],
  'sim.start': ['▶ Start', '▶ 開始'],
  'sim.start.title': ['Start simulation (Enter)', 'シミュレーション開始 (Enter)'],
  'sim.stop': ['⏸ Pause', '⏸ 一時停止'],
  'sim.stop.title': ['Pause simulation (Enter)', '一時停止 (Enter)'],
  'sim.reset.title': ['Clear the intersection and start over', '交差点をリセット'],
  'speed.title': ['Playback speed (1-4)', '再生速度 (1〜4)'],
  'lang.title': ['Language (L)', '表示言語 (L)'],
  'settings.title': ['Traffic settings (S)', '交通設定 (S)'],
  'steer.title': ['Steer (←/→, C to center)', 'ステア (←/→、C で中央)'],
  'pedal.accel': ['🔺 ACCEL', '🔺 加速'],
  'pedal.accel.title': ['Accelerate (Space / W)', '加速 (Space / W)'],
  'pedal.hold': ['⏸ HOLD', '⏸ 維持'],
  'pedal.hold.title': ['Hold current speed (H)', '速度維持 (H)'],
  'pedal.brake': ['🔻 BRAKE', '🔻 ブレーキ'],
  'pedal.brake.title': ['Brake (B)', 'ブレーキ (B)'],
  'set.head': ['Traffic settings', '交通設定'],
  'set.oncoming': ['North–south traffic', '南北道路の量'],
  'set.cross': ['East–west traffic', '東西道路の量'],
  'set.speed': ['Typical speed', '車の速度'],
  'set.truck': ['Trucks', 'トラックの割合'],
  'set.moto': ['Motorcycles', 'バイクの割合'],
  'set.peds': ['Pedestrians', '歩行者'],
  'set.bikes': ['Bicycles', '自転車'],
  'set.gap': ['Your accepted gap', '自車が待つ間隔'],
  'set.reaction': ['Reaction time', '反応時間'],
  'set.yellow': ['Yellow-light runners', '黄信号で突っ込む車'],
  'set.keepLeft': ['Pull over before left turns', '左折前に左へ寄せる'],
  'set.on': ['on', 'する'],
  'set.off': ['off', 'しない'],
  'set.defaults': ['Defaults', '初期値に戻す'],
  'set.note': ['Changes apply to new arrivals. Gap and reaction apply at once.', '台数・割合は新しく来る車から反映。間隔と反応時間はすぐ反映。'],
  'unit.permin': ['/min', '台/分'],
  'unit.pplmin': ['/min', '人/分'],
  'status.loading': ['Loading MuJoCo…', 'MuJoCo を読み込み中…'],
  'status.ready': ['Press ▶ Start', '▶ 開始 を押してください'],
  'status.running': ['▶ Running', '▶ 実行中'],
  'status.paused': ['⏸ Paused', '⏸ 一時停止中'],
  'status.reset': ['↺ Reset — press ▶ Start', '↺ リセット — ▶ 開始 を押してください'],
  'status.error': ['Failed to load: ', '読み込みに失敗しました: '],
  'status.replay': ['🎞 Replay — pick any camera', '🎞 リプレイ中 — カメラは自由に切り替え可'],
  'status.recording': ['⏺ Recording video…', '⏺ 動画を書き出し中…'],
  'replay.btn': ['🎞 Replay', '🎞 リプレイ'],
  'replay.title': ['Watch the last minutes again from any camera (R)', '直近の走行を好きなカメラで見直す (R)'],
  'replay.hud': ['Replay', 'リプレイ'],
  'replay.play.title': ['Play / pause (Space)', '再生 / 一時停止 (Space)'],
  'replay.seek.title': ['Seek (←/→ 2 s)', '再生位置 (←/→ で2秒)'],
  'replay.save': ['💾 Save', '💾 保存'],
  'replay.save.title': ['Save the recording to a file (open it later to watch again)', '記録をファイルに保存（後で開いて見直せます）'],
  'replay.open': ['📂 Open', '📂 開く'],
  'replay.open.title': ['Open a saved recording', '保存した記録を開く'],
  'replay.video': ['⏺ Video', '⏺ 動画'],
  'replay.video.title': ['Record the replay as it plays, from the current camera, to a WebM video', '再生中の画面を今のカメラのまま WebM 動画に書き出す'],
  'replay.exit': ['✕ Exit', '✕ 終了'],
  'replay.exit.title': ['Back to the live simulation (Esc)', 'シミュレーションに戻る (Esc)'],
  'replay.empty': ['Nothing recorded yet — press ▶ Start first', 'まだ記録がありません（先に ▶ 開始）'],
  'replay.bad': ['That file is not a replay from this simulator', 'このシミュレーターのリプレイファイルではありません'],
  'phase.approach': ['driving', '走行中'],
  'phase.lane': ['changing lanes', '車線変更'],
  'phase.red': ['stopped at red', '赤信号で停止'],
  'phase.wait': ['waiting to turn', '右折待ち'],
  'phase.stop': ['stop sign · checking', '一時停止・確認'],
  'phase.right': ['turning right', '右折中'],
  'phase.left': ['turning left', '左折中'],
  'phase.free': ['off the loop (free drive)', 'コース外（自由走行）'],
  'phase.crash': ['CRASHED', '事故'],
  'turn.right': ['right', '右折'],
  'turn.left': ['left', '左折'],
  'j.C': ['Chuo 1-chome', '中央一丁目'],
  'j.E': ['Higashi-machi', '東町'],
  'j.W': ['Nishi-machi', '西町'],
  'j.S': ['Minami-machi', '南町'],
  'j.SE': ['SE corner', '南東の角'],
  'j.SW': ['SW corner', '南西の角'],
  'hud.speed': ['Speed', '速度'],
  'hud.state': ['State', '状態'],
  'hud.signal': ['Signal', '信号'],
  'hud.next': ['Next', '次'],
  'hud.turns': ['Turns', '右左折'],
  'hud.lane': ['lane chg', '車線変更'],
  'hud.laps': ['Laps', '周回'],
  'hud.egocrash': ['Your crashes', '自車の事故'],
  'hud.crashes': ['All crashes', '全事故'],
  'hud.traffic': ['On stage', '通行中'],
  'hud.time': ['Time', '経過'],
  'hud.hidden': ['hidden', '死角'],
  'kind.ego': ['your car', '自車'],
  'kind.car': ['car', '乗用車'],
  'kind.truck': ['truck', 'トラック'],
  'kind.moto': ['motorcycle', 'バイク'],
  'kind.bicycle': ['bicycle', '自転車'],
  'kind.ped': ['pedestrian', '歩行者'],
  'kind.object': ['roadside object', '路上構造物'],
  'alert.crash': ['⚠ COLLISION', '⚠ 衝突'],
  'alert.ego': ['⚠ YOUR CAR CRASHED', '⚠ 自車が衝突'],
  'alert.rollover': ['⚠ ROLLOVER', '⚠ 横転'],
  'alert.egoRollover': ['⚠ YOUR CAR ROLLED OVER', '⚠ 自車が横転'],
  'log.rollover': ['rolled over', '横転'],
  'log.head': ['Collisions', '衝突ログ'],
  'log.empty': ['none yet', 'まだありません'],
  'help.keys': [
    'Keys: Enter start/pause · M mode · V camera · S settings · R replay · 1-4 speed · L language',
    'キー: Enter 開始/停止 · M モード · V カメラ · S 設定 · R リプレイ · 1〜4 速度 · L 言語',
  ],
} as const satisfies Record<string, readonly [string, string]>;

export type Key = keyof typeof DICT;

const STORE_KEY = 'intersection-lang';
let lang: Lang = initialLang();
const listeners: (() => void)[] = [];

function initialLang(): Lang {
  try {
    const saved = localStorage.getItem(STORE_KEY);
    if (saved === 'en' || saved === 'ja') return saved;
  } catch {
    // storage unavailable
  }
  return navigator.language.startsWith('ja') ? 'ja' : 'en';
}

export const getLang = () => lang;
export const t = (key: Key) => DICT[key][lang === 'en' ? 0 : 1];

export function onLangChange(fn: () => void) {
  listeners.push(fn);
}

/** Apply the language to every data-i18n / data-i18n-title element and notify listeners. */
export function setLang(l: Lang) {
  lang = l;
  try {
    localStorage.setItem(STORE_KEY, l);
  } catch {
    // storage unavailable
  }
  document.documentElement.lang = l;
  for (const el of document.querySelectorAll<HTMLElement>('[data-i18n]')) el.textContent = t(el.dataset.i18n as Key);
  for (const el of document.querySelectorAll<HTMLElement>('[data-i18n-title]')) {
    el.title = t(el.dataset.i18nTitle as Key);
  }
  document.title = t('title');
  for (const fn of listeners) fn();
}
