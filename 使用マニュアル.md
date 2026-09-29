# 交差点事故シミュレーター デスクトップ版 使用マニュアル
# Intersection Accident Simulator Desktop App - User Manual

[English below](#english-user-manual)

---

# 日本語 ユーザーマニュアル

本アプリケーションは、MuJoCo 物理エンジン（WASM）と Three.js 描画エンジンを用いた、左側通行・片側2車線の交差点および周辺街区（8の字周回）における交通事故・死角体験シミュレーター（Desktop App）です。

---

## 1. インストールと起動

### macOS の場合
1. `Intersection Accident_1.0.0_aarch64.dmg` をダブルクリックして開きます。
2. アプリアイコンを `Applications`（アプリケーション）フォルダにドラッグ＆ドロップします。
3. 初回起動時、「開発元が未確認のため開けません」と表示された場合：
   - `システム設定` > `プライバシーとセキュリティ` を開き、「このまま開く」をクリックするか、
   - アプリを右クリック（二本指タップ）して「開く」を選択してください。

### Windows の場合
1. `Intersection Accident_1.0.0_x64-setup.exe`（NSIS インストーラー）を実行してインストールします（または `Intersection Accident_1.0.0_x64_en-US.msi` を実行）。
2. Windows SmartScreen の警告が表示された場合：
   - 「詳細情報」をクリックし、「実行」をクリックしてください。

---

## 2. 画面構成と基本機能

```
┌─────────────────────────────────────────────────────────────┐
│ 🛰 [上空マップ・視線レーダー]     [HUD情報: 速度・状態・死角数]   │
│                                                             │
│                                                             │
│                      3D シミュレーション画面                  │
│                                                             │
│                                                             │
│ 🕹 [運転モード]  [カメラ]  [ペダル]  [速度]  [設定]  [リプレイ] │
└─────────────────────────────────────────────────────────────┘
```

1. **メイン 3D ビュー**:
   - 交差点・車両・歩行者・建物・信号機をリアルタイム物理演算で描画します。
2. **🛰 上空マップ（左上）**:
   - 自車を中心としたバードアイビュー。自車の運転者から対象への視線レイ（Raycast）が表示されます。
   - **緑色の線**: 運転者の視界に入っている対象
   - **赤色の線**: トラックやピラーなどの死角に入って見えていない対象
3. **HUD パネル**:
   - 自車速度、運転状態（走行中・右折待ち・赤信号など）、信号状態、周回数、事故回数、視界内の死角数（`👁 n 死角`）を表示。
4. **衝突アラート & 衝突ログ（右側）**:
   - 衝突発生時に衝突速度と当事者（例: 自車 × バイク 38 km/h）を表示・記録します。

---

## 3. 運転モード

画面下部の **モード切り替えボタン**（または `M` キー）で切り替えます。

| モード | 説明 |
|---|---|
| 🤖 **自動 (Auto)** | 自車が完全自動で 8の字コースを周回します。対向車の接近速度や死角を計算し、安全な間隔を見極めて右折・合流・車線変更を行います。 |
| 🦶 **ペダル (Pedal)** | ハンドル操作と車線変更は自動ですが、**加速・維持・ブレーキのタイミングをプレイヤーが操作**します。対向車や歩行者の隙間を突いて右折する判断を体験できます。 |
| 🎮 **手動 (Manual)** | **ハンドル操舵（←/→ キー）とペダル操作の両方をプレイヤーが完全にコントロール**します。周回コースを外れて街区全体（±170m）を自由にドライブすることも可能です。 |

---

## 4. カメラ視点

画面下部の **カメラボタン**（または `V` キー）で切り替えます。

| カメラ | 説明 |
|---|---|
| 🪟 **運転席 (Driver)** | 運転者の目線位置からの視点。フロントピラーや対向右折大型車によるリアルな死角を体感できます。 |
| 🚗 **後方 (Chase)** | 自車の後方上空から追従する三人称視点。周囲の状況を把握しやすい視点です。 |
| 🎥 **自由 (Orbit)** | マウスドラッグで視点を自由回転、ホイールでズーム、右ボタンドラッグで視点移動できます。 |
| 📹 **定点 (Corner)** | 南東角の電柱上部に設置された交通監視カメラ視点。交差点全体の挙動を客観的に観察できます。 |

---

## 5. 操作キーボードショートカット一覧

| キー | 動作 |
|---|---|
| `Enter` | シミュレーションの **開始 / 一時停止** |
| `M` | **運転モード切り替え**（自動 ⇄ ペダル ⇄ 手動） |
| `V` | **カメラ視点切り替え**（運転席 ⇄ 後方 ⇄ 自由 ⇄ 定点） |
| `S` | **交通設定パネル** の開閉 |
| `R` | **リプレイモード** の起動 / 終了 |
| `L` | **表示言語の切り替え**（日本語 ⇄ English） |
| `1` / `2` / `3` / `4` | **再生速度の変更**（0.25× / 0.5× / 1× / 2×） |
| `Space` / `W` | 【ペダル/手動】**加速 (ACCEL)** |
| `H` | 【ペダル/手動】**速度維持 (HOLD)** |
| `B` | 【ペダル/手動】**ブレーキ (BRAKE)** |
| `←` / `→` | 【手動モード】**ハンドル左 / 右操舵** |
| `C` | 【手動モード】**ハンドルを中央（直進）に戻す** |

---

## 6. 交通設定パラメータ

`S` キーまたは「設定」ボタンで各種パラメータを調整し、様々な事故シナリオを意図的に再現できます。

- **南北道路の量 / 東西道路の量 (台/分)**: 交通密度を調整。増やすと右折機会が減少します。
- **車の速度 (km/h)**: 車両の標準走行速度。
- **トラックの割合 (%)**: 大型車が増えると対向車線の死角が大幅に増加します。
- **バイクの割合 (%)**: 二輪車はトラックの陰に隠れやすく、いわゆる「右直事故」の主因になります。
- **歩行者 / 自転車の量 (人/分)**: 横断歩道の横断頻度。
- **自車が待つ間隔 (Accepted Gap, 秒)**: 自車が右折や合流を決断する際に対向車との間に必要とする最小余裕時間。短くすると危険なタイミングで右折し事故が多発します。
- **反応時間 (秒)**: 周囲のドライバーが進路塞ぎを認識してブレーキを踏むまでの遅延時間。
- **黄信号で突っ込む車 (%)**: 信号変化時に無理に交差点に進入する車両の割合。
- **左折前に左へ寄せる**: オンにすると左折時の二輪車巻き込みを防止する挙動をとります。

---

## 7. リプレイ & 動画書き出し機能

`R` キーまたは「🎞 リプレイ」ボタンを押すと、直近のシミュレーション走行記録を見直すことができます。

- **再生 / 一時停止**: `Space` キー
- **時間送り / 戻し**: `←` / `→` キー（2秒スキップ）またはスライダー操作
- **カメラ切替**: リプレイ中もすべてのカメラ（運転席・後方・自由・定点）を自由に切り替え可能
- **💾 保存**: リプレイ走行データを JSON ファイルとしてローカルに保存
- **📂 開く**: 以前保存したリプレイファイルを開いて再鑑賞
- **⏺ 動画 (WebM)**: 現在のカメラ視点でリプレイを WebM 動画ファイルとしてエクスポート
- **✕ 終了 (Esc)**: リアルタイムシミュレーション画面に復帰

---
---

# English User Manual

This application is a desktop simulator for traffic accident analysis, sightline tracking, and blind-spot awareness at a two-lane intersection and surrounding town loop block (figure-of-eight course), powered by **MuJoCo WASM** physics and **Three.js** 3D rendering.

---

## 1. Installation and Setup

### macOS
1. Open `Intersection Accident_1.0.0_aarch64.dmg`.
2. Drag and drop the app icon into your `Applications` folder.
3. If macOS displays a Gatekeeper prompt ("unidentified developer"):
   - Go to `System Settings` > `Privacy & Security` and click "Open Anyway", or
   - Right-click (or two-finger click) the app icon in Finder and select "Open".

### Windows
1. Run `Intersection Accident_1.0.0_x64-setup.exe` (NSIS Installer) or `Intersection Accident_1.0.0_x64_en-US.msi`.
2. If Windows Defender SmartScreen appears:
   - Click **"More info"**, then click **"Run anyway"**.

---

## 2. Interface Overview

1. **Main 3D Viewport**:
   - Renders realistic rigid-body vehicle dynamics, collision impulses, suspension movement, and ragdoll physics upon impact.
2. **🛰 Overhead Radar & Sightlines (Top-Left)**:
   - Tracks the ego vehicle from above. Real-time raycasts (`mj_ray`) are drawn from the driver’s eye position:
     - **Green lines**: Target vehicles/pedestrians currently visible to the driver.
     - **Red lines**: Blocked by trucks, oncoming vehicles, or pillars (Blind spots).
3. **HUD Dashboard**:
   - Displays speed, current state (approaching, waiting to turn, red light, etc.), signal phase, completed laps, collisions, and hidden vehicles in blind spots (`👁 n hidden`).
4. **Collision Banner & Log (Right Side)**:
   - Records every collision event with timestamp, impact speed, and involved agents (e.g. `Ego × Motorcycle 38 km/h`).

---

## 3. Driving Modes

Toggle using the **Mode button** in the lower control bar or press **`M`**.

| Mode | Description |
|---|---|
| 🤖 **Auto** | Fully autonomous navigation around the figure-of-eight town loop. The ego vehicle evaluates oncoming vehicle speeds, sightlines, and safe gaps before executing turns and lane changes. |
| 🦶 **Pedal** | Steering and lane keeping are automated, but **acceleration, cruise hold, and braking are controlled manually by you**. Experience the risk of gap judgment when turning right across oncoming traffic. |
| 🎮 **Manual** | **Full manual control of steering (`←`/`→`) and pedals**. You can also leave the designated loop and freely explore the entire town layout (±170 m). |

---

## 4. Camera Views

Toggle using the **Camera button** or press **`V`**.

| View | Description |
|---|---|
| 🪟 **Driver** | First-person perspective from the driver's seat. Ideal for observing real blind spots created by pillars and large opposing vehicles. |
| 🚗 **Chase** | Third-person tracking camera behind the ego vehicle. |
| 🎥 **Orbit** | Free-form inspection camera. Left-click drag to rotate, mouse wheel to zoom, right-click drag to pan. |
| 📹 **Corner** | Fixed CCTV traffic camera stationed on the south-east corner light pole. |

---

## 5. Keyboard Shortcuts

| Key | Function |
|---|---|
| `Enter` | **Start / Pause** simulation |
| `M` | **Cycle driving mode** (Auto ⇄ Pedal ⇄ Manual) |
| `V` | **Cycle camera view** (Driver ⇄ Chase ⇄ Orbit ⇄ Corner) |
| `S` | Toggle **Traffic Settings** panel |
| `R` | Enter / Exit **Replay Mode** |
| `L` | Toggle **Language** (English ⇄ Japanese) |
| `1` / `2` / `3` / `4` | Set **Simulation Speed** (0.25× / 0.5× / 1× / 2×) |
| `Space` / `W` | [Pedal/Manual] **ACCEL** (Accelerate) |
| `H` | [Pedal/Manual] **HOLD** (Maintain speed) |
| `B` | [Pedal/Manual] **BRAKE** (Apply brakes) |
| `←` / `→` | [Manual Mode] **Steer Left / Right** |
| `C` | [Manual Mode] **Center steering** |
| `Esc` | [Replay Mode] Exit replay |

---

## 6. Traffic Settings & Crash Factors

Press **`S`** to open the settings drawer and customize variables:

- **North–south / East–west traffic (veh/min)**: Traffic flow volume.
- **Typical speed (km/h)**: Cruise speed of surrounding vehicles.
- **Trucks (%)**: Proportion of large vehicles. Higher values obstruct sightlines significantly.
- **Motorcycles (%)**: Higher values increase frequency of classic right-turn ("smidsy") collisions.
- **Pedestrians / Bicycles (/min)**: Frequency of crosswalk crossings.
- **Accepted gap (seconds)**: Time buffer the ego driver waits for before executing turns. Reducing this setting induces frequent turning collisions.
- **Reaction time (seconds)**: Delay before other drivers initiate emergency braking.
- **Yellow-light runners (%)**: Percentage of traffic accelerating through amber signals.
- **Pull over before left turns**: Toggles hugging the curb prior to left turns to prevent motorcycle hooking accidents.

---

## 7. Replay & Video Export

Press **`R`** or click **🎞 Replay** to review past driving sessions:

- **Play / Pause**: Press `Space`
- **Seek**: Press `←` / `→` (2 seconds skip) or drag the timeline slider
- **Free Camera**: Switch cameras at any time during replay
- **💾 Save**: Export the recording as a session file for future playback
- **📂 Open**: Load and inspect previously recorded sessions
- **⏺ Video**: Record and export high-quality **WebM video** from the active camera
- **✕ Exit**: Return to live physics simulation

---

## 8. Development & Building

To run or build the desktop app from source:

```bash
cd intersection_accident_desktop

# Install dependencies
npm install

# Run desktop app in development mode
npm run tauri dev

# Build production bundle (.dmg for macOS, .exe/.msi for Windows)
npm run tauri build
```
