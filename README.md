# 工大祭用ロボット制御システム（STM32 + PS3コントローラ + Raspberry Pi）
## 2026.10.15
## 概要

本プロジェクトは、PS3コントローラで操作する3輪オムニホイールロボットの制御システムである。

システムは以下の3要素で構成されている。

- PS3コントローラ
- STM32（ロボット制御）
- Raspberry Pi（画像・音声演出）

STM32はロボット本体の制御を担当し、Raspberry Piはロボットの状態に応じた画像表示や効果音再生を担当する。

---

# システム構成

```text
PS3 Controller
       │
       │ Bluetooth
       ▼
     SBDBT
       │ UART
       ▼
┌─────────────────┐
│      STM32      │
│                 │
│ ・オムニ走行    │
│ ・アーム制御    │
│ ・ハンド制御    │
│ ・LED制御       │
└────────┬────────┘
         │ USB Serial
         ▼
┌─────────────────┐
│ Raspberry Pi    │
│                 │
│ ・画像表示      │
│ ・効果音再生    │
└─────────────────┘
```

---

# ソース構成

```text
.
├── main.cpp             # STM32メイン制御
├── ps3_sbdbt.cpp        # PS3通信ライブラリ
├── ps3_sbdbt.h
├── pin_define.h
│
├── koudaisai2026.py     # Raspberry Pi側演出制御
│
├── standby.png
├── moving.png
├── arm.png
├── hand_open.png
├── hand_close.png
│
├── motion_moving.wav
├── motion_arm.wav
├── hand_open.wav
├── hand_close.wav
└── yobikomikun.wav
```

---

# STM32側ソフトウェア

## 役割

STM32は以下を制御する。

- オムニホイール
- アーム昇降
- ハンド開閉
- 状態LED
- ブザー
- Raspberry Piへの状態通知

メイン処理は `main.cpp` に実装されている。

---

# 操作体系

## オムニ移動

### 左スティック

| 操作 | 動作 |
|--------|--------|
| 前後 | 前進・後退 |
| 左右 | 平行移動 |

左スティックの入力から移動方向ベクトルを生成し、3輪オムニの各モータへ分配している。

### 右スティックX軸

| 操作 | 動作 |
|--------|--------|
| 左右 | 旋回 |

回転成分として各ホイールへ加算する。

```cpp
omega = rx;
```

---

## アーム操作

### 右スティックY軸

| 操作 | 動作 |
|--------|--------|
| 上下 | アーム昇降 |

通常時は直接PWM制御を行う。

```cpp
PwmArm(ry);
```

---

## ハンド操作

| ボタン | 動作 |
|----------|----------|
| × | 開く |
| ○ | 閉じる |
| R2 | 閉じる |

```cpp
isOpen  = sbdbt.batu();
isClose = (sbdbt.maru() | sbdbt.R2());
```

---

# ボタン割り当て一覧

| ボタン | 機能 |
|----------|----------|
| L1 | 中心軸走行モード |
| L2 | 呼び込みモード |
| × | ハンド開 |
| ○ | ハンド閉 |
| △ | 1段目へ移動 |
| □ | 2段目へ移動 |
| ↑ | 大秘宝位置 |
| ← | コアラ位置 |
| ↓ | 最下位置 |

---

# アーム自動位置決め

アームにはポテンショメータ（VR）が接続されており、現在位置をAD値で取得している。

特定ボタンを押すと目標位置へ自動移動する。

## 登録位置

```cpp
#define DAIHIHOU_POSITION 510
#define COALA_POSITION    460

#define POSITION_1st      495
#define POSITION_2nd      550
```

| ボタン | 移動先 |
|----------|----------|
| ← | コアラ位置 |
| ↑ | 大秘宝位置 |
| △ | 1段目 |
| □ | 2段目 |
| ↓ | 最下位置 |

---

# アーム制御

## 機構限界

アーム部の減速機保護のためソフト的に上下限を設けている。

```cpp
#define ARM_UNDER_LIMIT 405
#define ARM_UPPER_LIMIT 640
```

限界到達時はその方向への駆動を停止する。

---

## 自動位置制御

関数：

```cpp
ArmGoToPosition()
```

内部では簡易P制御を使用している。

```cpp
cmd = gain * error
```

特徴

- フィードバック制御
- ヒステリシス付き停止判定
- 最大速度制限
- 目標付近でゲイン変更

---

# オムニホイール制御

## 構成

120度配置の3輪オムニ。

```text
         Motor1

Motor2            Motor3
```

---

## 計算フロー

### 1. スティックから方向ベクトル生成

```cpp
theta = atan2(lx_filtered, ly_filtered);
```

### 2. 各輪への分配

```cpp
cosA
cosB
cosC
```

### 3. PWM生成

```cpp
f1
f2
f3
```

### 4. 回転成分加算

```cpp
f1 += -omega;
f2 +=  omega;
f3 +=  omega;
```

---

# フィルタ処理

操作性改善のため複数のフィルタを入れている。

## Expo補正

スティック中央付近を扱いやすくする。

```cpp
expo(x, 0.5f);
```

---

## 慣性フィルタ

急激な入力変化を抑制。

```cpp
omega_filtered
lx_filtered
ly_filtered
```

---

## 直進補正

前後移動中に横方向入力を弱める。

```cpp
straight_gain_x
straight_gain_y
```

斜め方向への意図しない流れを抑制する目的。

---

# アナログ入力処理

アーム位置検出用ポテンショメータはノイズ除去を行っている。

関数：

```cpp
readAnalogAveraged10bit()
```

処理内容

1. ADCサンプリング
2. Median Filter
3. 平均化
4. ローパスフィルタ

```text
ADC
 ↓
Median
 ↓
Average
 ↓
LPF
```

---

# ハンド制御

ハンドモータはタイマ割り込み内でソフトPWM制御している。

## 関連変数

```cpp
volatile int handDuty;
volatile int handDir;
```

## 制御方向

```cpp
handDir = 1   // 閉じる
handDir = -1  // 開く
```

## 割り込み関数

```cpp
interrupt_005ms()
```

TIM4割り込みから実行される。

---

# LED表示

RGB LEDでロボット状態を表示する。

| 状態 | 色 |
|--------|--------|
| 呼び込み | 白 |
| 1段目 | 緑 |
| 2段目 | 紫 |
| ハンド開 | 青 |
| ハンド閉 | 赤 |
| 最下位置 | 黄 |
| 通常 | 消灯 |

---

# 起動音

## 通常起動

```cpp
toneSTM(4000, bpm, 0.07);
```

電子音を2回鳴らす。

---

## 呼び込みメロディ

```cpp
yobikomi_No4()
```

呼び込み君風メロディ。

---

## ファミマ音

実装済み。

```cpp
famima()
```

有効化する場合

```cpp
#define FAMIMA 1
```

---

# PS3通信ライブラリ

ファイル

```text
ps3_sbdbt.cpp
```

---

## 概要

SBDBTから送信されるシリアルデータを解析するライブラリ。

STM32版へ移植している。

---

## データ更新

```cpp
sbdbt.update();
```

シリアル受信データを内部バッファへ格納する。

---

## ボタン取得

```cpp
sbdbt.maru();
sbdbt.batu();

sbdbt.L1();
sbdbt.L2();

sbdbt.R1();
sbdbt.R2();
```

---

## スティック取得

```cpp
sbdbt.ls_x();
sbdbt.ls_y();

sbdbt.rs_x();
sbdbt.rs_y();
```

---

# Raspberry Pi側

ファイル

```text
koudaisai2026.py
```

---

## 役割

STM32から送られてきた状態コードを受信し、

- 画像表示
- 効果音再生

を行う。

---

# STM32との通信

## インタフェース

```text
USB Serial
9600bps
```

STM32側は以下を送信する。

```cpp
sendRasipSerial();
```

---

# 通信フォーマット

STM32はロボット状態を1Byteで通知する。

| Bit | 内容 |
|--------|--------|
| bit0 | 走行中 |
| bit1 | アーム動作 |
| bit2 | ハンド閉 |
| bit3 | ハンド開 |
| bit4 | 呼び込み |

---

## 例

### オムニ移動

```text
00000001
```

```cpp
0x01
```

---

### アーム＋ハンド閉

```text
00000110
```

```cpp
0x06
```

---

### 全動作

```text
00011111
```

```cpp
0x1F
```

---

# Raspberry Pi側演出

## 表示画像

| 状態 | ファイル |
|--------|--------|
| 待機 | standby.png |
| 走行 | moving.png |
| アーム | arm.png |
| ハンド開 | hand_open.png |
| ハンド閉 | hand_close.png |

---

## 効果音

| 状態 | ファイル |
|--------|--------|
| 走行 | motion_moving.wav |
| アーム | motion_arm.wav |
| ハンド開 | hand_open.wav |
| ハンド閉 | hand_close.wav |
| 呼び込み | yobikomikun.wav |

---

# 自動復旧機能

Raspberry Pi側にはソフトウェアウォッチドッグを実装している。

```python
WATCHDOG_TIMEOUT = 5.0
```

STM32から5秒以上データが来ない場合、

```python
restart_program()
```

を実行し、自動的に再起動する。

---

# 起動手順

## STM32

通常通り書き込み後に電源投入。

---

## Raspberry Pi

```bash
python3 koudaisai2026.py
```

起動時に

1. pygame初期化
2. シリアル接続
3. 画像ロード
4. 待機画面表示

を行う。

---

# キャリブレーション項目

機体差が出やすいため、以下は現物合わせで調整すること。

```cpp
COALA_POSITION
DAIHIHOU_POSITION

POSITION_1st
POSITION_2nd

ARM_UNDER_LIMIT
ARM_UPPER_LIMIT
```

---

# トラブルシュート

## アームが停止する

VR値が上下限へ到達している可能性あり。

確認箇所

```cpp
ARM_UNDER_LIMIT
ARM_UPPER_LIMIT
```

---

## Raspberry Piが反応しない

確認項目

```bash
ls /dev/ttyUSB*
```

またシリアル設定が一致しているか確認する。

```python
serial.Serial('/dev/ttyUSB0', 9600)
```

---

# 製作者メモ

ロボットの挙動を変更したい場合は以下を確認する。

- 移動制御 → `main.cpp`
- アーム制御 → `ArmGoToPosition()`
- ハンド制御 → `motorHand()`
- PS3受信処理 → `ps3_sbdbt.cpp`

演出を変更したい場合は以下を確認する。

- 画像表示 → `photoShow()`
- 音声再生 → `playEffects()`
- 状態判定 → `decode_STM_serial()`

設計思想としては、

**STM32 = 制御担当**

**Raspberry Pi = 演出担当**

に完全分離しており、両者はUSBシリアルで状態情報のみをやり取りする構成となっている。
