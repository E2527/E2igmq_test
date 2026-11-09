# swerve_drive — 独立ステアリング(スワーブ)の足回り

4 つの独立ステアリングのモジュールで、全方向に動く足回りを動かす ROS 2 パッケージです。
ロボット全体の目標速度 `/cmd_vel` を、各モジュールの「向き」と「タイヤの速さ」に変えて STM32 に送り、
STM32 から返ってくる実際の向きと速さから、ロボットがどれだけ動いたか(オドメトリ)を計算します。

## 全体の流れ

```
 ゲームパッド ─ joy ─ teleop_twist_joy ─┐
 キーボード ─ teleop_twist_keyboard ────┼─▶ /cmd_vel ─▶ swerve_node ─▶ stm1〜4/command ─▶ stm_bridge ─▶ STM32 × 4
 Nav2(自動操縦、これから)────────────┘    (vx, vy, ω)      │          [向き, タイヤの速さ]   (CAN / USB シリアル)
                                                            │
                         /odom、TF(odom → base_link) ◀─────┴─── stm1〜4/state ◀───────────────────────────┘
                         /swerve/markers(rviz2 の矢印)              [今の向き, 今のタイヤの速さ]
```

- **操縦のしかたを変えても、`swerve_node` はそのまま**です。`/cmd_vel` を出すものを、ゲームパッド・キーボード・Nav2 のどれにするかだけです。
- **STM32 が向きをどう測るか(ロータリーエンコーダ、ステッピングモーターなど)を変えても、`swerve_node` はそのまま**です。STM32 が下の「約束」を守れば動きます。

## STM32 との約束(トピック)

モジュールごとに、`stm_bridge` を通して次の 2 つをやり取りします(ID は `config/swerve.yaml` の `device_ids`)。

| トピック | 向き | 中身 |
|---|---|---|
| `stm<ID>/command` | Pi → STM32 | `[目標の向き rad, 目標のタイヤの速さ rad/s]` |
| `stm<ID>/state` | STM32 → Pi | `[今の向き rad, 今のタイヤの速さ rad/s]`(10 ms ごと) |

- **向き**:0 がロボットの前、上から見て反時計回りが正、-π〜π。
- **タイヤの速さ**:タイヤの回転の速さ。正で「向き」の方向に進みます。
- 90° より大きく向きを変えるときは、`swerve_node` が向きを 180° 反転して、タイヤを逆回転させます。なので STM32 は、届いた向きにそのまま合わせれば OK です。
- 指令が 100 ms 以上届かなければ、STM32 はタイヤを止めます(今の CAN / USB シリアルの仕組みと同じ)。

## ファイル

| ファイル | 中身 |
|---|---|
| `swerve_drive/kinematics.py` | 計算の本体(逆運動学・順運動学・オドメトリ)。ROS なしで動くので、テストしやすい |
| `swerve_drive/swerve_node.py` | ROS 2 のノード。`/cmd_vel` を受け取って指令を出し、`/odom` と TF と矢印を出す |
| `swerve_drive/fake_modules.py` | 実機の代わりに、モジュールのふりをする(向きはゆっくり回り、タイヤは少し遅れて追いつく) |
| `config/swerve.yaml` | **ロボットに合わせて書き換える設定**(モジュールの位置、タイヤの半径、ID など) |
| `config/joy.yaml` | ゲームパッドのボタンとスティックの割り当て |
| `config/swerve.rviz` | rviz2 の表示設定(上から見た図) |
| `launch/swerve.launch.py` | まとめて起動する |
| `test/test_kinematics.py` | 計算のテスト(`colcon test` で実行される) |

## 使い方

まず、dev コンテナでビルドします(`ros` でコンテナに入ってから)。

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 1. 実機なしで試す(PC)

```bash
ros2 launch swerve_drive swerve.launch.py sim:=true rviz:=true
```

rviz2 に、上から見たロボットが出ます。**赤い矢印が目標の向き、緑の矢印が実際の向き**(長さは速さ)です。
別のターミナルで、キーボードで動かしてみます。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- `i` 前進、`,` 後退、`j` / `l` その場で回転
- **Shift を押しながら** `J` / `L` で、横に移動(全方向に動けるロボット用のモード)
- `k` で止まる

ゲームパッドがあれば、`joy:=true` を付けると使えます(LB を押している間だけ動きます。ボタンの割り当ては `config/joy.yaml`)。

```bash
ros2 launch swerve_drive swerve.launch.py sim:=true rviz:=true joy:=true
```

### 2. 実機で動かす

STM32 × 4 を CAN でつないで、`stm_bridge` ごと起動します(ID は `swerve.yaml` の `device_ids` が使われます)。

```bash
ros2 launch swerve_drive swerve.launch.py joy:=true can_interface:=can0
```

**最初はタイヤを浮かせて**、少しずつ試してください。

## ロボットに合わせて変えるところ(`config/swerve.yaml`)

| 設定 | 中身 |
|---|---|
| `module_x` / `module_y` | ロボットの中心から各モジュールまでの距離 [m]。前と左がプラス |
| `wheel_radius` | タイヤの半径 [m] |
| `max_wheel_speed` | タイヤの速さの上限 [rad/s]。超えそうなときは、全部のモジュールを同じ割合で遅くします |
| `device_ids` | 各モジュールの STM32 の ID |
| `cosine_scale` | 向きが目標からずれている間は、タイヤを遅くします(違う方向へ進むのを防ぐ) |

変えたら、`colcon build` し直してから起動します。

## これからやること

- **STM32 側**:1 モジュール分の、差動ミキシング(2 つのモーター → 向きとタイヤ)と、向きの位置制御。角度センサーが決まったら作ります。
- **自動操縦**:Nav2 の出す `/cmd_vel` に切り替えます。ゲームパッドと Nav2 を切り替えるには `twist_mux` を使います。
- **オドメトリの精度**:タイヤが滑るとずれるので、IMU を足して robot_localization で合成します(そのときは `publish_tf: false`)。
