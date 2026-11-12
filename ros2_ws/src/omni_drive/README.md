# omni_drive — オムニホイールの足回り

オムニホイール(3 輪 / 4 輪)で全方向に動く足回りを動かす ROS 2 パッケージです。
スワーブ(`swerve_drive`)の**保険**として、同じ使い方で動くようにしてあります。

- **STM32 のファームウェアは、モーター 2 個の速度制御の通常版(`APP` なし)をそのまま使います**。STM32 1 枚でホイール 2 個、4 輪なら 2 枚、3 輪でも 2 枚(1 個は空き)です。
- 入口は `/cmd_vel` なので、**WASD・ゲームパッド・PC から Wi-Fi・Nav2** は、スワーブと同じものがそのまま使えます。
- ホイールの数・位置・向きは `config/omni.yaml` で決めます(3 輪の例もコメントにあります)。

## 全体の流れ

```
 WASD / ゲームパッド / Nav2 ─▶ /cmd_vel ─▶ omni_node ─▶ stm1/command [モーター1, モーター2] ─▶ stm_bridge ─▶ STM32 (ID 1)
                                (vx, vy, ω)    │        stm2/command [モーター1, モーター2] ─▶            ─▶ STM32 (ID 2)
                                               │                                [rad/s]
              /odom、TF(odom → base_link) ◀──┴── stm1/state、stm2/state [実際の速さ 2 個]
              /omni/markers(rviz2 の矢印)
```

## ファイル

| ファイル | 中身 |
|---|---|
| `omni_drive/kinematics.py` | 計算の本体(逆運動学・順運動学・オドメトリ)。ROS なしで動くので、テストしやすい |
| `omni_drive/omni_node.py` | ROS 2 のノード。`/cmd_vel` → 各 STM32 の 2 個のモーター、実際の速さ → `/odom` と TF と矢印 |
| `omni_drive/firmware_wheels.py` | 実機の代わりに、**STM32 のファームウェア(速度の PID)をそのまま動かして**ふりをする(`firmware:=true`、`./sim.sh omni fw`) |
| `omni_drive/fake_wheels.py` | 実機の代わりに、STM32(モーター 2 個)のふりをする |
| `config/omni.yaml` | **ロボットに合わせて書き換える設定**(ホイールの位置・向き、どの基板のどのモーターか) |
| `config/omni.rviz` | rviz2 の表示設定(上から見た図、前が画面の上) |
| `launch/omni.launch.py` | まとめて起動する |
| `test/test_kinematics.py` | 計算のテスト(4 輪の X 型、3 輪、上限、配置のまちがい) |

## 使い方

dev コンテナでビルドしてから(`ros` で入る)使います。

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 1. 実機なしで試す(PC)

ホストで `./sim.sh omni` を実行すると、ビルドから下の起動までを 1 回でやります(コンテナの中なら `~/ros2_ws` で同じ `./sim.sh`)。1 つずつ起動するときは次のとおりです。

```bash
ros2 launch omni_drive omni.launch.py sim:=true console:=true
```

**ロボットコンソール**(スワーブと共通。`swerve_drive/README.md`)が開きます。「ロボット」のカードの矢印は、各ホイールが**床を押す向きと速さ**です(緑が実際、白い点線が目標)。
コンソールを選んだまま、W / A / S / D で移動、Q / E で回転、Shift で速くなります。Esc か停止ボタンで止まります。

### 2. 実機で動かす

1. STM32 に、**モーター 2 個の速度制御の通常版**を、基板ごとに ID を変えて書き込みます(`firmware/motor_node_f303k8/README.md`)。
   ```bash
   ros make -C /root/firmware/motor_node_f303k8 NODE_ID=1 flash      # CAN のとき
   ros make -C /root/firmware/motor_node_f303k8 LINK=uart NODE_ID=1 flash   # USB シリアルのとき
   ```
2. **最初はタイヤを浮かせて**、足回りを起動します。
   ```bash
   ros2 launch omni_drive omni.launch.py console:=true can_interface:=can0
   ```
3. W を押して、ロボットが前に進む向きに各タイヤが回るか確認します。逆に回るタイヤは、`omni.yaml` の `wheel_sign` を `-1` にします(STM32 を書き換えなくてよい)。

Pi で動かして PC から Wi-Fi で操縦するときは、`.env` に `DRIVE=omni` を書いて `docker compose --profile robot up -d drive` を起動し、PC では `swerve_drive` の `teleop.launch.py` を使います(`swerve_drive/README.md` の「実機を PC から Wi-Fi で操縦する」と同じ)。

## ロボットに合わせて変えるところ(`config/omni.yaml`)

| 設定 | 中身 |
|---|---|
| `wheel_x` / `wheel_y` | ロボットの中心から各ホイールまでの距離 [m]。前と左がプラス |
| `wheel_direction_deg` | タイヤが「正」に回ったときに、床を押して進む向き [度]。0 が前、反時計回りが正 |
| `device_ids` / `channels` | そのホイールを、どの STM32(ID)の、どのモーター(1 か 2)につないだか |
| `wheel_sign` | タイヤが逆に回るときに `-1` |
| `wheel_radius` | タイヤの半径 [m] |
| `max_wheel_speed` | タイヤの速さの上限 [rad/s]。超えそうなときは、全部を同じ割合で遅くします |

変えたら `colcon build` し直してから起動します(`fake_wheels` と `stm_bridge` の ID は、起動ファイルが `device_ids` から自動で決めます)。
