# robot_nav — 自動操縦(Nav2)

地図の上でゴールをクリックすると、**ぶつからない道を探して、その道に沿って走る**ためのパッケージです。
ROS 2 の自動操縦の部品集 **Nav2** の起動ファイル(`launch/nav.launch.py`)と設定(`config/nav2_params.yaml`)が入っています。
地図は `field_map` がフィールドのファイルから作るので、LiDAR で走り回って地図を作る必要はありません。

## 試す(シミュレーション)

```bash
./sim.sh nav          # 自動操縦つきのシミュレーター(位置はシミュレーションの本当の位置を使う)
./sim.sh amcl         # さらに AMCL で、LiDAR を使って自分の位置を合わせる(実機と同じやり方)
./sim.sh omni nav     # オムニホイールで
```

1. シミュレーターの **「自動操縦」タブ**を開きます(Nav2 の起動に 10 秒くらいかかります。「待機中」になれば OK)。
2. **「地図をクリックしてゴールを置く」** を押して、地図をクリックします(ドラッグすると、着いたときの向き)。
3. 紫の線(探した道)が出て、ロボットが走り出します。着くと「着きました」になります。

- 途中で **「自動操縦をやめる」** を押すか、**WASD で手で操縦する**か、**停止ボタン**を押すと、自動操縦はやめます。
- シミュレーターの「障害物を置く」で道をふさぐと、LiDAR で見つけて、よける道を探し直します。
- `./sim.sh amcl` で「タイヤの滑り」を入れると、オドメトリはずれていきますが、AMCL が LiDAR と地図を見比べて位置を直すので、ゴールに着けます(`./sim.sh nav` だと直しません)。

## 実機で

今は、**Nav2 は PC で動かします**(Nav2 一式は大きいので、Pi のイメージには入れていません)。LiDAR の `/scan` は Wi-Fi で PC に届きます。

```bash
# Pi: 足回りを起動しておく(cmd_vel_mux も一緒に起動する)
docker compose --profile robot up -d drive
# PC: ロボットコンソール + 自動操縦(地図のノードと AMCL も起動する)
./console.sh nav:=true field:=example
```

1. 「自動操縦」タブの **「地図をクリックして、今いる位置を教える」** で、ロボットの今の位置と向きをクリックします。LiDAR の点(赤)が地図の壁に重なれば、位置が合っています。
2. ゴールをクリックします。

**必ず、すぐ止められるようにしてから試します**(停止ボタンと、本物の非常停止スイッチ)。最初は `vx_max` などを小さくします。

## しくみ

```
 地図(field_map)── /map ───────────────┐
                 └ /lidar_map ─▶ AMCL ─┤ TF map → odom(自分の位置)
 LiDAR ─── /scan ──────────────▶ AMCL ─┤
                                        ▼
 コンソール ── ゴール ─▶ bt_navigator ─▶ planner_server(道を探す。/plan)
 (navigate_to_pose)                   └▶ controller_server(道に沿って走る)── /cmd_vel_nav ─┐
                                                                                             ▼
 コンソール(WASD)/ ゲームパッド ─────────────────────────────── /cmd_vel_teleop ─▶ cmd_vel_mux ─▶ /cmd_vel ─▶ 足回り
                                                                          (手で操縦する方が優先)
```

| 部品 | 何をするか |
|---|---|
| AMCL | LiDAR の点と地図(`/lidar_map`。LiDAR に映る物だけ)を重ねて、自分の位置を合わせる。TF の `map → odom` を出す |
| コストマップ | 地図(`/map`)と LiDAR で見えた物を、ロボットの半径だけ太らせて塗ったもの。壁から遠いほど通りやすい |
| planner_server | コストマップの上で、ゴールまでの一番短い道を探す(A*)。1 秒ごとに探し直す |
| controller_server | 道に沿って走る速さを、1 秒に 20 回計算する(MPPI。横にも動ける足回り用の設定) |
| behavior_server | 行き詰まったときに、その場で回る・少し下がる・待つ |
| bt_navigator | 上の部品を順番に呼ぶ司令塔。ゴールを受け取る(`navigate_to_pose`) |
| cmd_vel_mux | 手で操縦する指令と自動操縦の指令をまとめて `/cmd_vel` に出す(`swerve_drive` にある。足回りと一緒に起動) |

## 設定(`config/nav2_params.yaml`)

まず直すところには ★ が付いています。

| 設定 | 意味 |
|---|---|
| `robot_radius` | ロボットの半径 [m](真上から見て、中心からいちばん遠い角まで)。2 か所 |
| `inflation_radius` | 壁からこれだけ離れた道を好む [m]。`robot_radius` より大きく。2 か所 |
| `vx_max` / `vy_max` / `wz_max` | 自動操縦の速さの上限。足回りが出せる速さ以下に |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | これだけ近づいたら「着いた」 |
| `alpha1`〜`alpha5`(amcl) | オドメトリのずれの大きさ。タイヤがよく滑るなら大きくする |

変えたら `colcon build` し直します(`./sim.sh` は自動でやります)。設定の書きまちがいは `test/test_params.py` で見つけられます。

## これから

- Nav2 を Pi で動かす(Pi のイメージに Nav2 を入れる。`docker/Dockerfile.pi` の `SKIP_KEYS` から外す)
- 何か所かを順番に回る(`navigate_through_poses`)、決まった動きをボタンで呼ぶ
- IMU を足して、オドメトリを良くする(robot_localization)
