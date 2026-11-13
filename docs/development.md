# 開発の進め方

コードを変えて、チームに共有するまでの流れです。

## 1 回の作業の流れ

```bash
cd ~/e2igmq_test
git checkout main && git pull origin main        # 最新の main から始める
git checkout -b 名前/やること                     # 例: tanaka/arm-control。作業用のブランチを作る

# …… ファイルを編集して、sim や実機で試す ……

docker compose run --rm build                     # テストを全部動かす(CI と同じ)
git add 変えたファイル
git commit -m "何をしたか(日本語で OK)"
git push -u origin 名前/やること
```

push したら GitHub で **Pull Request(PR)** を作ります。

1. CI(自動テスト)が緑 ✅ になるのを待つ。赤 ❌ ならログを見て直し、もう一度 push する
2. 誰かに見てもらう(できれば)
3. **Merge** で main に入れる

**main に直接 push しない**でください。壊れたコードが main に入ると、全員の `git pull` で壊れます。

## CI(自動テスト)で何をしているか

`.github/workflows/docker-ros2.yml` で、push と PR のたびに次の 3 つが動きます。

| ジョブ | やること | 手元で同じことをするには |
|---|---|---|
| amd64 | PC 用のイメージを作り、`colcon build` と `colcon test`(ROS 2 のテストとコードの書き方チェック) | `docker compose run --rm build` |
| firmware | STM32 のコードを PC でテストし、全部の組み合わせ(`APP=` / `LINK=uart` など)でビルドする | `ros make -C /root/firmware/stm32_can/test` |
| arm64 | Pi 用のイメージが作れるか確かめる | (重いので CI に任せてよい) |

### Python の書き方のルール(CI で止まりやすいところ)

`colcon test` で flake8 と pep257 のチェックが入ります。ひっかかりやすいのは次のとおりです。

- 文字列は**シングルクォート** `'...'`。ただし、複数行の文字列(docstring)は **`"""..."""`**(`'''` はエラー)
- 1 行は **99 文字まで**
- import は「標準ライブラリ → ほかのライブラリ → 自分のパッケージ」の順で、それぞれアルファベット順
- 関数やクラスの説明(docstring)の 1 行目は `.` で終わる

エラーが出たら、表示されたファイル名と行番号の場所を直します。

## テストの書き方

ROS 2 に依存しない計算(運動学、設定ファイルの読み書きなど)は、ROS 2 から切り離したファイルに書き、`test/test_*.py` に pytest のテストを書きます。例:`ros2_ws/src/swerve_drive/swerve_drive/kinematics.py` と `test/test_kinematics.py`。

```bash
ros   # コンテナに入る
cd ~/ros2_ws && colcon test --packages-select swerve_drive && colcon test-result --verbose
```

STM32 の部品(`firmware/stm32_can/`)は、HAL の代わりの部品(`test/hal_stub`)を使って PC でテストできます(`test/test_main.c`)。

## 新しいものを足すとき

### ROS 2 のパッケージを足す

```bash
ros
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python パッケージ名 --dependencies rclpy std_msgs
```

- コマンド(ノード)を足したら、`setup.py` の `entry_points` に書き、`colcon build` をし直します。
- 足回りに速度を送りたいときは、**`/cmd_vel` を publish する**だけです(キーボード・ゲームパッド・自動操縦と同じ入口)。足回りのコードを変える必要はありません。
- 状態を知らせたいときは `/diagnostics` に出すと、ロボットコンソールに表示されます(`swerve_drive/robot_status.py` が例)。
- `apt` のパッケージが新しく必要なら、`package.xml` に書いて `docker compose build dev` をし直します(Pi 用は `docker/Dockerfile.pi` も確認)。

### STM32 のプログラムを足す(`APP=`)

`firmware/motor_node_f303k8/` では、`make APP=名前` で `Core/Src/app_名前.c` が使われます(例:`app_led.c`、`app_swerve.c`)。新しい動きを作るときは、

1. 似ている `app_*.c` をコピーして `app_名前.c` を作る
2. `make APP=名前` でビルドできるか確かめる
3. CI のビルドの一覧(`.github/workflows/docker-ros2.yml`)にも足す

通信(`stm<ID>/command` と `stm<ID>/state`)はそのまま使えます。

### 通信で送れる量の決まり

今の通信は、1 つの STM32 につき **小数 2 個まで**(CAN の 8 バイト = 4 バイト × 2)です。3 個以上送りたいときは、

- STM32 を分ける(ID を増やす)
- 値を小さい型(2 バイトの整数など)に詰める
- 通信の決まり(`firmware/stm32_can/can_node.c` と `ros2_ws/src/stm_bridge/`)を両方そろえて変える

のどれかにします。決まりの中身は [stm_bridge.md](stm_bridge.md) を見てください。

## 設定を変えるとき

| 変えたいもの | 場所 | 反映のしかた |
|---|---|---|
| ギア比、タイヤの大きさ、モジュールの位置 | 設定画面(`ros ros2 run swerve_drive drive_setup`) | STM32 は書き込み直す、Pi は再起動 |
| 足回りの細かい設定 | `ros2_ws/src/swerve_drive/config/swerve.yaml`、`omni_drive/config/omni.yaml` | ノードを起動し直す |
| PID のゲインなど STM32 の設定 | `firmware/motor_node_f303k8/Core/Inc/app_config.h` | 書き込み直す(`./sim.sh fw` で先に試せる) |
| STM32 の ID や通信の周期 | `ros2_ws/src/stm_bridge/config/bridge.yaml` | ノードを起動し直す |

Pi のイメージには `ros2_ws` が焼き込まれているので、Pi で使う設定を変えたら**イメージを作り直して送り直します**([raspberry_pi.md](raspberry_pi.md))。
