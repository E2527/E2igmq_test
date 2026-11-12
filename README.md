# ロボコン ROS 2 リポジトリ

ロボコンのロボットを動かすためのプログラム一式です。PC と Raspberry Pi で動く **ROS 2(Jazzy)** と、モーターを回す **STM32**(Nucleo-F303K8)のプログラムが入っています。

- 開発環境は **Docker** で作るので、誰の PC でも同じ環境になります(WSL でも Linux でも)。
- 実機がなくても、PC の**シミュレーション**で足回りを動かして試せます。
- GitHub に push すると、**CI**(自動テスト)が毎回動いて、壊れていないか確かめます。

初めての人は、下の「[はじめての人へ](#はじめての人へ)」から読んでください。

---

## 全体の構成

```
 【PC】                          【Raspberry Pi(ロボットに載せる)】          【STM32 × 4】
 ロボットコンソール(操縦・状態)    足回りの計算(swerve_drive / omni_drive)      モーターの速度制御
 ゲームパッド                       ↓                                           (エンコーダ + PID)
   │                               stm_bridge ── CAN / USB シリアル ─────────▶ スワーブなら
   └── /cmd_vel(Wi-Fi)──────────▶   ↑                                       向きの制御も
                                     └──────── 実際の速さ・向き ◀─────────────┘
```

| 担当 | 何をするか | 言語 | 場所 |
|---|---|---|---|
| **PC** | 操縦する、ロボットの状態を見る、シミュレーション | Python(ROS 2) | `ros2_ws/src/swerve_drive/`(ロボットコンソール) |
| **Raspberry Pi** | 「どう動くか」を決める(足回りの計算、オドメトリ) | Python(ROS 2) | `ros2_ws/src/` |
| **STM32** | 決められたとおりにモーターを回す(1 ms ごと) | C | `firmware/` |

PC と Pi は同じ ROS 2 なので、**Pi がなくても PC だけで全部動かせます**(シミュレーション)。

---

## はじめての人へ

次の順番で進めてください。

| 順番 | やること | 読むもの |
|---|---|---|
| 1 | 言葉の意味をざっと知る(ROS 2、トピック、Docker など) | [docs/glossary.md](docs/glossary.md)(用語集) |
| 2 | 自分の PC に開発環境を作る(初回 30 分〜1 時間) | [docs/setup.md](docs/setup.md) |
| 3 | シミュレーションでロボットを動かしてみる | 下の「[いつもの使い方](#いつもの使い方)」 |
| 4 | 足回りのしくみを知る | [ros2_ws/src/swerve_drive/README.md](ros2_ws/src/swerve_drive/README.md) |
| 5 | STM32 に書き込んで実機を動かす | [firmware/motor_node_f303k8/README.md](firmware/motor_node_f303k8/README.md) |
| 6 | コードを変えて、チームに共有する | [docs/development.md](docs/development.md) |

困ったときは [docs/troubleshooting.md](docs/troubleshooting.md)(よくあるトラブルと直し方)を見てください。

---

## いつもの使い方

セットアップ([docs/setup.md](docs/setup.md))が終わっていれば、**PC のターミナルで次を打つだけ**です。

```bash
cd ~/e2igmq_test
git pull origin main      # 最新にする
./sim.sh                  # シミュレーション + ロボットコンソールが開く
```

ロボットコンソールのウィンドウをクリックして、**W A S D** で動かします。止めるときは、ターミナルで **Ctrl + C** です。

### よく使うコマンド

| やりたいこと | コマンド(PC のターミナルで) |
|---|---|
| シミュレーション(スワーブ) | `./sim.sh` |
| シミュレーション(オムニ) | `./sim.sh omni` |
| STM32 のプログラムまで含めたシミュレーション | `./sim.sh fw` |
| rviz2 も開く / ゲームパッドでも操縦 | 後ろに `rviz` / `joy` を付ける(例:`./sim.sh fw rviz`) |
| ギア比と寸法の設定画面 | `ros ros2 run swerve_drive drive_setup` |
| コンテナ(開発環境)の中に入る | `ros` |
| STM32 に書き込む | `ros make -C /root/firmware/motor_node_f303k8 APP=swerve NODE_ID=1 flash` |
| テストを全部動かす | `docker compose run --rm build` |

`ros` は `./dev.sh` の短い名前です(セットアップで登録します)。**コンテナの中**(表示が `root@...#`)では、`ros` を付けずに `ros2 ...` や `make ...` を打ちます。

---

## フォルダの地図

```
e2igmq_test/
├── README.md              … このファイル(入口)
├── docs/                  … 詳しい説明(セットアップ、Pi、困ったとき、用語集など)
├── dev.sh                 … 開発環境(コンテナ)に入る(ros)
├── sim.sh                 … シミュレーションを起動する
├── compose.yaml           … Docker のコンテナの設定(PC 用の dev、Pi 用の drive など)
├── docker/                … Docker のイメージの作り方(PC 用 / Pi 用)
├── ros2_ws/src/           … ROS 2 のパッケージ(Python)
│   ├── swerve_drive/      …   スワーブの足回り + ロボットコンソール + 設定画面
│   ├── omni_drive/        …   オムニホイールの足回り(スワーブの保険)
│   ├── stm_bridge/        …   ROS 2 と STM32 をつなぐ(CAN / USB シリアル)
│   └── my_pkg/ my_cpp_pkg/ …  練習用のサンプル
├── firmware/              … STM32 のプログラム(C)
│   ├── stm32_can/         …   部品(通信、速度制御、スワーブの計算)と PC でのテスト
│   ├── motor_node_f303k8/ …   Nucleo-F303K8 用のプロジェクト(make でビルド・書き込み)
│   ├── sim/               …   STM32 のプログラムを PC のシミュレーションで動かす部分
│   └── lib/               …   ST の HAL / CMSIS(git サブモジュール)
└── .github/workflows/     … CI(push のたびに動く自動テスト)
```

---

## ドキュメント一覧

| ファイル | 中身 |
|---|---|
| [docs/glossary.md](docs/glossary.md) | 用語集(ROS 2、Docker、STM32、制御の言葉) |
| [docs/setup.md](docs/setup.md) | 開発環境のセットアップ(Windows + WSL / Linux)、Docker の使い方 |
| [docs/development.md](docs/development.md) | 開発の進め方(ブランチ、PR、テスト、新しい機能の足し方) |
| [docs/raspberry_pi.md](docs/raspberry_pi.md) | Raspberry Pi で動かす(イメージの作り方、送り方、Wi-Fi での操縦) |
| [docs/stm_bridge.md](docs/stm_bridge.md) | ROS 2 と STM32 の通信(CAN / USB シリアル、通信の決まり) |
| [docs/troubleshooting.md](docs/troubleshooting.md) | 困ったとき(よくあるエラーと直し方) |
| [ros2_ws/src/swerve_drive/README.md](ros2_ws/src/swerve_drive/README.md) | スワーブの足回り、ロボットコンソール、設定画面、シミュレーション |
| [ros2_ws/src/omni_drive/README.md](ros2_ws/src/omni_drive/README.md) | オムニホイールの足回り |
| [firmware/motor_node_f303k8/README.md](firmware/motor_node_f303k8/README.md) | STM32 のビルド・書き込み・ピン配置・初めて動かす手順 |
| [firmware/stm32_can/README.md](firmware/stm32_can/README.md) | STM32 の部品の中身と、PID の調整のしかた |

---

## 今の状況

| 項目 | 状況 |
|---|---|
| 開発環境(WSL / Fedora)、シミュレーション、ロボットコンソール | ✅ 動作確認済み |
| STM32 への書き込み、USB シリアルでの通信 | ✅ 実機で確認済み |
| スワーブ・オムニの足回り(Pi 側の計算) | ✅ シミュレーションで確認済み |
| スワーブのファームウェア(`APP=swerve`)を実機のモーターで | ⬜ まだ |
| Raspberry Pi で動かす、Wi-Fi での操縦 | ⬜ まだ |
| CAN での通信 | ⬜ まだ(先に [クロックの注意](firmware/motor_node_f303k8/README.md#can-を実機で使う前にクロックの精度) を読む) |
| 自動操縦(Nav2 など) | ⬜ 必要になったら作る |
