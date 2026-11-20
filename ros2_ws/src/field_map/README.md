# field_map — フィールドの地図

大会のフィールドの形(壁・台・柱など)を **YAML のファイルに書く**と、自動操縦(Nav2)で使える地図(`/map`)を作るパッケージです。

フィールドは図面で決まっているので、LiDAR で走り回って地図を作る(SLAM)必要がありません。
フィールドが変わったら、**ファイルを書き換えて保存するだけ**で、地図も作り直されます。

ほかの高専の [gn10-pointcloud-localization](https://github.com/tmcit-ararobo-2026a/gn10-pointcloud-localization)(Apache-2.0)の、フィールドを JSON で書く方法を参考にしました。その JSON もそのまま読めます。

## 使い方

```bash
ros ros2 launch field_map field_map.launch.py                    # fields/example.yaml の地図を rviz2 で見る
ros ros2 launch field_map field_map.launch.py field:=my_field    # fields/my_field.yaml
ros ros2 run field_map make_map example                          # 地図をターミナルに文字で出して確かめる
```

rviz2 を開いたまま `fields/example.yaml` を書き換えて保存すると、1 秒以内に地図が変わります。

## 画面で編集する(フィールドエディタ)

```bash
ros ros2 run field_map field_editor              # fields/example.yaml を開く
ros ros2 run field_map field_editor my_field     # fields/my_field.yaml(なければ新しく作る)
```

- **描く**:上の「箱(B)」「円(C)」「多角形(P)」を選んで、図の上でドラッグ / クリックします(多角形はダブルクリックか Enter で終わり)。
- **直す**:「選択(V)」でクリックして選び、ドラッグで動かします。白い四角の点で大きさや形、紫の丸い点で回転を変えます。右の「図形」の欄で、名前・位置・大きさ・回転・高さを数字で直せます。
- **グリッド**:上の「グリッド」で 1 cm / 5 cm / 10 cm に吸着します(回転は 5° ずつ)。
- **元に戻す**:Ctrl+Z(やり直しは Ctrl+Y)。複製は Ctrl+D、消すのは Delete。
- **保存**:Ctrl+S。地図のノード(`field_map_node`)が動いていれば、rviz2 の地図もすぐ変わります。
- **色**:青 = 通れない・LiDAR に映る、オレンジの斜線 = 通れないが LiDAR に映らない、紫 = LiDAR に映るが下をくぐれる、灰色の点線 = 通れる。

### フィールドの図をなぞる

右の「画像」の欄で、ルールブックのフィールドの図などを下に敷けます。

1. **画像を開く**(PNG / JPG / PGM。Nav2 の地図の `.yaml` なら、縮尺と位置も自動で合う)
2. **2 つの点で位置を合わせる**:図の上で、座標が分かる点(フィールドの角など)を 2 か所クリックして、それぞれの座標 [m] を入れる。これで縮尺・位置・向きが合う
3. 「濃さ」で画像を薄くして、上から箱や多角形でなぞる

画像の場所と位置合わせは、フィールドのファイル(`field: background:`)に保存されるので、次に開いたときもそのまま出ます。地図のノードは、この設定を使いません。

### 画像から図形を自動で作る

「画像」の欄の「図形を作る」で、フィールドの範囲の中の **暗いところ**(または **選んだ色に近いところ**)を図形にします。

- 長方形に近い物は箱、円に近い物は円、それ以外は多角形になります。外枠のような中が空いた線は、中を埋めずに細い箱に分けます。
- 小さい物(文字やごみ)は捨てます(「これより小さい物は捨てる」で調整)。
- 色で選ぶときは「画像から拾う」で図の上をクリックすると、その色になります。
- 作った図形は名前が「画像1」「画像2」… になるので、要らない物を消し、高さを付けて仕上げます。

保存すると、YAML の中のコメントは、決まった説明だけに書き直されます(図形の名前は残ります)。

## フィールドのファイルの書き方(`fields/*.yaml`)

`fields/example.yaml` をコピーして書き換えるのがいちばん簡単です。

```yaml
field:
  name: 2027 年のフィールド
  resolution: 0.05          # 1 マスの大きさ [m]
  x_range: [-3.0, 3.0]      # x の範囲 [m](x が前 = 図の右)
  y_range: [-2.0, 2.0]      # y の範囲 [m](y が左 = 図の上)
  border: 0.05              # まわりに付ける壁の厚さ [m](0 なら付けない)

shapes:                     # ふさがっている場所
  - {type: box, name: 台, x: -1.2, y: 0.5, size_x: 0.6, size_y: 0.4}
  - {type: box, name: 斜めの板, x: 0.0, y: -1.0, size_x: 1.0, size_y: 0.02, yaw_deg: 30}
  - {type: circle, name: 柱, x: 1.5, y: 0.6, radius: 0.15}
  - {type: polygon, name: 三角の台, points: [[2.2, -1.9], [2.9, -1.9], [2.9, -1.2]]}
```

| 図形 | 書くもの |
|---|---|
| `box` | 中心 `x`、`y` と大きさ `size_x`、`size_y` [m]。`yaw_deg` で回せる(反時計回りが正) |
| `circle` | 中心 `x`、`y` と半径 `radius` [m] |
| `polygon` | 角の位置 `points: [[x, y], ...]`(3 個以上、順番に) |

- 床のテープなど、通れる物は書かないか、`passable: true` を付けます(`step_height` 以下の高さにしてもよい)。
- マスより細い壁(2 cm の板など)も、ぬけずに地図に入ります。
- 新しいファイルを `fields/` に作ったら、`field:=ファイルの名前` で使えます(dev コンテナでは colcon build しなくても見つかります)。
- 書き方をまちがえたときは、前の地図のまま、ロボットコンソールの「状態」とログに理由が出ます。

## 高さ(ボックス・台・棚など)

図形ごとに高さを書くと、地図を 2 枚に分けます。2D LiDAR は決まった高さの面しか見ないので、低いボックスや上の棚は LiDAR に映りません。映らない物まで位置合わせに使うと、位置がずれる原因になります。

```yaml
field:
  lidar_height: 0.20        # 2D LiDAR の高さ [m](複数なら [0.2, 0.5])
  robot_height: 0.60        # ロボットの高さ [m]。これより上の物の下はくぐれる
  step_height: 0.01         # 乗り越えられる高さ [m]。これ以下の物は通れる

shapes:
  - {type: box, name: 低いボックス, x: -0.2, y: 1.2, size_x: 0.4, size_y: 0.3, height: 0.10}
  - {type: box, name: 上の棚, x: 0.9, y: -0.2, size_x: 0.5, size_y: 0.5, z_min: 0.7, z_max: 0.75}
```

| 書き方 | 意味 |
|---|---|
| `height: 0.3` | 床から 0.3 m の高さの物 |
| `z_min: 0.7, z_max: 0.9` | 浮いている物(棚の板など)。下の高さと上の高さ |
| 書かない | 床から天井まで(壁・柱など) |

| 地図 | 入る物 | 使うところ |
|---|---|---|
| `/map`(通れるか用) | ロボットの高さ(`step_height`〜`robot_height`)にかかる物 | 経路計画、rviz2 |
| `/lidar_map`(位置合わせ用) | LiDAR の高さ(`lidar_height`)にかかる物だけ | AMCL(`map_topic: lidar_map`) |

- `lidar_height` などを書かなければ、今までどおり、全部の図形が両方の地図に入ります。
- `make_map` の表示では、`#` が通れない・LiDAR に映る、`+` が通れないが LiDAR に映らない、`^` が LiDAR に映るが下をくぐれる物です。
- gn10 の JSON の `z_min` / `z_max` も、そのまま使えます。

## 出すもの

| トピック | 型 | 中身 |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 通れるか用の地図(0 = 通れる、100 = ふさがっている)。あとから起動したノードにも届く |
| `/lidar_map` | `nav_msgs/OccupancyGrid` | 位置合わせ用の地図(LiDAR に映る物だけ)。rviz2 では左の一覧のチェックで表示 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 地図を作れたか(「フィールドの地図」) |

## シミュレーション用の LiDAR(`fake_lidar`)

位置合わせ用の地図(`/lidar_map`)とロボットの位置から、2D LiDAR が見るはずの距離を計算して `/scan`(`sensor_msgs/LaserScan`)に出します。実機の LiDAR がなくても、ロボットコンソールや rviz2 で LiDAR の見え方を確かめられ、あとで Nav2(AMCL)を試すときにも使えます。

- `./sim.sh` で自動で起動します(`sim_world.launch.py`: 地図のノード + `fake_lidar` + `map → odom` の TF)。
- 1° おきに 360 本、12 m まで、1 秒に 10 回。距離に少しばらつき(1 cm)を足します。
- LiDAR の取り付け位置は `laser_x` / `laser_y` / `laser_yaw_deg`(`base_link → laser` の TF も出す)。
- 計算は `field_map/lidar_sim.py`(ROS に依存しない。`test/test_lidar_sim.py`)。

## Nav2 の map_server で使うとき

`field_map_node` の代わりに Nav2 の `map_server` を使うときは、画像と設定のファイルを書き出します。

```bash
ros ros2 run field_map make_map example -o /root/ros2_ws/maps/example   # example.pgm / .yaml と example_lidar.pgm / .yaml
```

## ファイル

| ファイル | 中身 |
|---|---|
| `field_map/field.py` | ファイルの読み込みと、地図(マス目)づくり。ROS に依存しない(`test/test_field.py`) |
| `field_map/field_map_node.py` | 地図を `/map` に出す。ファイルが変わったら作り直す |
| `field_map/make_map.py` | 地図をターミナルに出す / 画像に書き出すコマンド |
| `field_map/field_editor.py` | フィールドエディタ(画面。PC 用) |
| `field_map/fake_lidar.py` / `lidar_sim.py` | シミュレーション用の LiDAR(`/scan`)と、その計算 |
| `launch/sim_world.launch.py` | シミュレーションの世界(地図 + LiDAR + `map → odom`)。`./sim.sh` が読み込む |
| `field_map/field_io.py` | エディタの読み書き、画像の位置合わせ、画像から図形を作る計算(`test/test_field_io.py`) |
| `fields/` | フィールドのファイル(`example.yaml` は例。本物のフィールドに合わせて書き直す) |

## これから

- 自動操縦(Nav2)を足すときに、この `/map` を AMCL(LiDAR で自分の位置を合わせる)と経路計画に使います。
- 赤・青のフィールドで開始位置を切り替える設定は、そのときに足します。
