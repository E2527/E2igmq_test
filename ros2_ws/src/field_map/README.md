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
| `fields/` | フィールドのファイル(`example.yaml` は例。本物のフィールドに合わせて書き直す) |

## これから

- 自動操縦(Nav2)を足すときに、この `/map` を AMCL(LiDAR で自分の位置を合わせる)と経路計画に使います。
- 赤・青のフィールドで開始位置を切り替える設定は、そのときに足します。
