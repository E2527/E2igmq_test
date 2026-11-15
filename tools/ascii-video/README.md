# ascii-video — 動画をASCIIアートにする (ターミナル再生 / MP4書き出し)

Claude Code への引き継ぎ用メモ。claude.ai のチャットで作ったものをまとめたもの。

## 目的
自作MV (`MV_完成版.mp4`) を「Bad Apple!! ASCII」のような文字アート動画にする。
1. ターミナルで再生する (音声つき)
2. 文字アートの見た目のまま MP4 に書き出す

## ファイル
| ファイル | 役割 |
|---|---|
| `asciiplay.py` | 動画 → 文字フレーム(`.asc.gz`)+音声(`.m4a`) への変換と、ターミナル再生 |
| `render_mp4.py` | `.asc.gz` を文字アートのMP4に書き出す |

## 依存
- 変換: Python 3, `opencv-python`, `numpy`, `ffmpeg`
- ターミナル再生: Python標準ライブラリのみ (音声は `ffplay` があれば鳴る)
- MP4書き出し: `numpy`, `Pillow`, `ffmpeg`, 等幅フォント (DejaVu Sans Mono 既定)
- Fedora: `sudo dnf install ffmpeg` (RPM Fusion 必要), `pip install opencv-python pillow numpy`

## 使い方
```bash
# 1) 変換 (横120文字・白黒が既定)
python3 asciiplay.py convert MV_完成版.mp4 -o MV_ascii.asc.gz
#   -> MV_ascii.asc.gz と MV_ascii.m4a ができる
#   オプション: -w 横文字数 / --fps 間引き / --color カラー / --invert 明暗反転 / --no-audio

# 2) ターミナル再生 (端末幅は横文字数以上にする)
python3 asciiplay.py play MV_ascii.asc.gz          # --mute で無音, --speed で速度
python3 asciiplay.py live input.mp4                # 変換しながら直接再生

# 3) MP4書き出し
python3 render_mp4.py MV_ascii.asc.gz -a MV_ascii.m4a -o MV_ascii_video.mp4 --bitrate 8M
```

## 仕組み
- **変換**: 各フレームを横W文字に縮小 (文字が縦長なので高さは ×0.5)、グレースケール化し、
  明るさを70段階の文字列 `RAMP`(暗→明)に割り当てる。
- **`.asc.gz` 形式**: gzip テキスト。1行目がJSONメタ `{"fps":24.0,"color":false}`、
  以降はフレーム本文を `\f` (フォームフィード) で区切って連結。カラー時は行内に ANSI 24bit色エスケープ。
- **再生**: `ESC[H` でカーソルを左上に戻して上書き描画。時刻基準で待ち、遅れたフレームは飛ばして音声と同期。
  Windows は `os.system("")` で ANSI を有効化。
- **MP4書き出し**: 使用文字ごとのタイル画像 (1080p/120文字なら 16×32px) を先に作り、
  numpy でタイルを並べてフレームを生成 → 生のグレースケール映像を ffmpeg (libx264) にパイプ。

## これまでの実績 (MV_完成版.mp4)
- 元動画: 1920×1080, 24fps, 4分8秒 (247.8秒), 約356MB
- 変換: 横120×縦33文字, 5945フレーム, `.asc.gz` 約10.6MB, 変換約1分半
- MP4: 1920×1080, フォント26pt, 8Mbps → **約246MB**, 画質良好
  - crf 20 だと約814MB。文字が毎フレーム変わるので普通の動画より圧縮が効きにくい
  - 5Mbps だと文字がにじむ。8Mbps前後が妥協点

## 今後の案 (未着手)
- 80文字版 (T480など小さい画面向け)
- カラー版 (`--color`)、MP4書き出しのカラー対応 (`render_mp4.py` は現状白黒のみ)
- 点字文字 (⣿) による高解像度化
- MP4の軽量化 (100MB前後。H.265 や 720p 化を検討)

---

## ロボットコンソールの隠し機能で流す(このリポジトリで足した部分)

変換した `.asc.gz` を `ros2_ws/egg/` に置くと、ロボットコンソール(`./sim.sh`)の中から流せます。

```bash
# 1) 変換する(ホストで。opencv-python と ffmpeg が必要)
python3 tools/ascii-video/asciiplay.py convert MV_完成版.mp4 -o ros2_ws/egg/MV_ascii.asc.gz
#   -> ros2_ws/egg/MV_ascii.asc.gz と ros2_ws/egg/MV_ascii.m4a ができる

# 2) コンソールを起動する
./sim.sh
```

開き方(どちらでも):

- ロボットコンソールのウィンドウで、矢印キーを **↑ ↑ ↓ ↓ ← → ← →** の順に押す
- 左上の四角いマークを、2 秒の間に **5 回クリック**する

別のウィンドウが開いて、動画が流れます。閉じるには **Esc**、**Q**、またはクリック。最後まで流れると自動で閉じます。

- `.asc.gz` を置いていないときは、代わりに短い模様が流れます。
- カラーで変換したファイルは、白黒として流れます。
- **音**:同じ名前の `.m4a` があれば、`ffplay` で鳴らします(dev イメージに入っています。WSL は WSLg、Fedora などは PipeWire を通して PC のスピーカーから出ます)。音が出ないときは [docs/troubleshooting.md](../../docs/troubleshooting.md) を見てください。
- **安全**:隠し機能のウィンドウを開くと、コンソールは押していたキーを離したことにするので、ロボットは止まります。矢印キーは操縦に使っていないので、コマンドを打ってもロボットは動きません。
- 別のファイルを流したいときは、環境変数 `ROBOT_CONSOLE_EGG` にファイルの場所を入れます。
- 中身は `ros2_ws/src/swerve_drive/swerve_drive/ascii_video.py`(データ)と `ascii_player.py`(ウィンドウ)です。

**動画のファイルを git に入れるかどうか**:4 分・横 120 文字だと `.asc.gz` が約 10 MB、`.m4a` が数 MB になります。git に入れると全員の clone が重くなるので、入れるなら横 80 文字・15 fps(`-w 80 --fps 15`)などで小さくするのがおすすめです。
