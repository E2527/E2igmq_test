# egg — ロボットコンソールの隠し機能のデータ置き場

ここに文字アートの動画(`.asc.gz`)を置くと、ロボットコンソールの隠し機能で流れます。
同じ名前の音声(`.m4a`)も置くと、`ffplay` が入っていれば音も鳴ります。

```
ros2_ws/egg/
├── MV_ascii.asc.gz     ← 動画(tools/ascii-video/asciiplay.py の convert で作る)
└── MV_ascii.m4a        ← 音声(なくてもよい)
```

- 今は `MV_ascii`(自作の MV、4 分 7 秒、横 120 文字)が入っています。
- 何も置いていないときは、代わりに短い模様が流れます。
- 複数あるときは、名前の順でいちばん先のものを使います。
- 作り方と開き方は [tools/ascii-video/README.md](../../tools/ascii-video/README.md) を見てください。

(`COLCON_IGNORE` は、colcon がこのフォルダを ROS 2 のパッケージとして探さないようにするためのファイルです)
