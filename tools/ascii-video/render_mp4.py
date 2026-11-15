#!/usr/bin/env python3
"""
render_mp4.py - asciiplay.py で作った .asc.gz を、文字アートのMP4に書き出す

使い方:
  python3 render_mp4.py MV_ascii.asc.gz -a MV_ascii.m4a -o MV_ascii_video.mp4
  python3 render_mp4.py in.asc.gz -a in.m4a -o out.mp4 --bitrate 8M
  python3 render_mp4.py in.asc.gz -o out.mp4 --crf 20            # 高画質(ファイル大)

必要: numpy, Pillow, ffmpeg, 等幅フォント(既定 DejaVu Sans Mono)
"""
import argparse
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import asciiplay  # noqa: E402

FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",   # Debian/Ubuntu
    "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",  # Fedora
    "C:/Windows/Fonts/consola.ttf",                           # Windows
]


def find_font(path):
    if path:
        return path
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            return p
    sys.exit("等幅フォントが見つかりません。--font で指定してください")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help=".asc.gz (白黒のみ対応)")
    ap.add_argument("-a", "--audio", help="音声ファイル (.m4a など)")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--font-size", type=int, default=26, help="1080p/120文字なら26")
    ap.add_argument("--font")
    ap.add_argument("--bitrate", default="8M", help="例 8M。--crf を指定すると無視")
    ap.add_argument("--crf", type=int)
    ap.add_argument("--preset", default="veryfast")
    a = ap.parse_args()

    meta, frames = asciiplay.read_asc(a.src)
    if meta.get("color"):
        sys.exit("カラーの .asc.gz には未対応です (白黒で変換してください)")
    first = frames[0].split("\n")
    cols, nrows = len(first[0]), len(first)
    W, H = (int(v) for v in a.size.split("x"))
    cw = W // cols
    ch = min(H // nrows, cw * 2)
    font = ImageFont.truetype(find_font(a.font), a.font_size)

    # 文字ごとのタイル画像を先に作っておき、numpyで並べて高速に描画する
    chars = sorted((set("".join(frames[::50])) | set(asciiplay.RAMP)) - {"\n"})
    lut = {c: i for i, c in enumerate(chars)}
    atlas = np.zeros((len(chars), ch, cw), np.uint8)
    for i, c in enumerate(chars):
        im = Image.new("L", (cw, ch), 0)
        d = ImageDraw.Draw(im)
        l, t, r, b = d.textbbox((0, 0), c, font=font)
        d.text(((cw - (r - l)) / 2 - l, (ch - (b - t)) / 2 - t), c, font=font, fill=255)
        atlas[i] = np.array(im)

    cmd = ["ffmpeg", "-y", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "gray", "-s", f"{W}x{H}",
           "-r", str(meta["fps"]), "-i", "-"]
    if a.audio:
        cmd += ["-i", a.audio, "-map", "0:v", "-map", "1:a", "-c:a", "aac", "-b:a", "128k", "-shortest"]
    cmd += ["-c:v", "libx264", "-preset", a.preset, "-pix_fmt", "yuv420p"]
    if a.crf is not None:
        cmd += ["-crf", str(a.crf)]
    else:
        cmd += ["-b:v", a.bitrate, "-maxrate", a.bitrate, "-bufsize", a.bitrate]
    cmd += ["-movflags", "+faststart", a.output]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    ox, oy = (W - cw * cols) // 2, (H - ch * nrows) // 2
    canvas = np.zeros((H, W), np.uint8)
    for n, f in enumerate(frames):
        lines = f.split("\n")[:nrows]
        idx = np.array([[lut.get(c, 0) for c in ln.ljust(cols)[:cols]] for ln in lines])
        img = atlas[idx].transpose(0, 2, 1, 3).reshape(len(lines) * ch, cols * cw)
        canvas[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img
        ff.stdin.write(canvas.tobytes())
        if n % 200 == 0:
            sys.stderr.write("\r書き出し中 %d/%d" % (n, len(frames)))
    ff.stdin.close()
    ff.wait()
    sys.stderr.write("\r完了 → %s\n" % a.output)
    sys.exit(ff.returncode)


if __name__ == "__main__":
    main()
