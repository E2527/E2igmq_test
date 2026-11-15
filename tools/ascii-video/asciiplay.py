#!/usr/bin/env python3
"""
asciiplay.py - 動画を文字(ASCIIアート)にしてターミナルで再生する

使い方:
  # 1) 変換 (opencv-python と ffmpeg が必要)
  python3 asciiplay.py convert input.mp4              # 白黒, 横120文字
  python3 asciiplay.py convert input.mp4 -w 160 --color
  #   -> input.asc.gz (文字フレーム) と input.m4a (音声) ができる

  # 2) 再生 (Python標準ライブラリだけで動く。音声は ffplay があれば鳴る)
  python3 asciiplay.py play input.asc.gz
  python3 asciiplay.py play input.asc.gz --mute

  # 変換せずに直接再生 (毎回その場で変換するので少し重い)
  python3 asciiplay.py live input.mp4

再生中は Ctrl+C で終了。
"""
import argparse
import gzip
import json
import os
import shutil
import subprocess
import sys
import time

# 暗い→明るい の順 (黒背景ターミナル向け)
RAMP = " .'`^\",:;Il!i><~+_-?][}{1)(|/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$"
SEP = "\f"  # フレーム区切り


# ---------------------------------------------------------------- 変換
def frame_to_text(frame, width, color, invert, cv2, np):
    h, w = frame.shape[:2]
    # 文字は縦長なので高さを約半分にする
    height = max(1, int(h / w * width * 0.5))
    small = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    if invert:
        gray = 255 - gray
    idx = (gray.astype(np.int32) * (len(RAMP) - 1) // 255)
    ramp = np.array(list(RAMP))
    chars = ramp[idx]
    if not color:
        return "\n".join("".join(row) for row in chars)
    lines = []
    for y in range(height):
        parts = []
        prev = None
        for x in range(width):
            b, g, r = (int(v) for v in small[y, x])
            # 色を少し丸めてデータ量を減らす
            c = (r >> 3 << 3, g >> 3 << 3, b >> 3 << 3)
            if c != prev:
                parts.append("\x1b[38;2;%d;%d;%dm" % c)
                prev = c
            parts.append(chars[y, x])
        parts.append("\x1b[0m")
        lines.append("".join(parts))
    return "\n".join(lines)


def open_video(path):
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("変換には opencv-python が必要です:  pip install opencv-python")
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit("動画を開けませんでした: " + path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    return cap, fps, total, cv2, np


def cmd_convert(a):
    cap, fps, total, cv2, np = open_video(a.input)
    step = max(1, round(fps / a.fps)) if a.fps else 1
    out_fps = fps / step
    base = os.path.splitext(a.input)[0]
    out = a.output or base + ".asc.gz"

    n = 0
    i = 0
    with gzip.open(out, "wt", encoding="utf-8", compresslevel=6) as f:
        first = True
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i % step == 0:
                text = frame_to_text(frame, a.width, a.color, a.invert, cv2, np)
                if first:
                    f.write(json.dumps({"fps": out_fps, "color": a.color}) + "\n")
                    first = False
                else:
                    f.write(SEP)
                f.write(text)
                n += 1
                if total:
                    sys.stderr.write("\r変換中 %d%%" % (i * 100 // total))
            i += 1
    cap.release()
    sys.stderr.write("\r%d フレーム → %s\n" % (n, out))

    if not a.no_audio and shutil.which("ffmpeg"):
        audio = os.path.splitext(out[:-3] if out.endswith(".gz") else out)[0] + ".m4a"
        r = subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-i", a.input, "-vn",
             "-c:a", "aac", "-b:a", "128k", audio])
        if r.returncode == 0 and os.path.exists(audio):
            sys.stderr.write("音声 → %s\n" % audio)


# ---------------------------------------------------------------- 再生
def enable_ansi():
    if os.name == "nt":
        os.system("")  # Windows で ANSI エスケープを有効化


def start_audio(path, mute):
    if mute or not path or not os.path.exists(path):
        return None
    player = shutil.which("ffplay")
    if not player:
        sys.stderr.write("(ffplay が無いので音声なしで再生します)\n")
        time.sleep(1)
        return None
    return subprocess.Popen(
        [player, "-nodisp", "-autoexit", "-loglevel", "quiet", path],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def play_frames(frames, fps, audio_path=None, mute=False):
    enable_ansi()
    out = sys.stdout
    out.write("\x1b[?25l\x1b[2J")  # カーソル非表示・画面クリア
    out.flush()
    proc = start_audio(audio_path, mute)
    start = time.perf_counter()
    try:
        for i, text in enumerate(frames):
            target = start + i / fps
            now = time.perf_counter()
            if now > target + 1 / fps:  # 遅れたらフレームを飛ばして音に合わせる
                continue
            if target > now:
                time.sleep(target - now)
            out.write("\x1b[H" + text)
            out.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if proc and proc.poll() is None:
            proc.terminate()
        out.write("\x1b[0m\x1b[?25h\n")
        out.flush()


def read_asc(path):
    with gzip.open(path, "rt", encoding="utf-8") as f:
        meta = json.loads(f.readline())
        data = f.read()
    return meta, data.split(SEP)


def cmd_play(a):
    meta, frames = read_asc(a.file)
    audio = a.audio
    if audio is None:
        stem = a.file[:-3] if a.file.endswith(".gz") else a.file
        audio = os.path.splitext(stem)[0] + ".m4a"
    play_frames(frames, meta["fps"] * a.speed, audio, a.mute or a.speed != 1.0)


def cmd_live(a):
    cap, fps, _, cv2, np = open_video(a.input)
    width = a.width or max(20, shutil.get_terminal_size().columns - 1)

    def gen():
        while True:
            ok, frame = cap.read()
            if not ok:
                return
            yield frame_to_text(frame, width, a.color, a.invert, cv2, np)

    audio = None
    if not a.mute and shutil.which("ffplay"):
        audio = a.input
    play_frames(gen(), fps, audio, a.mute)
    cap.release()


def main():
    p = argparse.ArgumentParser(description="動画をASCIIアートにしてターミナルで再生")
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("convert", help="動画を文字フレームに変換")
    c.add_argument("input")
    c.add_argument("-o", "--output")
    c.add_argument("-w", "--width", type=int, default=120, help="横の文字数 (既定120)")
    c.add_argument("--fps", type=float, help="フレームレートを落とす (例: 15)")
    c.add_argument("--color", action="store_true", help="カラー (24bit色対応ターミナル)")
    c.add_argument("--invert", action="store_true", help="明暗反転 (白背景ターミナル用)")
    c.add_argument("--no-audio", action="store_true")
    c.set_defaults(func=cmd_convert)

    pl = sub.add_parser("play", help="変換済みファイルを再生")
    pl.add_argument("file")
    pl.add_argument("--audio", help="音声ファイル (既定: 同名の .m4a)")
    pl.add_argument("--mute", action="store_true")
    pl.add_argument("--speed", type=float, default=1.0, help="再生速度 (1以外は無音)")
    pl.set_defaults(func=cmd_play)

    lv = sub.add_parser("live", help="動画を直接その場で変換しながら再生")
    lv.add_argument("input")
    lv.add_argument("-w", "--width", type=int, help="横の文字数 (既定: 端末幅)")
    lv.add_argument("--color", action="store_true")
    lv.add_argument("--invert", action="store_true")
    lv.add_argument("--mute", action="store_true")
    lv.set_defaults(func=cmd_live)

    a = p.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
