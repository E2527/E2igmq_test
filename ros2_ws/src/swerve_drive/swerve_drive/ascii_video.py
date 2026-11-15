"""
隠し機能(イースターエッグ)で流す、文字アートの動画のデータ.

画面にも ROS にも依存しないので、PC だけでテストできる(test/test_ascii_video.py)。

    動画のファイル … tools/ascii-video/asciiplay.py の convert で作る .asc.gz
                     (1 行目が {"fps": 24.0, "color": false}、そのあとはフレームを改ページの文字(0x0C)で区切ったもの)
    置き場所       … ros2_ws/egg/(コンテナの中では ~/ros2_ws/egg/)。同じ名前の .m4a があれば音も鳴る
    ないとき       … demo_frames() の、計算で作る短い模様を代わりに流す
"""

import glob
import gzip
import json
import math
import os
import re

RAMP = ' .\'`^",:;Il!i><~+_-?][}{1)(|/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$'    # 暗い → 明るい
FRAME_SEPARATOR = '\f'
VIDEO_ENV = 'ROBOT_CONSOLE_EGG'     # 動画のファイルを直接指定する環境変数
_ANSI = re.compile(r'\x1b\[[0-9;]*m')


def read_video(path):
    """
    .asc.gz を読み、(1 秒あたりのフレーム数, フレームの並び) を返す.

    色付きのファイルは、色の指定を取りのぞいて白黒として読む。
    読めないときは ValueError。
    """
    try:
        with gzip.open(path, 'rt', encoding='utf-8') as f:
            meta = json.loads(f.readline())
            data = f.read()
        fps = float(meta['fps'])
    except (OSError, EOFError, ValueError, KeyError, TypeError) as e:
        raise ValueError(f'{path} を読めません: {e}')
    if not 1.0 <= fps <= 240.0:
        raise ValueError(f'{path}: fps が変です({fps})')
    if meta.get('color'):
        data = _ANSI.sub('', data)
    frames = [frame for frame in data.split(FRAME_SEPARATOR) if frame.strip()]
    if not frames:
        raise ValueError(f'{path}: フレームがありません')
    return fps, frames


def search_dirs():
    """動画を探すフォルダ(先にあるものを優先)."""
    return [os.path.join(os.path.expanduser('~'), 'ros2_ws', 'egg'),
            os.path.join(os.getcwd(), 'egg')]


def find_video(dirs=None, environ=None):
    """流す動画のファイルを探す。見つからなければ None."""
    environ = os.environ if environ is None else environ
    direct = environ.get(VIDEO_ENV, '')
    if direct:
        return direct if os.path.isfile(direct) else None
    for folder in (search_dirs() if dirs is None else dirs):
        found = sorted(glob.glob(os.path.join(folder, '*.asc.gz')))
        if found:
            return found[0]
    return None


def audio_for(video_path):
    """動画と同じ名前の音声(.m4a)。なければ None."""
    stem = video_path[:-len('.asc.gz')] if video_path.endswith('.asc.gz') else video_path
    audio = stem + '.m4a'
    return audio if os.path.isfile(audio) else None


def frame_size(frame):
    """フレームの (横の文字数, 行数)."""
    lines = frame.split('\n')
    return max(len(line) for line in lines), len(lines)


def demo_frames(columns=80, rows=24, count=96,
                lines=('?', '', 'ros2_ws/egg/ に .asc.gz を置くと、ここで動画が流れます')):
    """動画のファイルがないときに流す、波の模様(くり返しても切れ目が見えない)."""
    frames = []
    top = rows // 2 - len(lines) // 2
    for n in range(count):
        t = 2.0 * math.pi * n / count
        grid = []
        for y in range(rows):
            row = []
            for x in range(columns):
                dx, dy = (x - columns / 2.0) / 2.0, y - rows / 2.0
                value = (math.sin(math.hypot(dx, dy) * 0.55 - 2.0 * t)
                         + math.sin(dx * 0.35 + t) + math.sin(dy * 0.5 - t))
                row.append(RAMP[int((value + 3.0) / 6.0 * (len(RAMP) - 1))])
            grid.append(''.join(row))
        for i, text in enumerate(lines):
            if text and 0 <= top + i < rows:
                grid[top + i] = _overlay(grid[top + i], f'  {text}  ')
        frames.append('\n'.join(grid))
    return frames


def _display_width(text):
    """全角の文字を 2 文字分として数えた、文字の幅."""
    return sum(1 if ord(c) < 0x2000 else 2 for c in text)


def _overlay(line, text):
    """行のまん中に text を重ねる(全角の文字の分だけ、もとの行を多めに消す)."""
    width = _display_width(text)
    if width > len(line):
        return line
    start = (len(line) - width) // 2
    return line[:start] + text + line[start + width:]
