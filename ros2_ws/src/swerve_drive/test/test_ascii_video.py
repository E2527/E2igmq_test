import gzip
import json

import pytest
from swerve_drive import ascii_video as av


def write_video(path, frames, fps=24.0, color=False):
    with gzip.open(path, 'wt', encoding='utf-8') as f:
        f.write(json.dumps({'fps': fps, 'color': color}) + '\n')
        f.write(av.FRAME_SEPARATOR.join(frames))


def test_read_video(tmp_path):
    path = str(tmp_path / 'a.asc.gz')
    write_video(path, ['ab\ncd', '12\n34'], fps=15.0)
    fps, frames = av.read_video(path)
    assert fps == 15.0 and frames == ['ab\ncd', '12\n34']
    assert av.frame_size(frames[0]) == (2, 2)


def test_read_video_strips_colour(tmp_path):
    path = str(tmp_path / 'c.asc.gz')
    write_video(path, ['\x1b[38;2;8;16;24ma\x1b[38;2;0;0;0mb\x1b[0m'], color=True)
    assert av.read_video(path)[1] == ['ab']


def test_read_video_rejects_bad_files(tmp_path):
    with pytest.raises(ValueError):
        av.read_video(str(tmp_path / 'missing.asc.gz'))
    plain = tmp_path / 'plain.asc.gz'
    plain.write_text('gzip ではない')
    with pytest.raises(ValueError):
        av.read_video(str(plain))
    empty = str(tmp_path / 'empty.asc.gz')
    write_video(empty, [])
    with pytest.raises(ValueError):
        av.read_video(empty)
    odd = str(tmp_path / 'odd.asc.gz')
    write_video(odd, ['a'], fps=0.0)
    with pytest.raises(ValueError):
        av.read_video(odd)


def test_find_video_and_audio(tmp_path):
    folder = tmp_path / 'egg'
    folder.mkdir()
    assert av.find_video([str(folder)], {}) is None
    write_video(str(folder / 'b.asc.gz'), ['x'])
    write_video(str(folder / 'a.asc.gz'), ['x'])
    found = av.find_video([str(tmp_path / 'none'), str(folder)], {})
    assert found == str(folder / 'a.asc.gz')        # 名前の順でいちばん先のもの
    assert av.audio_for(found) is None
    (folder / 'a.m4a').write_bytes(b'')
    assert av.audio_for(found) == str(folder / 'a.m4a')
    # 環境変数で直接指定したときは、そちらを使う(ファイルがなければ None)
    env = {av.VIDEO_ENV: str(folder / 'b.asc.gz')}
    assert av.find_video([str(folder)], env) == str(folder / 'b.asc.gz')
    assert av.find_video([str(folder)], {av.VIDEO_ENV: str(folder / 'zzz.asc.gz')}) is None


def test_secret_code():
    code = av.SecretCode()
    assert not any(code.press(k) for k in av.SECRET_KEYS[:-1])
    assert code.press(av.SECRET_KEYS[-1])
    # まちがえたあとでも、続けて正しく押せば開く
    assert not code.press('left')
    assert [code.press(k) for k in av.SECRET_KEYS][-1]
    assert not code.press('right')                  # 開いたあとは、最初からやり直し


def test_click_counter():
    clicks = av.ClickCounter(count=3, within_s=1.0)
    assert [clicks.click(t) for t in (0.0, 0.3, 0.6)] == [False, False, True]
    assert [clicks.click(t) for t in (5.0, 5.9, 7.5)] == [False, False, False]     # 間があいた


def test_demo_frames_are_uniform():
    frames = av.demo_frames(columns=60, rows=16, count=8)
    assert len(frames) == 8 and len(set(frames)) > 1
    for frame in frames:
        lines = frame.split('\n')
        assert len(lines) == 16
        # 全角の文字は 2 文字分。どの行も、画面での幅が同じになる
        assert {av._display_width(line) for line in lines} == {60}
