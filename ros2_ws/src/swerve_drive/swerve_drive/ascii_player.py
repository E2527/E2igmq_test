"""
文字アートの動画を流すウィンドウ(ロボットコンソールの隠し機能).

データは ascii_video.py が読む。動画のファイルがなければ、短い模様をくり返す。
音声(動画と同じ名前の .m4a)は、ffplay が入っていれば鳴らす。

閉じるには: Esc、Q、クリック、またはウィンドウを閉じる。

このウィンドウを開くと、コンソールは「ほかのウィンドウに移った」と判断して、
押していたキーを離したことにする(ロボットは止まる)。
"""

import shutil
import subprocess
import time

from PyQt5.QtCore import QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QFontMetricsF, QPainter
from PyQt5.QtWidgets import QWidget

from . import ascii_video as av

FONT_FAMILIES = ('DejaVu Sans Mono', 'Noto Sans Mono', 'Liberation Mono', 'Monospace')


class AsciiPlayer(QWidget):
    """文字のフレームを、時刻に合わせて順番に描く."""

    def __init__(self, video_path=None):
        super().__init__()
        self.setWindowTitle('?')
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setStyleSheet('background: #000000;')
        self.resize(1120, 660)
        self.audio = None
        self.note = ''
        self.loop = False
        self.fps, self.frames = 20.0, []
        audio_path = None
        if video_path is not None:
            try:
                self.fps, self.frames = av.read_video(video_path)
                audio_path = av.audio_for(video_path)
            except ValueError as e:
                self.note = str(e)
        if not self.frames:            # ファイルがない / 読めないときは、模様をくり返す
            self.fps, self.frames, self.loop = 20.0, av.demo_frames(), True
        self.columns, self.rows = av.frame_size(self.frames[0])
        self.index = 0
        self.start_audio(audio_path)
        self.started = time.monotonic()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(5, int(500 / self.fps)))      # 1 フレームの半分の間隔で時刻を見る

    def start_audio(self, path):
        player = shutil.which('ffplay')
        if path is None or player is None:
            return
        try:
            self.audio = subprocess.Popen(
                [player, '-nodisp', '-autoexit', '-loglevel', 'quiet', path],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            self.audio = None

    def tick(self):
        """今の時刻に合うフレームを選ぶ(描くのが遅れても、音とずれない)."""
        index = int((time.monotonic() - self.started) * self.fps)
        if index >= len(self.frames):
            if not self.loop:
                self.close()
                return
            index %= len(self.frames)
        if index != self.index:
            self.index = index
            self.update()

    def fitted_font(self):
        """フレームの全体がウィンドウに入る、いちばん大きい等幅フォント."""
        font = QFont()
        font.setFamilies(list(FONT_FAMILIES))
        font.setStyleHint(QFont.Monospace)
        font.setFixedPitch(True)
        font.setPixelSize(100)
        metrics = QFontMetricsF(font)
        scale = min(self.width() / (metrics.horizontalAdvance('M') * self.columns),
                    self.height() / (metrics.lineSpacing() * self.rows))
        font.setPixelSize(max(4, int(100 * scale)))
        return font

    def paintEvent(self, _event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor('#000000'))
        font = self.fitted_font()
        metrics = QFontMetricsF(font)
        line_h = metrics.lineSpacing()
        block_w = metrics.horizontalAdvance('M') * self.columns
        x = (self.width() - block_w) / 2.0
        y = (self.height() - line_h * self.rows) / 2.0
        p.setFont(font)
        p.setPen(QColor('#e9eef6'))
        for i, line in enumerate(self.frames[self.index].split('\n')[:self.rows]):
            p.drawText(QRectF(x, y + i * line_h, block_w + line_h, line_h),
                       Qt.AlignLeft | Qt.AlignVCenter | Qt.TextDontClip, line)
        if self.note:
            p.setPen(QColor('#fb5c72'))
            p.drawText(QRectF(8, 4, self.width() - 16, line_h * 2), Qt.AlignLeft | Qt.AlignTop,
                       self.note)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Escape, Qt.Key_Q, Qt.Key_Space):
            self.close()

    def mousePressEvent(self, _event):
        self.close()

    def closeEvent(self, event):
        self.timer.stop()
        if self.audio is not None and self.audio.poll() is None:
            self.audio.terminate()
        super().closeEvent(event)
