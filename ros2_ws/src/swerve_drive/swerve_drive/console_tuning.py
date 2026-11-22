"""
ロボットコンソールの「調整」タブ: STM32 のゲインを、書き込み直さずに読み書きする.

    1. STM32 を選んで「読み込む」… 今のゲインを STM32 から読む(ファームウェアの種類も分かる)
    2. 新しい値を入れて「送る」  … すぐに STM32 の動きが変わる(「グラフ」タブで見ながら調整する)
    3. 良い値が見つかったら、下の「app_config.h に書く値」を app_config.h に写して、書き込み直す
       (STM32 は RAM にだけ覚えるので、電源を切ると app_config.h の値にもどる)

スワーブなら「今の向きを 0(前)にする」で、モジュールを手でまっすぐ前に向けてから、向きの基準を合わせ直せる。

通信は stm<ID>/param / stm<ID>/param_reply(stm_bridge。番号は stm_bridge/protocol.py の PARAMS と
firmware/stm32_can/params.h)。シミュレーターでは ./sim.sh fw(ファームウェアをそのまま動かす)のときに使える。
"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QScrollArea,
                             QVBoxLayout, QWidget)
from stm_bridge import protocol

# 調整できる番号(並べる順)と、app_config.h での名前
TUNABLE = [(1, 'APP_KP'), (2, 'APP_KI'), (3, 'APP_KFF'), (4, 'APP_MAX_DUTY'),
           (10, 'APP_SWERVE_ANGLE_KP'), (11, 'APP_SWERVE_ANGLE_KD'),
           (12, 'APP_SWERVE_ANGLE_FF'), (13, 'APP_SWERVE_MAX_STEER_RATE')]
FIRMWARE_ID = protocol.PARAM_IDS['firmware']
ANGLE_ID = protocol.PARAM_IDS['angle']
READ_IDS = [FIRMWARE_ID] + [pid for pid, _ in TUNABLE] + [ANGLE_ID]


def firmware_kind(values):
    """{番号: (値, 結果, 何秒前)} から、ファームウェアの種類('motors' / 'swerve' / None)."""
    entry = values.get(FIRMWARE_ID)
    if entry is None or entry[1] != protocol.PARAM_OK:
        return None
    return protocol.FIRMWARE_KINDS.get(int(round(entry[0])))


def config_text(values):
    """app_config.h に書く #define の行."""
    lines = []
    for pid, macro in TUNABLE:
        entry = values.get(pid)
        if entry is not None and entry[1] == protocol.PARAM_OK:
            number = f'{entry[0]:.6g}'
            if not any(ch in number for ch in '.e'):
                number += '.0'                  # C の float は 8f ではなく 8.0f と書く
            lines.append(f'#define {macro} {number}f')
    return '\n'.join(lines)


class TuneTab(QScrollArea):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWidgetResizable(True)
        self.setFocusPolicy(Qt.NoFocus)
        page = QWidget()
        column = QVBoxLayout(page)

        top = QHBoxLayout()
        top.addWidget(QLabel('STM32'))
        self.choice = QComboBox()
        self.choice.setFocusPolicy(Qt.NoFocus)
        self.choice.currentIndexChanged.connect(lambda _: self.read_all())
        top.addWidget(self.choice, 1)
        read = QPushButton('読み込む')
        read.setFocusPolicy(Qt.NoFocus)
        read.clicked.connect(self.read_all)
        top.addWidget(read)
        column.addLayout(top)
        self.same = QCheckBox('同じ種類の STM32 全部に送る(スワーブの 4 つのモジュールなど)')
        self.same.setFocusPolicy(Qt.NoFocus)
        self.same.setChecked(True)
        column.addWidget(self.same)
        self.info = QLabel('')
        self.info.setWordWrap(True)
        column.addWidget(self.info)

        gains = QGroupBox('ゲイン(STM32 の今の値 → 新しい値)')
        self.grid = QGridLayout(gains)
        self.rows = {}
        for r, (pid, _) in enumerate(TUNABLE):
            name, text, unit, _kind = protocol.PARAMS[pid]
            label = QLabel(f'{text}' + (f' [{unit}]' if unit else ''))
            label.setWordWrap(True)
            now = QLabel('--')
            now.setMinimumWidth(70)
            box = QDoubleSpinBox()
            box.setRange(0.0, 100.0)
            box.setDecimals(4)
            box.setSingleStep(0.01)
            box.setKeyboardTracking(False)
            box.setFocusPolicy(Qt.ClickFocus)
            send = QPushButton('送る')
            send.setFocusPolicy(Qt.NoFocus)
            send.clicked.connect(lambda _=False, p=pid, b=box: self.send(p, b.value()))
            for c, w in enumerate((label, now, box, send)):
                self.grid.addWidget(w, r, c)
            self.rows[pid] = (label, now, box, send)
        self.grid.setColumnStretch(0, 1)
        column.addWidget(gains)

        self.angle_group = QGroupBox('スワーブの向きの基準')
        angle_layout = QVBoxLayout(self.angle_group)
        self.angle_label = QLabel('今の向き: --')
        angle_layout.addWidget(self.angle_label)
        zero = QPushButton('今の向きを 0(前)にする')
        zero.setFocusPolicy(Qt.NoFocus)
        zero.clicked.connect(lambda: self.send(ANGLE_ID, 0.0))
        angle_layout.addWidget(zero)
        hint = QLabel('モジュールを手でまっすぐ前に向けてから押します(止まっているときに)。'
                      '電源を入れたときの向きが 0 になるので、ずれていたらここで合わせ直します。')
        hint.setWordWrap(True)
        angle_layout.addWidget(hint)
        column.addWidget(self.angle_group)

        save = QGroupBox('app_config.h に書く値(電源を切ると元にもどるので、良い値はここに写す)')
        save_layout = QVBoxLayout(save)
        self.config = QPlainTextEdit()
        self.config.setReadOnly(True)
        self.config.setMaximumHeight(140)
        save_layout.addWidget(self.config)
        column.addWidget(save)
        note = QLabel('「グラフ」タブで指令(点線)と実際(実線)を見ながら調整します。'
                      'シミュレーターでは ./sim.sh fw のとき(ファームウェアをそのまま動かす)に使えます。')
        note.setWordWrap(True)
        column.addWidget(note)
        column.addStretch(1)
        self.setWidget(page)
        self.ids = []
        self.last_seen = {}         # (ID, 番号) → 最後に表示した STM32 の値
        self.view_key = None
        self.message = ''

    # ---- 操作 ----

    def current_id(self):
        text = self.choice.currentText()
        return int(text[3:]) if text.startswith('stm') else None

    def targets(self, snap_params):
        """送る先の ID(「全部に送る」なら、選んだ STM32 と同じ種類の全部)."""
        dev_id = self.current_id()
        if dev_id is None:
            return []
        if not self.same.isChecked():
            return [dev_id]
        kind = firmware_kind(snap_params.get(dev_id, {}))
        return [i for i in self.ids
                if i == dev_id or (kind and firmware_kind(snap_params.get(i, {})) == kind)]

    def read_all(self):
        dev_id = self.current_id()
        if dev_id is None:
            return
        for pid in READ_IDS:
            self.node.request_param(dev_id, pid)
        self.message = f'stm{dev_id} から読んでいます…'

    def send(self, param_id, value):
        snap_params = self.node.state.snapshot(0.0, with_history=False).params
        targets = self.targets(snap_params)
        for dev_id in targets:
            self.node.request_param(dev_id, param_id, value)
        name = '今の向き' if param_id == ANGLE_ID else protocol.PARAMS[param_id][1]
        self.message = f'{name} = {value:g} を ' + '、'.join(f'stm{i}' for i in targets) + ' に送りました'

    # ---- 表示(30 回/秒) ----

    def refresh(self, snap):
        ids = sorted(r.dev_id for r in snap.stm_rows)
        if ids != self.ids:
            current = self.choice.currentText()
            self.ids = ids
            self.choice.blockSignals(True)
            self.choice.clear()
            self.choice.addItems([f'stm{i}' for i in ids])
            index = self.choice.findText(current)
            self.choice.setCurrentIndex(max(0, index))
            self.choice.blockSignals(False)
            if index < 0 and ids:
                self.read_all()
        dev_id = self.current_id()
        values = snap.params.get(dev_id, {}) if dev_id is not None else {}
        key = (dev_id, tuple(sorted((k, round(v[0], 6), v[1]) for k, v in values.items())),
               self.message)
        if key == self.view_key:
            return
        self.view_key = key
        kind = firmware_kind(values)
        if dev_id is None:
            self.info.setText('STM32 が見つかりません(stm<ID>/command が流れていません)')
        elif not values:
            self.info.setText(f'stm{dev_id} から返事がありません。実機では stm_bridge と新しい'
                              'ファームウェアが要ります。シミュレーターでは ./sim.sh fw で使えます')
        else:
            names = {'motors': 'モーター 2 個の速度制御', 'swerve': 'スワーブのモジュール'}
            self.info.setText(f'stm{dev_id}: {names.get(kind, "種類が分からない")}'
                              + (f'\n{self.message}' if self.message else ''))
        for pid, (label, now, box, send) in self.rows.items():
            needs = protocol.PARAMS[pid][3]
            visible = needs == 'all' or kind is None or needs == kind
            for w in (label, now, box, send):
                w.setVisible(visible)
            entry = values.get(pid)
            if entry is None:
                now.setText('--')
                continue
            value, status, _age = entry
            if status == protocol.PARAM_OK:
                now.setText(f'{value:.4g}')
                # STM32 の値が変わったときだけ、新しい値の欄にも入れる(入力中の値を消さないように)
                if self.last_seen.get((dev_id, pid)) != value and not box.hasFocus():
                    box.setValue(value)
                self.last_seen[(dev_id, pid)] = value
            else:
                now.setText(protocol.PARAM_STATUS_TEXT.get(status, f'結果 {status}'))
        angle = values.get(ANGLE_ID)
        self.angle_group.setVisible(kind == 'swerve')
        if angle is not None and angle[1] == protocol.PARAM_OK:
            self.angle_label.setText(f'今の向き: {angle[0]:+.3f} rad({angle[0] * 57.2958:+.1f}°)')
        self.config.setPlainText(config_text(values))
