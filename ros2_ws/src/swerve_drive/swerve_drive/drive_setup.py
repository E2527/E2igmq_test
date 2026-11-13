"""
ギア比と寸法を入れる、簡単な設定画面.

    ros2 run swerve_drive drive_setup        # dev コンテナの中で(ホストからは ros ros2 run ...)

「保存」を押すと、次の 2 つのファイルの値だけを書き換える(コメントやほかの行はそのまま):
    firmware/motor_node_f303k8/Core/Inc/app_config.h … STM32(エンコーダのカウント数、ギア比、モーターの上限)
    ros2_ws/src/swerve_drive/config/swerve.yaml       … Pi(モジュールの数と位置・ID、タイヤの半径、速さの上限)

モジュールの数(2・3・4・6 個)や間隔を変えると、下の表(名前・ID・位置)がすぐ作り直される。
表は直接書き換えてもよい(行を足す・消すこともできる)ので、ほかの並べ方にもできる。
表を自分で書き換えたあとは、間隔を変えても表はそのまま(「表を作り直す」を押すと選んだ並べ方に戻る)。

タイヤの速さの上限(max_wheel_speed)は、STM32 が出せる速さ(モーターの上限 × タイヤのギア比)から
自動で決める。保存したあと、STM32 は書き込み直し(make flash)、Pi 側は起動し直すと反映される
(./sim.sh fw なら、どちらも起動し直すだけで反映される)。
"""

import argparse
import math
import os
import sys

from PyQt5.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
                             QHBoxLayout, QHeaderView, QLabel, QMessageBox, QPushButton, QSpinBox,
                             QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from . import setup_files as sf

DEFAULT_APP_CONFIG = '/root/firmware/motor_node_f303k8/Core/Inc/app_config.h'
DEFAULT_SWERVE_YAML = '/root/ros2_ws/src/swerve_drive/config/swerve.yaml'


def spin_box(value, low, high, decimals, step, suffix=''):
    box = QDoubleSpinBox()
    box.setRange(low, high)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    if suffix:
        box.setSuffix(' ' + suffix)
    return box


class SetupWindow(QWidget):

    def __init__(self, app_config, swerve_yaml):
        super().__init__()
        self.app_config = app_config
        self.swerve_yaml = swerve_yaml
        self.setWindowTitle('ギア比と寸法の設定')
        self.custom = False         # 表を自分で書き換えたか
        self.load()

        layout = QVBoxLayout(self)
        layout.addWidget(self.make_firmware_group())
        layout.addWidget(self.make_robot_group())
        layout.addWidget(self.make_result_group())
        buttons = QHBoxLayout()
        reload_button = QPushButton('ファイルから読み直す')
        reload_button.clicked.connect(self.reload)
        save_button = QPushButton('保存')
        save_button.setDefault(True)
        save_button.clicked.connect(self.save)
        buttons.addWidget(reload_button)
        buttons.addStretch(1)
        buttons.addWidget(save_button)
        layout.addLayout(buttons)
        for box in self.findChildren((QDoubleSpinBox, QSpinBox)):
            box.valueChanged.connect(self.update_result)
        self.fill_table(self.pi['module_names'], self.pi['device_ids'], self.pi['module_x'],
                        self.pi['module_y'])

    # ---- ファイル ----

    def load(self):
        with open(self.app_config, encoding='utf-8') as f:
            self.fw = sf.read_defines(f.read())
        with open(self.swerve_yaml, encoding='utf-8') as f:
            self.pi = sf.read_yaml_values(f.read())

    def reload(self):
        self.load()
        self.counts.setValue(self.fw['APP_COUNTS_PER_REV'])
        self.steer.setValue(self.fw['APP_SWERVE_STEER_RATIO'])
        self.wheel.setValue(self.fw['APP_SWERVE_WHEEL_RATIO'])
        self.coupling.setValue(self.fw['APP_SWERVE_COUPLING'])
        self.motor_max.setValue(self.fw['APP_SWERVE_MAX_MOTOR_SPEED'])
        length, width = self.spacing()
        self.custom = True          # 読み直している間は、表を作り直さない(下の fill_table で決まる)
        self.length.setValue(length * 1000)
        self.width.setValue(width * 1000)
        self.radius.setValue(self.pi['wheel_radius'] * 1000)
        index = self.count.findData(len(self.pi['module_names']))
        if index >= 0:
            self.count.blockSignals(True)
            self.count.setCurrentIndex(index)
            self.count.blockSignals(False)
        self.fill_table(self.pi['module_names'], self.pi['device_ids'], self.pi['module_x'],
                        self.pi['module_y'])

    def spacing(self):
        """今の swerve.yaml のモジュールの位置から、前後と左右の間隔 [m] を求める."""
        xs, ys = self.pi['module_x'], self.pi['module_y']
        return max(xs) - min(xs), max(ys) - min(ys)

    # ---- モジュールの表 ----

    COLUMNS = ('名前', 'STM32 の ID', '前 [mm]', '左 [mm]')

    def fill_table(self, names, ids, xs, ys):
        # 表が「選んだ並べ方」のままか(そうなら、間隔を変えたときに表も作り直す)
        count = len(names)
        self.custom = count not in sf.LAYOUTS or not self.same_as_layout(names, ids, xs, ys)
        self.table.blockSignals(True)
        self.table.setRowCount(len(names))
        for row, values in enumerate(zip(names, ids, xs, ys)):
            name, dev_id, x, y = values
            for col, text in enumerate((str(name), str(dev_id), f'{x * 1000:g}',
                                        f'{y * 1000:g}')):
                self.table.setItem(row, col, QTableWidgetItem(text))
        self.table.blockSignals(False)
        self.update_result()

    @staticmethod
    def same_as_layout(names, ids, xs, ys):
        """表の中身が、その数の決まった並べ方(間隔は表から求める)と同じかどうか."""
        layout = sf.module_layout(len(names), max(xs) - min(xs), max(ys) - min(ys))
        close = all(abs(a - b) < 1e-6 for a, b in zip(xs + ys, layout[2] + layout[3]))
        return list(names) == layout[0] and list(ids) == layout[1] and close

    def on_table_edited(self, *_):
        self.custom = True
        self.update_result()

    def on_layout_changed(self, *_):
        """数や間隔を変えたとき。表を自分で書き換えていなければ、表も作り直す."""
        if not self.custom:
            self.use_layout()

    def read_table(self):
        """表から (名前, ID, x [m], y [m]) の並びを読む。数でないところがあれば ValueError."""
        names, ids, xs, ys = [], [], [], []
        for row in range(self.table.rowCount()):
            cells = [self.table.item(row, col) for col in range(len(self.COLUMNS))]
            texts = [c.text().strip() if c is not None else '' for c in cells]
            try:
                ids.append(int(texts[1]))
                xs.append(float(texts[2]) / 1000)
                ys.append(float(texts[3]) / 1000)
            except ValueError:
                raise ValueError(f'{row + 1} 行目: ID は整数、前・左は数を入れてください')
            names.append(texts[0])
        return names, ids, xs, ys

    def use_layout(self):
        """選んだ数の並べ方で、表を作り直す."""
        names, ids, xs, ys = sf.module_layout(self.count.currentData(),
                                              self.length.value() / 1000,
                                              self.width.value() / 1000)
        self.fill_table(names, ids, xs, ys)

    def add_row(self):
        row = self.table.rowCount()
        self.table.blockSignals(True)
        self.table.insertRow(row)
        for col, text in enumerate((f'module_{row + 1}', str(row + 1), '0', '0')):
            self.table.setItem(row, col, QTableWidgetItem(text))
        self.table.blockSignals(False)
        self.on_table_edited()

    def remove_row(self):
        row = self.table.currentRow()
        self.table.removeRow(row if row >= 0 else self.table.rowCount() - 1)
        self.on_table_edited()

    def save(self):
        fw = {'APP_COUNTS_PER_REV': self.counts.value(),
              'APP_SWERVE_STEER_RATIO': self.steer.value(),
              'APP_SWERVE_WHEEL_RATIO': self.wheel.value(),
              'APP_SWERVE_COUPLING': self.coupling.value(),
              'APP_SWERVE_MAX_MOTOR_SPEED': self.motor_max.value()}
        if fw['APP_SWERVE_STEER_RATIO'] == 0 or fw['APP_SWERVE_WHEEL_RATIO'] == 0:
            QMessageBox.warning(self, '保存できません', 'ギア比に 0 は使えません')
            return
        try:
            names, ids, xs, ys = self.read_table()
        except ValueError as e:
            QMessageBox.warning(self, '保存できません', str(e))
            return
        problems = sf.check_modules(names, ids, xs, ys)
        if problems:
            QMessageBox.warning(self, '保存できません', '\n'.join(problems))
            return
        pi = {'module_names': names, 'device_ids': ids, 'module_x': xs, 'module_y': ys,
              'wheel_radius': self.radius.value() / 1000,
              'max_wheel_speed': round(self.max_wheel(), 2)}
        try:
            for path, writer, values in ((self.app_config, sf.write_defines, fw),
                                         (self.swerve_yaml, sf.write_yaml_values, pi)):
                with open(path, encoding='utf-8') as f:
                    text = writer(f.read(), values)
                with open(path, 'w', encoding='utf-8') as f:
                    f.write(text)
        except (OSError, KeyError) as e:
            QMessageBox.critical(self, '保存できませんでした', str(e))
            return
        QMessageBox.information(
            self, '保存しました',
            f'{self.app_config}\n{self.swerve_yaml}\nを書き換えました'
            f'(モジュール {len(names)} 個: {", ".join(names)})。\n\n'
            f'・STM32: モジュールごとに書き込み直す(make APP=swerve NODE_ID=… flash。'
            f'NODE_ID は {", ".join(str(i) for i in ids)})\n'
            '・Pi / PC: 起動し直す(./sim.sh fw なら、起動し直すだけで両方反映)')

    # ---- 画面 ----

    def make_firmware_group(self):
        group = QGroupBox('STM32(app_config.h)')
        form = QFormLayout(group)
        # カウント数の計算: エンコーダの PPR × 4 × 減速比
        self.ppr = QSpinBox()
        self.ppr.setRange(1, 100000)
        self.ppr.setValue(16)
        self.reduction = spin_box(1.0, 0.01, 10000, 3, 1.0)
        calc = QPushButton('↓ 計算して入れる')
        calc.clicked.connect(
            lambda: self.counts.setValue(self.ppr.value() * 4 * self.reduction.value()))
        row = QHBoxLayout()
        for widget in (QLabel('PPR'), self.ppr, QLabel('× 4 × 減速比'), self.reduction, calc):
            row.addWidget(widget)
        form.addRow('カウント数の計算', row)
        self.counts = spin_box(self.fw['APP_COUNTS_PER_REV'], 1, 1e7, 1, 1, 'カウント/回転')
        form.addRow('差動ギアに入る軸 1 回転のカウント数', self.counts)
        self.steer = spin_box(self.fw['APP_SWERVE_STEER_RATIO'], -100, 100, 4, 0.01)
        form.addRow('向きのギア比(同じ向きに 1 rad → モジュール ? rad)', self.steer)
        self.wheel = spin_box(self.fw['APP_SWERVE_WHEEL_RATIO'], -100, 100, 4, 0.01)
        form.addRow('タイヤのギア比(逆向きに 1 rad ずつ → タイヤ ? rad)', self.wheel)
        self.coupling = spin_box(self.fw['APP_SWERVE_COUPLING'], -100, 100, 4, 0.01)
        form.addRow('干渉(モジュールだけ 1 rad → タイヤ ? rad。ふつう 0)', self.coupling)
        self.motor_max = spin_box(self.fw['APP_SWERVE_MAX_MOTOR_SPEED'], 0.1, 10000, 1, 1,
                                  'rad/s')
        form.addRow('モーター 1 個の最大の速さ', self.motor_max)
        return group

    def make_robot_group(self):
        group = QGroupBox('ロボット(swerve.yaml)')
        form = QFormLayout(group)

        # 並べ方を選んで、表に入れる
        self.count = QComboBox()
        for count in sorted(sf.LAYOUTS):
            self.count.addItem(f'{count} 個 … {sf.LAYOUT_NOTES[count]}', count)
        index = self.count.findData(len(self.pi['module_names']))
        self.count.setCurrentIndex(index if index >= 0 else self.count.findData(4))
        form.addRow('モジュールの数', self.count)
        length, width = self.spacing()
        self.length = spin_box(length * 1000, 0, 5000, 1, 5, 'mm')
        form.addRow('モジュールの前後の間隔(中心から中心)', self.length)
        self.width = spin_box(width * 1000, 0, 5000, 1, 5, 'mm')
        form.addRow('モジュールの左右の間隔(中心から中心)', self.width)
        use = QPushButton('↓ 表を作り直す(自分で書き換えた表を、選んだ並べ方に戻す)')
        use.clicked.connect(self.use_layout)
        form.addRow('', use)
        # 数を選んだら、すぐ表を作り直す。間隔は、表を自分で書き換えていないときだけ反映する
        self.count.currentIndexChanged.connect(self.use_layout)
        self.length.valueChanged.connect(self.on_layout_changed)
        self.width.valueChanged.connect(self.on_layout_changed)

        # 表: 1 行が 1 モジュール。直接書き換えてもよい
        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setMinimumHeight(190)
        self.table.itemChanged.connect(self.on_table_edited)
        form.addRow(self.table)
        rows = QHBoxLayout()
        add = QPushButton('行を足す')
        add.clicked.connect(self.add_row)
        remove = QPushButton('選んだ行を消す')
        remove.clicked.connect(self.remove_row)
        note = QLabel('前・左は、ロボットの中心からの距離(後ろ・右はマイナス)。ID は STM32 の NODE_ID')
        note.setWordWrap(True)
        rows.addWidget(add)
        rows.addWidget(remove)
        rows.addWidget(note, 1)
        form.addRow(rows)

        self.radius = spin_box(self.pi['wheel_radius'] * 1000, 1, 1000, 2, 0.5, 'mm')
        form.addRow('タイヤの半径', self.radius)
        return group

    def make_result_group(self):
        group = QGroupBox('計算結果(保存すると swerve.yaml の max_wheel_speed に入る)')
        form = QFormLayout(group)
        self.result_wheel = QLabel()
        self.result_speed = QLabel()
        self.result_turn = QLabel()
        self.result_check = QLabel()
        self.result_check.setWordWrap(True)
        form.addRow('モジュールの設定', self.result_check)
        form.addRow('タイヤの速さの上限', self.result_wheel)
        form.addRow('ロボットの最高速度(まっすぐ)', self.result_speed)
        form.addRow('その場で回る最高の速さ', self.result_turn)
        return group

    def max_wheel(self):
        return sf.max_wheel_speed(self.motor_max.value(), self.wheel.value())

    def update_result(self, *_):
        if not hasattr(self, 'result_check'):      # 画面を作っている途中
            return
        wheel = self.max_wheel()
        speed = wheel * self.radius.value() / 1000
        self.result_wheel.setText(f'{wheel:.2f} rad/s')
        self.result_speed.setText(f'{speed:.2f} m/s({speed * 3.6:.1f} km/h)')
        try:
            names, ids, xs, ys = self.read_table()
            problems = sf.check_modules(names, ids, xs, ys)
        except ValueError as e:
            names, xs, ys, problems = [], [], [], [str(e)]
        if problems:
            self.result_check.setText('⚠ ' + ' / '.join(problems))
            self.result_check.setStyleSheet('color: #d33;')
        else:
            self.result_check.setText(f'OK({len(names)} 個: {", ".join(names)})')
            self.result_check.setStyleSheet('color: #2a2;')
        # その場で回るときは、中心からいちばん遠いモジュールがいちばん速い
        reach = max((math.hypot(x, y) for x, y in zip(xs, ys)), default=0.0)
        if reach > 0:
            turn = speed / reach
            self.result_turn.setText(f'{turn:.2f} rad/s({math.degrees(turn):.0f} °/s)')
        else:
            self.result_turn.setText('--')


def main():
    parser = argparse.ArgumentParser(description='ギア比と寸法の設定画面')
    parser.add_argument('--app-config', default=DEFAULT_APP_CONFIG)
    parser.add_argument('--swerve-yaml', default=DEFAULT_SWERVE_YAML)
    args, _ = parser.parse_known_args()     # ros2 run が付ける引数は無視する
    for path in (args.app_config, args.swerve_yaml):
        if not os.path.exists(path):
            sys.exit(f'{path} が見つかりません(dev コンテナの中で動かすか、--app-config / '
                     '--swerve-yaml で場所を指定してください)')
    app = QApplication(sys.argv[:1])
    window = SetupWindow(args.app_config, args.swerve_yaml)
    window.show()
    app.exec_()


if __name__ == '__main__':
    main()
