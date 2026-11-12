"""
ギア比と寸法を入れる、簡単な設定画面.

    ros2 run swerve_drive drive_setup        # dev コンテナの中で(ホストからは ros ros2 run ...)

「保存」を押すと、次の 2 つのファイルの値だけを書き換える(コメントやほかの行はそのまま):
    firmware/motor_node_f303k8/Core/Inc/app_config.h … STM32(エンコーダのカウント数、ギア比、モーターの上限)
    ros2_ws/src/swerve_drive/config/swerve.yaml       … Pi(モジュールの間隔、タイヤの半径、速さの上限)

タイヤの速さの上限(max_wheel_speed)は、STM32 が出せる速さ(モーターの上限 × タイヤのギア比)から
自動で決める。保存したあと、STM32 は書き込み直し(make flash)、Pi 側は起動し直すと反映される
(./sim.sh fw なら、どちらも起動し直すだけで反映される)。
"""

import argparse
import math
import os
import sys

from PyQt5.QtWidgets import (QApplication, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
                             QLabel, QMessageBox, QPushButton, QSpinBox, QVBoxLayout, QWidget)

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
        self.update_result()

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
        self.length.setValue(length * 1000)
        self.width.setValue(width * 1000)
        self.radius.setValue(self.pi['wheel_radius'] * 1000)

    def spacing(self):
        """今の swerve.yaml のモジュールの位置から、前後と左右の間隔 [m] を求める."""
        xs, ys = self.pi['module_x'], self.pi['module_y']
        return max(xs) - min(xs), max(ys) - min(ys)

    def save(self):
        fw = {'APP_COUNTS_PER_REV': self.counts.value(),
              'APP_SWERVE_STEER_RATIO': self.steer.value(),
              'APP_SWERVE_WHEEL_RATIO': self.wheel.value(),
              'APP_SWERVE_COUPLING': self.coupling.value(),
              'APP_SWERVE_MAX_MOTOR_SPEED': self.motor_max.value()}
        if fw['APP_SWERVE_STEER_RATIO'] == 0 or fw['APP_SWERVE_WHEEL_RATIO'] == 0:
            QMessageBox.warning(self, '保存できません', 'ギア比に 0 は使えません')
            return
        xs, ys = sf.module_layout(self.length.value() / 1000, self.width.value() / 1000)
        pi = {'module_x': xs, 'module_y': ys, 'wheel_radius': self.radius.value() / 1000,
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
            f'{self.app_config}\n{self.swerve_yaml}\nを書き換えました。\n\n'
            '・STM32: 書き込み直す(make APP=swerve NODE_ID=… flash)\n'
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
        length, width = self.spacing()
        self.length = spin_box(length * 1000, 10, 5000, 1, 5, 'mm')
        form.addRow('モジュールの前後の間隔(中心から中心)', self.length)
        self.width = spin_box(width * 1000, 10, 5000, 1, 5, 'mm')
        form.addRow('モジュールの左右の間隔(中心から中心)', self.width)
        self.radius = spin_box(self.pi['wheel_radius'] * 1000, 1, 1000, 2, 0.5, 'mm')
        form.addRow('タイヤの半径', self.radius)
        return group

    def make_result_group(self):
        group = QGroupBox('計算結果(保存すると swerve.yaml の max_wheel_speed に入る)')
        form = QFormLayout(group)
        self.result_wheel = QLabel()
        self.result_speed = QLabel()
        self.result_turn = QLabel()
        form.addRow('タイヤの速さの上限', self.result_wheel)
        form.addRow('ロボットの最高速度(まっすぐ)', self.result_speed)
        form.addRow('その場で回る最高の速さ', self.result_turn)
        return group

    def max_wheel(self):
        return sf.max_wheel_speed(self.motor_max.value(), self.wheel.value())

    def update_result(self):
        wheel = self.max_wheel()
        speed = wheel * self.radius.value() / 1000
        reach = math.hypot(self.length.value(), self.width.value()) / 2000   # 中心からモジュール [m]
        self.result_wheel.setText(f'{wheel:.2f} rad/s')
        self.result_speed.setText(f'{speed:.2f} m/s({speed * 3.6:.1f} km/h)')
        turn = speed / reach
        self.result_turn.setText(f'{turn:.2f} rad/s({math.degrees(turn):.0f} °/s)')


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
