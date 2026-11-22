"""
実機の代わりに CAN バス上の STM32 のふりをするシミュレーター(動作確認用).

firmware/stm32_can の STM32 と同じ動きをする:
  - 0x100 + ID の指令(モーター 2 個の目標速度 [rad/s])を受け取る
  - 速度制御されたモーターのように、実際の速度が目標速度に time_constant でなめらかに近づく
  - period_ms ごとに 0x200 + ID で状態(モーター 2 個の実際の速度 [rad/s])を送る
  - 指令が watchdog_ms 以上届かない、または空の指令なら目標を 0 にする(止まる)
  - 0x300 + ID のゲインの読み書きに、0x400 + ID で答える(モーター 2 個の版と同じ番号。値は覚えるだけ)

    ros2 run stm_bridge fake_stm --ids 1 2                        # カーネルに CAN がない PC 向け
    ros2 run stm_bridge bridge --ros-args -p can_interface:=sim -p "device_ids:=[1, 2]"

    ros2 run stm_bridge fake_stm --ids 1 --interface vcan0        # Pi で仮想 CAN(vcan)を使う場合
"""

import argparse
import time

from stm_bridge import can_bus, protocol

# ゲインの初期値(firmware/motor_node_f303k8/Core/Inc/app_config.h と同じ)と範囲(control_params.c と同じ)
FAKE_PARAMS = {0: (1.0, None), 1: (0.02, (0.0, 5.0)), 2: (0.5, (0.0, 50.0)),
               3: (1.0 / 30.0, (0.0, 1.0)), 4: (0.95, (0.0, 1.0))}


def answer_param(params, data):
    """ゲインの頼みごとに答える(params は {番号: [値, 範囲]})。返事のバイト列を返す."""
    try:
        param_id, value = protocol.unpack_param_request(data)
    except ValueError:
        return protocol.pack_param_reply(data[0] if data else 0, protocol.PARAM_BAD_REQUEST, 0.0)
    if param_id not in params:
        return protocol.pack_param_reply(param_id, protocol.PARAM_UNKNOWN, 0.0)
    entry = params[param_id]
    status = protocol.PARAM_OK
    if value is not None:
        if entry[1] is None:
            status = protocol.PARAM_READ_ONLY
        elif not entry[1][0] <= value <= entry[1][1]:
            status = protocol.PARAM_OUT_OF_RANGE
        else:
            entry[0] = value
    return protocol.pack_param_reply(param_id, status, entry[0])


def main():
    parser = argparse.ArgumentParser(description='CAN バス上の STM32 のふりをする')
    parser.add_argument('--ids', type=int, nargs='+', default=[1], help='基板の ID')
    parser.add_argument('--interface', default='sim', help='sim、または vcan0 / can0 など')
    parser.add_argument('--period-ms', type=float, default=10.0, help='状態を送る周期')
    parser.add_argument('--watchdog-ms', type=float, default=100.0, help='指令が途切れたら止まる時間')
    parser.add_argument('--time-constant', type=float, default=0.1,
                        help='速度が目標に近づく速さ [s](実機のモーター + PID の応答の代わり)')
    args = parser.parse_args()

    bus = can_bus.open_bus(args.interface, protocol.COMMAND_FILTER, role='device')
    print(f'{args.interface} で ID {args.ids} の STM32 として動いています(Ctrl+C で終了)', flush=True)

    commands = {i: (0.0, []) for i in args.ids}   # ID -> (受け取った時刻, 目標速度)
    params = {i: {k: list(v) for k, v in FAKE_PARAMS.items()} for i in args.ids}
    speeds = {i: [0.0] * protocol.MAX_FLOATS for i in args.ids}
    period = args.period_ms / 1000.0
    gain = min(1.0, period / args.time_constant)
    next_send = time.monotonic()
    try:
        while True:
            frame = bus.recv(max(0.0, next_send - time.monotonic()))
            if frame is not None:
                dev_id = protocol.device_of(frame[0], protocol.COMMAND_BASE)
                if dev_id in commands:
                    commands[dev_id] = (time.monotonic(), protocol.unpack_floats(frame[1]))
                dev_id = protocol.device_of(frame[0], protocol.PARAM_REQUEST_BASE)
                if dev_id in params:
                    try:
                        bus.send(protocol.param_reply_id(dev_id),
                                 answer_param(params[dev_id], frame[1]))
                    except OSError:
                        pass
            now = time.monotonic()
            if now < next_send:
                continue
            next_send += period
            for dev_id, (received, values) in commands.items():
                running = now - received < args.watchdog_ms / 1000.0
                targets = values if running else []
                speed = speeds[dev_id]
                for k in range(len(speed)):
                    target = targets[k] if k < len(targets) else 0.0
                    speed[k] += gain * (target - speed[k])
                try:
                    bus.send(protocol.state_id(dev_id), protocol.pack_floats(speed))
                except OSError:
                    pass   # ブリッジがまだ起動していない(sim)など
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == '__main__':
    main()
