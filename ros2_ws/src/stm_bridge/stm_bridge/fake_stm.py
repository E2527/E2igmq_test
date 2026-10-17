"""
実機の代わりに CAN バス上の STM32 のふりをするシミュレーター(動作確認用).

firmware/stm32_can の STM32 と同じ動きをする:
  - 0x100 + ID の指令を受け取る
  - period_ms ごとに 0x200 + ID で状態を送る
  - 指令が watchdog_ms 以上届かない、または空の指令なら「止まる」

状態は、動いているときは受け取った指令値をそのまま、止まっているときは [0.0, 0.0] を返す。

    ros2 run stm_bridge fake_stm --ids 1 2                        # カーネルに CAN がない PC 向け
    ros2 run stm_bridge bridge --ros-args -p can_interface:=sim -p "device_ids:=[1, 2]"

    ros2 run stm_bridge fake_stm --ids 1 --interface vcan0        # Pi で仮想 CAN(vcan)を使う場合
"""

import argparse
import time

from stm_bridge import can_bus, protocol


def main():
    parser = argparse.ArgumentParser(description='CAN バス上の STM32 のふりをする')
    parser.add_argument('--ids', type=int, nargs='+', default=[1], help='基板の ID')
    parser.add_argument('--interface', default='sim', help='sim、または vcan0 / can0 など')
    parser.add_argument('--period-ms', type=float, default=10.0, help='状態を送る周期')
    parser.add_argument('--watchdog-ms', type=float, default=100.0, help='指令が途切れたら止まる時間')
    args = parser.parse_args()

    bus = can_bus.open_bus(args.interface, protocol.COMMAND_FILTER, role='device')
    print(f'{args.interface} で ID {args.ids} の STM32 として動いています(Ctrl+C で終了)', flush=True)

    commands = {i: (0.0, []) for i in args.ids}   # ID -> (受け取った時刻, 指令値)
    period = args.period_ms / 1000.0
    next_send = time.monotonic()
    try:
        while True:
            frame = bus.recv(max(0.0, next_send - time.monotonic()))
            if frame is not None:
                dev_id = protocol.device_of(frame[0], protocol.COMMAND_BASE)
                if dev_id in commands:
                    commands[dev_id] = (time.monotonic(), protocol.unpack_floats(frame[1]))
            now = time.monotonic()
            if now < next_send:
                continue
            next_send += period
            for dev_id, (received, values) in commands.items():
                running = values and now - received < args.watchdog_ms / 1000.0
                state = values if running else [0.0, 0.0]
                try:
                    bus.send(protocol.state_id(dev_id), protocol.pack_floats(state))
                except OSError:
                    pass   # ブリッジがまだ起動していない(sim)など
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == '__main__':
    main()
