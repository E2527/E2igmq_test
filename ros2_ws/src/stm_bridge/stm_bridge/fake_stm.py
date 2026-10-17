"""
実機の代わりにバス上の STM32 のふりをするシミュレーター(動作確認用).

疑似端末(pty)を作って --link のパスに置く。ブリッジの port をそのパスにすると、
実機がなくても Pi 側の通信を試せる。

    ros2 run stm_bridge fake_stm --ids 1 2
    ros2 run stm_bridge bridge --ros-args -p port:=/tmp/stm_bus -p "device_ids:=[1, 2]"

返す状態は [受信回数, 受け取った指令値...]。空の指令(= 止まる)のときは [受信回数] だけを返す。
"""

import argparse
import os
import select
import tty

from stm_bridge import protocol


def main():
    parser = argparse.ArgumentParser(description='バス上の STM32 のふりをする')
    parser.add_argument('--ids', type=int, nargs='+', default=[1], help='応答する ID')
    parser.add_argument('--link', default='/tmp/stm_bus', help='疑似端末へのリンクを置くパス')
    args = parser.parse_args()

    master_fd, slave_fd = os.openpty()
    tty.setraw(slave_fd)
    if os.path.lexists(args.link):
        os.remove(args.link)
    os.symlink(os.ttyname(slave_fd), args.link)
    print(f'{args.link} で ID {args.ids} の STM32 として待っています(Ctrl+C で終了)', flush=True)

    frame_parser = protocol.FrameParser()
    counts = {i: 0 for i in args.ids}
    try:
        while True:
            ready, _, _ = select.select([master_fd], [], [], 1.0)
            if not ready:
                continue
            for frame in frame_parser.feed(os.read(master_fd, 256)):
                if frame.dev_id not in counts or frame.cmd & protocol.REPLY_FLAG:
                    continue
                counts[frame.dev_id] += 1
                if frame.cmd == protocol.CMD_DATA:
                    state = [float(counts[frame.dev_id])] + protocol.unpack_floats(frame.payload)
                    reply = protocol.pack_floats(state)
                else:
                    reply = b''
                os.write(master_fd, protocol.encode(
                    frame.dev_id, frame.cmd | protocol.REPLY_FLAG, reply))
    except KeyboardInterrupt:
        pass
    finally:
        os.remove(args.link)
        os.close(master_fd)
        os.close(slave_fd)


if __name__ == '__main__':
    main()
