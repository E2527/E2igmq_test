"""
いくつかの速さの指令から、優先度のいちばん高いものを選ぶ計算.

ROS に依存しないので、PC だけでテストできる(test/test_cmd_mux.py)。ノードは cmd_vel_mux.py。

    手で操縦する(ロボットコンソール・ゲームパッド)… cmd_vel_teleop(優先度 高)
    自動操縦(Nav2)                               … cmd_vel_nav(優先度 低)

手で操縦している間は、自動操縦の指令を捨てる。手で操縦する指令が timeout 秒届かなければ、
自動操縦の指令を通すようにもどる。
"""


class Source:

    def __init__(self, name, priority, timeout_s):
        self.name = name
        self.priority = priority
        self.timeout_s = timeout_s
        self.last_time = None

    def alive(self, now):
        return self.last_time is not None and now - self.last_time < self.timeout_s


class CmdMux:

    def __init__(self, sources):
        """指令を出すものの一覧 [(名前, 優先度, timeout [s])]。優先度は大きいほど強い."""
        self.sources = {name: Source(name, priority, timeout)
                        for name, priority, timeout in sources}

    def active(self, now):
        """今、指令を通しているものの名前(どれも届いていなければ None)."""
        alive = [s for s in self.sources.values() if s.alive(now)]
        if not alive:
            return None
        return max(alive, key=lambda s: s.priority).name

    def on_input(self, name, now):
        """指令が届いた(name から)。通すなら True、捨てるなら False."""
        source = self.sources[name]
        source.last_time = now
        return self.active(now) == name
