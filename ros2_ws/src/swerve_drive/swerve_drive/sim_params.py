"""
シミュレーションの STM32(firmware_modules / omni の firmware_wheels)で、ゲインの読み書きに答える.

実機では stm_bridge が中継する stm<ID>/param と stm<ID>/param_reply を、シミュレーションでも同じように
使えるようにする(ロボットコンソールの「調整」タブを、実機なしで試せる)。

    stm<ID>/param       (std_msgs/Float32MultiArray) … [番号] で読む、[番号, 値] で書く
    stm<ID>/param_reply (std_msgs/Float32MultiArray) … [番号, 結果, 今の値]

番号と結果は stm_bridge の protocol.py(firmware/stm32_can/params.h と同じ)。
"""

from std_msgs.msg import Float32MultiArray
from stm_bridge import protocol


def serve_params(node, sims):
    """ゲインの読み書きに答える(sims は {STM32 の ID: firmware_sim のシミュレーション})."""
    for dev_id, sim in sims.items():
        pub = node.create_publisher(Float32MultiArray, f'stm{dev_id}/param_reply', 10)

        def on_request(msg, sim=sim, pub=pub, dev_id=dev_id):
            try:
                request = protocol.request_from_list(list(msg.data))
            except ValueError as e:
                node.get_logger().warn(f'stm{dev_id}/param: {e}')
                return
            pub.publish(Float32MultiArray(data=protocol.reply_to_list(sim.param(request))))

        node.create_subscription(Float32MultiArray, f'stm{dev_id}/param', on_request, 10)
