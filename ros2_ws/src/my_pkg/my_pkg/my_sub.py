import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MySub(Node):
    def __init__(self):
        super().__init__('my_sub')
        # 'chatter' を受け取ったら on_msg を呼ぶ
        self.sub = self.create_subscription(String, 'chatter', self.on_msg, 10)

    def on_msg(self, msg):
        self.get_logger().info(f'受信: {msg.data}')


def main():
    rclpy.init()
    node = MySub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()