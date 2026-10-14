import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.count = 0
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg = String()
        msg.data = f'Hello {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1


def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
