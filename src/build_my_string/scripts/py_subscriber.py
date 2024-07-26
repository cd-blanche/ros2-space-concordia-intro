#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from build_my_string.msg import Message

class MessageSubscriber(Node):

  def __init__(self):
    super().__init__('message_subscriber')
    self.subscription = self.create_subscription(
      Message,
      'build_my_string',
      self.listener_callback,
      10
    )
    self.subscription

  def listener_callback(self, msg):
    self.get_logger().info('hearing: "%s"' % msg.message)


def main(args=None):
  rclpy.init(args=args)

  message_subscriber = MessageSubscriber()

  rclpy.spin(message_subscriber)

  message_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

