import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")

        # Create the subscription.
        # This tells the node to listen for messages of type String,
        # on the "chatter" topic. When a message is received,
        # the msgCallback function is called.
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)

    def msgCallback(self, msg):
        # This function is the "receiver". It runs every time a message is heard.
        self.get_logger().info("I heard: '%s'" % msg.data)



def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


# FIX: Corrected from '__mainn__' to '__main__'
if __name__ == '__main__':
    main()
