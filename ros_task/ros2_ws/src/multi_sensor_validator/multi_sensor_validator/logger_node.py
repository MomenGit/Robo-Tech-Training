from multi_sensor_validator.utility import Utility
from rclpy.node import Node
from std_msgs.msg import String

class LoggerNode(Node):
    """Node that simulates an logger by publishing random range values."""
    
    def __init__(self):
        super().__init__('logger_node')
        self.get_logger().info('Logger Node has been started.')
        self.subscription = self.create_subscription(String, 'validation_result',self.log_callback, Utility.queue_size)

    def log_callback(self,msg):
        # self.get_logger().info(msg.data)
        print(msg.data)


def main(args=None):
    Utility.run_node(args=args,node_factory=lambda:LoggerNode())

if __name__ == '__main__':
    main()
