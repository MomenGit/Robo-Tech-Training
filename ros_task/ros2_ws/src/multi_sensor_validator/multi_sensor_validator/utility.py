import rclpy
import rclpy
from rclpy.executors import ExternalShutdownException


class Utility:
    timer_period = 1  # seconds
    queue_size = 10

    @staticmethod
    def run_node(node_factory,args=None):
        """Entry point for running a sensor node."""
        try:
            with rclpy.init(args=args):
                sensor_node = node_factory()
    
                rclpy.spin(sensor_node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
