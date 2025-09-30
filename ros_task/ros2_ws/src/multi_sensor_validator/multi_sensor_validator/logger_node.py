from sensor_validator_interfaces.msg import SensorProperties
from multi_sensor_validator.utility import Utility
from rclpy.node import Node
from std_msgs.msg import String


class LoggerNode(Node):
    """Node that simulates an logger by publishing random range values."""

    def __init__(self):
        super().__init__('logger_node')
        self.get_logger().info('Logger Node has been started.')
        # self.subscription = self.create_subscription(
        self.subscription = self.create_subscription(
            SensorProperties, 'sensor_properties', self.sensor_props_callback, Utility.queue_size)

    def sensor_props_callback(self, msg):
        quality = f"Quality: {msg.quality}"
        data = ""
        is_range100 = True
        path_state = ""
        for range in msg.sensor_values:
            is_range100 = False if range.range < 100.0 else is_range100

        path_state = "Path clear" if is_range100 else "Obstacle detected"

        if msg.quality >= 50:
            for sensor in msg.sensor_values:
                sensor = f"{sensor.header.frame_id.capitalize()}: range={sensor.range}"
                data = f"{sensor}" if not data else f"{data}, {sensor}"
            data = f"{quality} -> {data} -> {path_state}"
        else:
            data = f"{quality} -> Quality rejected"

        self.get_logger().info(data)


def main(args=None):
    Utility.run_node(args=args, node_factory=lambda: LoggerNode())


if __name__ == '__main__':
    main()
