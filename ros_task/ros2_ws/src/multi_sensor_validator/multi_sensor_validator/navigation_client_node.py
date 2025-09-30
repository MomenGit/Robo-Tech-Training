import random
from sensor_validator_interfaces.srv import CheckObstacle
from sensor_msgs.msg import Range
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class NavigationClientNode(Node):

    def __init__(self):
        super().__init__('navigation_client')
        self.cli = self.create_client(
            CheckObstacle, 'check_obstacle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CheckObstacle.Request()
        self.sensor_values = []

    def send_request(self):
        self.sensor_props_populate()
        self.req.sensor_values = self.sensor_values
        request_msg = "Request sent"
        data = ""

        for sensor in self.sensor_values:
            sensor = f"{sensor.header.frame_id.capitalize()}: {sensor.range} cm"
            data = f"{sensor}" if not data else f"{data}, {sensor}"

        self.get_logger().info(f"{request_msg} -> {data}")
        return self.cli.call_async(self.req)

    def sensor_props_populate(self):
        ranges = dict()
        for topic in ["ultrasonic_range", "infrared_range"]:
            ranges[topic] = Range()
            ranges[topic].header.stamp = self.get_clock().now().to_msg()
            ranges[topic].header.frame_id = topic.split("_")[0]
            ranges[topic].radiation_type = Range.ULTRASOUND if "ultrasonic" in topic else Range.INFRARED
            ranges[topic].min_range = 2.0
            ranges[topic].max_range = 200.0 if "ultrasonic" in topic else 150.0
            ranges[topic].range = float(random.uniform(
                40.0, 150.0) if "ultrasonic" in topic else random.uniform(30.0, 120.0))

        # Add to message
        self.sensor_values = [ranges[topic]
                              for topic in ["ultrasonic_range", "infrared_range"]]


def main(args=None):
    try:
        with rclpy.init(args=args):
            navigation_client = NavigationClientNode()

            future = navigation_client.send_request()
            rclpy.spin_until_future_complete(navigation_client, future)

            response = future.result()
            if response is not None:
                navigation_client.get_logger().info(
                    f"Response -> {response.message}")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
