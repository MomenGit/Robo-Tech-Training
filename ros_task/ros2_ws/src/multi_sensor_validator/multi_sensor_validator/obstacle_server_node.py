import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_validator_interfaces.srv import CheckObstacle  # CHANGE


class ObstacleServerNode(Node):
    def __init__(self):
        super().__init__('summer_service')
        self.srv = self.create_service(
            CheckObstacle, 'check_obstacle', self.check_obstacle_callback)

    def check_obstacle_callback(self, request, response):
        response.obstacle_detected = False
        detect_msg = ""

        for range in request.sensor_values:
            if range.range < 50:
                response.obstacle_detected = True
                detect_msg = f"Obstacle detected by {range.header.frame_id} at {range.range} cm"
                break

        response.message = detect_msg if response.obstacle_detected else "Path is clear"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    try:
        with rclpy.init(args=args):
            obstacle_server = ObstacleServerNode()

            rclpy.spin(obstacle_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
