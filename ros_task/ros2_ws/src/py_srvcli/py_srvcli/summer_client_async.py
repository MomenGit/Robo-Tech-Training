from interfaces.srv import AddThreeInts                            # CHANGE

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SummerClientAsync(Node):

    def __init__(self):
        super().__init__('summer_client_async')
        self.cli = self.create_client(
            AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.req.c = 1                                                      # CHANGE
        return self.cli.call_async(self.req)


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_client = SummerClientAsync()
            future = minimal_client.send_request()
            rclpy.spin_until_future_complete(minimal_client, future)
            response = future.result()
            minimal_client.get_logger().info(
                # CHANGE
                'Result of add_three_ints: for %d + %d + %d = %d' %
                # CHANGE
                (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
