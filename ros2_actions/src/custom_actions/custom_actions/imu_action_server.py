from custom_actions.imu import IMUModel
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import time

from custom_action_interfaces.action import IMU


class IMUActionServer(Node):

    def __init__(self):
        super().__init__('IMU_action_server')
        self._action_server = ActionServer(
            self,
            IMU,
            'IMU',
            self.execute_callback)
        self.imu = IMUModel()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = IMU.Feedback()
        feedback_msg.covered_angles = [0]

        for i in self.imu.calculate_angle_deviation(goal_handle.request.coverage, 30):
            feedback_msg.covered_angles.append(i)
            self.get_logger().info(
                f"Feedback: {feedback_msg.covered_angles}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = IMU.Result()
        result.results = feedback_msg.covered_angles
        return result


def main(args=None):
    try:
        with rclpy.init(args=args):
            IMU_action_server = IMUActionServer()

            rclpy.spin(IMU_action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
