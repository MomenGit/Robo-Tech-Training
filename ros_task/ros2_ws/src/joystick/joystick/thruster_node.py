import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray


class ThrusterNode(Node):
    def __init__(self):
        super().__init__('thruster_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'smoothed_pwm',
            self.pwm_callback,
            10)

        self.get_logger().info('Thruster node started')

    def pwm_callback(self, msg: Float32MultiArray):
        """Receive smoothed PWM values and send to thrusters"""
        # In a real system, this would interface with thruster hardware
        # For simulation, we just log the values
        pwm_values = msg.data

        # Validate PWM values (safety check)
        valid_pwm = all(1000 <= pwm <= 2000 for pwm in pwm_values)

        if valid_pwm:
            self.get_logger().info(f'Thrusters receiving PWM: {pwm_values}')
            # Here you would send commands to actual thrusters
        else:
            self.get_logger().error('Invalid PWM values detected!')


def main(args=None):
    """Entry point for running a sensor node."""
    try:
        with rclpy.init(args=args):
            node = ThrusterNode()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
