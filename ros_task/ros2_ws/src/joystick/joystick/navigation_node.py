import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray
from joystick.pwm_mapper import PWMMapper
from sensor_msgs.msg import Joy
from joystick.smoothing_strategy import SmoothingStrategy, LinearSmoothing, ExponentialSmoothing


class NavigationNode(Node):
    def __init__(self, smoothing_strategy: SmoothingStrategy, pwm_mapper=PWMMapper.map_to_pwm):
        super().__init__('navigation_node')

        # Configuration
        self.thruster_count = 8
        self.set_smoothing_strategy(smoothing_strategy)  # Default strategy

        # Initialize current PWM values
        self.current_pwm = [1500] * self.thruster_count

        # Publishers and Subscribers
        self.subscription = self.create_subscription(
            Joy,
            'joystick_data',
            self.joystick_callback,
            10)

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'smoothed_pwm',
            10)

        # PWM Mapper
        self.pwm_mapper = pwm_mapper

        self.get_logger().info('Navigation node started')

    def set_smoothing_strategy(self, strategy: SmoothingStrategy):
        """Allow dynamic strategy changes"""
        self.smoothing_strategy = strategy
        self.get_logger().info(
            f'Smoothing strategy changed to {strategy.__class__.__name__}')

    def joystick_callback(self, msg: Joy):
        """Process joystick data and apply smoothing"""
        # Simple mapping: each joystick axis controls 2 thrusters
        target_pwm = []

        for i in range(min(len(msg.axes), self.thruster_count // 2)):
            axis_value = msg.axes[i]
            pwm_value = self.pwm_mapper(axis_value)
            # Assigning to two thrusters for simplicity
            target_pwm.extend([pwm_value, pwm_value])

        # Filling remaining thrusters with neutral
        while len(target_pwm) < self.thruster_count:
            target_pwm.append(float(PWMMapper.mid_pwm))

        # Apply smoothing
        smoothed_pwm = []
        for i in range(self.thruster_count):
            smoothed_value = self.smoothing_strategy.smooth(
                self.current_pwm[i], target_pwm[i], 10
            )
            smoothed_pwm.extend(smoothed_value)
            self.current_pwm[i] = smoothed_pwm[-1]

        print(self.current_pwm)

        # Publish smoothed PWM values
        pwm_msg = Float32MultiArray()
        pwm_msg.data = smoothed_pwm
        self.publisher_.publish(pwm_msg)

        self.get_logger().info(f'Published smoothed PWM: {smoothed_pwm}')


def main(args=None):
    """Entry point for running a sensor node."""
    try:
        with rclpy.init(args=args):
            node = NavigationNode(
                smoothing_strategy=LinearSmoothing(), pwm_mapper=PWMMapper.map_to_pwm)

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
