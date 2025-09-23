import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import pygame
from sensor_msgs.msg import Joy
import random


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().warn("No joystick detected! Running in simulation mode.")
            self.simulation_mode = True
        else:
            self.simulation_mode = False
            # Take the first joystick
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(
                f"Joystick initialized: {self.joystick.get_name()}")

        # Publishers
        self.publisher_ = self.create_publisher(Joy, 'joystick_data', 10)
        msg = self.set_simulated_joystick_msg(
        ) if self.simulation_mode else self.set_joystick_msg()

        self.timer = self.create_timer(
            2, self.publish_joystick_data)  # 10Hz

    def read_joystick(self):
        if self.simulation_mode:
            return self.set_simulated_joystick_msg()
        else:
            return self.set_joystick_msg()

    def set_joystick_msg(self):
        pygame.event.pump()  # Update joystick events
        msg = Joy()

        # Read button states
        msg.buttons = [self.joystick.get_button(i)
                       for i in range(self.joystick.get_numbuttons())]

        # Read axis values (analog sticks)
        msg.axes = [self.joystick.get_axis(
            i) for i in range(self.joystick.get_numaxes())]

        return msg

    def set_simulated_joystick_msg(self):
        # Random button states (simulate up to 4 buttons)
        msg = Joy()
        msg.buttons = [random.randint(0, 1) for _ in range(4)]

        # Random analog values (simulate 2 axes: left stick x,y)
        msg.axes = [random.uniform(-1.0, 1.0) for _ in range(2)]

        return msg

    def publish_joystick_data(self):
        msg = self.read_joystick()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joystick data: {msg.axes}')


def main(args=None):
    """Entry point for running a sensor node."""
    try:
        with rclpy.init(args=args):
            node = JoystickNode()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        pygame.quit()


if __name__ == '__main__':
    main()
