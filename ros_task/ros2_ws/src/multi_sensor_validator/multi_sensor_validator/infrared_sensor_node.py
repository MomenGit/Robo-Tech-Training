from random import randint
from std_msgs.msg import Int32
from sensor_node import SensorNode
from utility import Utility


class InfraredSensorNode(SensorNode):
    """Node that simulates an infrared sensor by publishing random range values."""

    def __init__(self, timer_period=2, queue_size=10):
        super().__init__(sensor_type='infrared', msg_type=Int32,
                         timer_period=Utility.timer_period, queue_size=Utility.queue_size)

    def generate_data(self):
        return randint(10, 200)  # Simulated ultrasonic range in cm


def main(args=None):
    Utility.run_node(args=args, node_factory=lambda: InfraredSensorNode(
        timer_period=Utility.timer_period, queue_size=Utility.queue_size))


if __name__ == '__main__':
    main()
