from random import randint
from utility import Utility
from sensor_node import SensorNode
from std_msgs.msg import Int32


class UltrasonicSensorNode(SensorNode):
    """Node that simulates an ultrasonic sensor by publishing random range values."""

    def __init__(self, timer_period=2, queue_size=10):
        super().__init__(sensor_type='ultrasonic', msg_type=Int32,
                         timer_period=timer_period, queue_size=queue_size)
        print("Ultrasonic Sensor Node initialized.")
        print(self._publisher)

    def generate_data(self):
        return randint(10, 200)  # Simulated ultrasonic range in cm


def main(args=None):
    Utility.run_node(args=args, node_factory=lambda: UltrasonicSensorNode(
        Utility.timer_period, Utility.queue_size))


if __name__ == '__main__':
    main()
