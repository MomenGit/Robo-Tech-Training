from random import randint
from std_msgs.msg import Int32
from multi_sensor_validator.sensor_node import SensorNode 
from multi_sensor_validator.utility import Utility

class InfraredSensorNode(SensorNode):
    """Node that simulates an infrared sensor by publishing random range values."""
    
    def __init__(self):
        super().__init__(sensor_type='infrared',msg_type=Int32 , timer_period=Utility.timer_period, queue_size=Utility.queue_size)

    def generate_data(self):
        return randint(10, 200)  # Simulated ultrasonic range in cm


def main(args=None):
    Utility.run_node(args=args,node_factory=lambda:InfraredSensorNode())


if __name__ == '__main__':
    main()
