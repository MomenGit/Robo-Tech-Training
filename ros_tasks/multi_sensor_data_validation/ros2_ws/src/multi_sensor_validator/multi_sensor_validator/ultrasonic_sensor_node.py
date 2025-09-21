from random import randint 
from multi_sensor_validator.utility import Utility
from multi_sensor_validator.sensor_node import SensorNode
from std_msgs.msg import Int32


class UltrasonicSensorNode(SensorNode):
    """Node that simulates an ultrasonic sensor by publishing random range values."""
    
    def __init__(self):
        super().__init__(sensor_type='ultrasonic',msg_type=Int32 , timer_period=Utility.timer_period, queue_size=Utility.queue_size)

    def generate_data(self):
       return randint(10, 200)  # Simulated ultrasonic range in cm


def main(args=None):
    Utility.run_node(args=args,node_factory=lambda:UltrasonicSensorNode())


if __name__ == '__main__':
    main()
