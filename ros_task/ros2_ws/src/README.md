# ROS Task

## Multi-sensor Validator

Simulation of two range sensors and processing their values using multiple ROS2 nodes.

### Usage

```bash
source install/setup.bash
colcon build
# run individual nodes in separate terminal windows
ros2 run mutli_sensor_validator ultrasonic_sensor_node
ros2 run mutli_sensor_validator infrared_sensor_node
ros2 run mutli_sensor_validator validator_node
ros2 run mutli_sensor_validator logger_node
```

### Topics

- `/ultrasonic_range` - Ultrasonic sensor measurements
- `/infrared_range` - Infrared sensor measurements
- `/validation_result` - Result of validation of sensors measurements

### Technical Documentation

#### Libraries Used

1. **ROS2 (rclpy)** - Robotics middleware for communication
2. **PyGame** - For joystick input
3. **Standard Python libraries** - No external dependencies for core functionality

#### Design Choices

1. **Observer Pattern** - ROS2 pub/sub naturally implements this
2. **Single Responsibility** - Each class has a clear, focused purpose
3. **Open/Closed Principle** - Easy to extend with new sensors
4. **Dependency Inversion Principle** - Easy to inject the names of sensor topics into the validator or inject the sensors themselves
5. **Liskov Substitution Principle** - Easy to substitute SensorNode with different types of sensors (eg. UltraSonicSensorNode, InfraredSensorNode...)
6. **Interface Segregation Principle** - Having the IPublisher interface to implement the publish_msg functionality

## ROV Control System

A ROS2-based ROV control system with joystick input, PWM mapping, and signal smoothing.

### Features

- Joystick data simulation/publishing
- PWM signal mapping
- Multiple smoothing strategies (Linear, Exponential)
- Modular, OOP-based architecture

### Usage

```bash
source install/setup.bash
colcon build
# run individual nodes in separate terminal windows
ros2 run joystick joystick_node
ros2 run joystick navigation_node
ros2 run joystick thruster_node
```

### Smoothing Strategies

- **Linear Smoothing**

  - Divides changes into equal steps
  - Configurable number of steps
  - Predictable, constant rate

- **Exponential Smoothing**
  - Applies smoothing factor (alpha)
  - Recent values have more weight
  - Smoother transitions

#### Changing Strategies

- **Modify the navigation node**:

```python
navigation_node.set_smoothing_strategy(ExponentialSmoothing(alpha=0.2))
```

### Topics

- `/joystick_data` - Raw joystick input
- `/smoothed_pwm` - Processed PWM signals

### Technical Documentation

#### Libraries Used

1. **ROS2 (rclpy)** - Robotics middleware for communication
2. **Standard Python libraries** - No external dependencies for core functionality

#### Design Choices

1. **Strategy Pattern** - For interchangeable smoothing algorithms
2. **Observer Pattern** - ROS2 pub/sub naturally implements this
3. **Single Responsibility** - Each class has a clear, focused purpose
4. **Open/Closed Principle** - Easy to extend with new smoothing strategies
5. **Dependency Injection** - Easy to inject the smoothing startegy into navigation

#### Key Features

- **Modularity**: Each component can be developed/tested independently
- **Extensibility**: Easy to add new smoothing strategies or hardware interfaces
- **Safety**: PWM value validation and smoothing prevent thruster damage
- **Configurability**: Parameters easily adjustable for different ROV configurations

This implementation provides a solid foundation that can be extended with real hardware interfaces, more sophisticated control algorithms, and additional safety features.
