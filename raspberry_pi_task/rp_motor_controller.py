#!/usr/bin/env python3
"""
Rover Control System
Raspberry Pi 4 with L298 Motor Driver
Keyboard-controlled 4-wheel rover
"""

import RPi.GPIO as GPIO
import time
import sys
import tty
import termios


class RoverController:
    def __init__(self):
        # L298 Motor Driver GPIO pin assignments
        # Motor A (Left side motors)
        self.IN1 = 18  # Motor A direction pin 1
        self.IN2 = 19  # Motor A direction pin 2
        self.ENA = 12  # Motor A speed control (PWM)

        # Motor B (Right side motors)
        self.IN3 = 20  # Motor B direction pin 1
        self.IN4 = 21  # Motor B direction pin 2
        self.ENB = 13  # Motor B speed control (PWM)

        # Motor speed (0-100)
        self.speed = 75

        # Setup GPIO
        self.setup_gpio()

    def setup_gpio(self):
        """Initialize GPIO pins and PWM"""
        GPIO.setmode(GPIO.BCM)

        # Setup motor control pins
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4], GPIO.OUT)
        GPIO.setup([self.ENA, self.ENB], GPIO.OUT)

        # Initialize PWM for speed control
        self.pwm_a = GPIO.PWM(self.ENA, 1000)  # 1kHz frequency
        self.pwm_b = GPIO.PWM(self.ENB, 1000)

        self.pwm_a.start(0)
        self.pwm_b.start(0)

        print("GPIO initialized successfully")

    def stop_motors(self):
        """Stop both motors"""
        GPIO.output([self.IN1, self.IN2, self.IN3, self.IN4], GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        print("Motors stopped")

    def move_forward(self):
        """Move rover forward"""
        # Left motors forward
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)

        # Right motors forward
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        # Set speed
        self.pwm_a.ChangeDutyCycle(self.speed)
        self.pwm_b.ChangeDutyCycle(self.speed)
        print(f"Moving forward at {self.speed}% speed")

    def move_backward(self):
        """Move rover backward"""
        # Left motors backward
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)

        # Right motors backward
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

        # Set speed
        self.pwm_a.ChangeDutyCycle(self.speed)
        self.pwm_b.ChangeDutyCycle(self.speed)
        print(f"Moving backward at {self.speed}% speed")

    def turn_left(self):
        """Turn rover left (right motors forward, left motors backward)"""
        # Left motors backward
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)

        # Right motors forward
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

        # Set speed
        self.pwm_a.ChangeDutyCycle(self.speed)
        self.pwm_b.ChangeDutyCycle(self.speed)
        print(f"Turning left at {self.speed}% speed")

    def turn_right(self):
        """Turn rover right (left motors forward, right motors backward)"""
        # Left motors forward
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)

        # Right motors backward
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)

        # Set speed
        self.pwm_a.ChangeDutyCycle(self.speed)
        self.pwm_b.ChangeDutyCycle(self.speed)
        print(f"Turning right at {self.speed}% speed")

    def adjust_speed(self, change):
        """Adjust motor speed"""
        self.speed = max(0, min(100, self.speed + change))
        print(f"Speed adjusted to {self.speed}%")

    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_motors()
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")


def get_key():
    """Get keyboard input without pressing Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.cbreak(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main():
    """Main control loop"""
    print("=" * 50)
    print("ROVER CONTROL SYSTEM")
    print("=" * 50)
    print("Controls:")
    print("  W - Forward")
    print("  S - Backward")
    print("  A - Turn Left")
    print("  D - Turn Right")
    print("  + - Increase Speed")
    print("  - - Decrease Speed")
    print("  SPACE - Stop")
    print("  Q - Quit")
    print("=" * 50)

    rover = RoverController()

    try:
        while True:
            key = get_key().lower()

            if key == 'w':
                rover.move_forward()
            elif key == 's':
                rover.move_backward()
            elif key == 'a':
                rover.turn_left()
            elif key == 'd':
                rover.turn_right()
            elif key == ' ':
                rover.stop_motors()
            elif key == '+' or key == '=':
                rover.adjust_speed(10)
            elif key == '-':
                rover.adjust_speed(-10)
            elif key == 'q':
                print("Exiting rover control...")
                break
            else:
                print(f"Unknown command: {key}")

            time.sleep(0.1)  # Small delay to prevent rapid commands

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rover.cleanup()
        print("Rover control system terminated")


if __name__ == "__main__":
    main()
