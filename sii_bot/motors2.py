#!/usr/bin/env python

import rclpy
import rclpy.node

from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO

import threading
from rclpy.duration import Duration

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_FREQUENCY = 20

def _clip(value, minimum, maximum):
   """Ensure value is between minimum and maximum."""

   if value < minimum:
       return minimum
   elif value > maximum:
       return maximum
   return value

class Motor:
   def __init__(self, forward_pin, backward_pin):
       GPIO.setup(forward_pin, GPIO.OUT)
       GPIO.setup(backward_pin, GPIO.OUT)

       self._forward_pwm = GPIO.PWM(forward_pin, _FREQUENCY)
       self._backward_pwm = GPIO.PWM(backward_pin, _FREQUENCY)

   def move(self, speed_percent):
       speed = _clip(abs(speed_percent), 0, 100)

       # Positive speeds move wheels forward, negative speeds
       # move wheels backward
       if speed_percent < 0:
           self._backward_pwm.start(speed)
           self._forward_pwm.start(0)
       else:
           self._forward_pwm.start(speed)
           self._backward_pwm.start(0)

class Driver(rclpy.node.Node):

    def __init__(self):
        print('Starting driver init')
        super().__init__('driver')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self._velocity_received_callback,
            10)
        self.subscription  # prevent unused variable warning

        # store parameters and current time, visible with command ros2 param list
        self._last_received = self.get_clock().now()

        self.declare_parameter('~timeout', 2)
        self._timeout = self.get_parameter('~timeout').value
        self.declare_parameter('~rate', 2)
        self._rate = self.get_parameter('~rate').value
        self.declare_parameter('~max_speed', 0.5)
        self._max_speed = self.get_parameter('~max_speed').value
        self.declare_parameter('~wheel_base', 0.091)
        self._wheel_base = self.get_parameter('~wheel_base').value

        print("~timeout = ", self._timeout)
        print("~rate = ", self._rate)
        print("~max_speed = ", self._max_speed)
        print("~wheel_base = ", self._wheel_base)


        # Assign pins to motors. These may be distributed
        # differently depending on how you've built your robot
        self._left_motor = Motor(10, 9)
        self._right_motor = Motor(8, 7)
        self._left_speed_percent = 0
        self._right_speed_percent = 0


    def _velocity_received_callback(self, message):
       """Handle new velocity command message."""
       print('Received something')

       self._last_received = self.get_clock().now()

       # Extract linear and angular velocities from the message
       linear = message.linear.x
       angular = message.angular.z

       # Calculate wheel speeds in m/s
      
       left_speed = linear - angular*self._wheel_base/2
       right_speed = linear + angular*self._wheel_base/2

       # Ideally we'd now use the desired wheel speeds along
       # with data from wheel speed sensors to come up with the
       # power we need to apply to the wheels, but we don't have
       # wheel speed sensors. Instead, we'll simply convert m/s
       # into percent of maximum wheel speed, which gives us a
       # duty cycle that we can apply to each motor.
       self._left_speed_percent = (100 * left_speed/self._max_speed)
       self._right_speed_percent = (100 * right_speed/self._max_speed)

    def run(self):
       """The control loop of the driver."""
       print('On est dans le run')

       rate = self.create_rate(self.get_parameter('~rate').get_parameter_value().integer_value)

       while rclpy.ok():
           print('rclpy est OK')
           # If we haven't received new commands for a while, we
           # may have lost contact with the commander-- stop
           # moving
           now = self.get_clock().now()
           past = self._last_received
           delay = (now - past).nanoseconds * 1e-9
           print('delay = ', delay)
           if delay < self._timeout:
               self._left_motor.move(self._left_speed_percent)
               self._right_motor.move(self._right_speed_percent)
           else:
               self._left_motor.move(0)
               self._right_motor.move(0)
           print('on est avant le sleep')
           rate.sleep()
           print('on est après le sleep')

def main(args=None):

    rclpy.init(args=args)

    driver_node = Driver()

    print('launching thread')
    thread = threading.Thread(target=rclpy.spin, args=(driver_node, ), daemon=True)
    thread.start()
    print('thread launched')

    # Run driver. This will block
    driver_node.run()

    thread.join()
    GPIO.cleanup()
    rclpy.shutdown()
    

if __name__ == '__main__':
   main()