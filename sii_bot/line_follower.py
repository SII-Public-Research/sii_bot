# This script 
# following sensor
#   - WHITE = 1
#   - BLACK = 0
import rclpy
import rclpy.node

from geometry_msgs.msg import Twist

import threading

import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_FOLLOWER_LEFT = 25
_FOLLOWER_MIDDLE = 24
_FOLLOWER_RIGHT = 23


# Define a class containing 3 following sensors 
class FollowSensor:
    def __init__(self, left_pin, middle_pin, right_pin):
        self._left_pin = left_pin
        self._middle_pin = middle_pin
        self._right_pin = right_pin

        GPIO.setup(self._left_pin, GPIO.IN)
        GPIO.setup(self._middle_pin, GPIO.IN)
        GPIO.setup(self._right_pin, GPIO.IN)

def get_values(self):
    left_value = GPIO.input(self._left_pin)
    middle_value = GPIO.input(self._middle_pin)
    right_value = GPIO.input(self._right_pin)

    return (left_value, middle_value, right_value)

# Define a class that will publish a Twist depending on values from following sensors
class LineFollower(rclpy.node.Node):

    def __init__(self):
        super().__init__('line_follower')
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        self.publisher

        self._followers = FollowSensor(_FOLLOWER_LEFT, _FOLLOWER_MIDDLE, _FOLLOWER_RIGHT)

        self._linear_velocity = 0
        self._angular_velocity = 0

    def run(self):
        


        while rclpy.ok():
            values = self._followers.get_values()
            match values:
                case (0, 0, 0):
                    print('Je suis dans le noir !')
                    self._linear_velocity = 0
                    self._angular_velocity = 0
                case (0, 0, 1):
                    print('Je derive a fond vers la gauche, a droite toute !')
                    self._linear_velocity = 0
                    self._angular_velocity = 2
                case (0, 1, 1):
                    print('Je derive un peu vers la gauche, a droite !')
                    self._linear_velocity = 2
                    self._angular_velocity = 1
                case (1, 1, 1):
                    print('On va tout droit !')
                    self._linear_velocity = 2
                    self._angular_velocity = 0
                case (1, 1, 0):
                    print('Je derive un peu vers la droite, a gauche !')
                    self._linear_velocity = 2
                    self._angular_velocity = -1
                case (1, 0, 0):
                    print('Je derive a fond vers la droite, a gauche toute ! ')
                    self._linear_velocity = 0
                    self._angular_velocity = -2
                case _:
                    print('Cas bizarre, je ne sais pas quoi faire !')
                    self._linear_velocity = 0
                    self._angular_velocity = 0
            
            twist = Twist()
            twist.linear.x = self._linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self._angular_velocity

            self.publisher.publish(twist)
            


def main(args=None):

    rclpy.init(args=args)

    line_follower = LineFollower()

    thread = threading.Thread(target=rclpy.spin, args=(line_follower, ), daemon=True)
    thread.start()

    # Run node. This will block
    line_follower.run()

    thread.join()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
   main()
