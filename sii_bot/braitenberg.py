# This script 
# following sensor
#   - WHITE = 1
#   - BLACK = 0
import rclpy
import rclpy.node

from geometry_msgs.msg import Twist

import threading

import RPi.GPIO as GPIO

import time

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_FOLLOWER_LEFT = 25
_FOLLOWER_MIDDLE = 24
_FOLLOWER_RIGHT = 23

pinTrigger1 = 17
pinEcho1 = 18
pinTrigger2 = 23
pinEcho2 = 22

print("Ultrasconic Measurement")

GPIO.setup(pinTrigger1, GPIO.OUT)
GPIO.setup(pinEcho1, GPIO.IN)
GPIO.setup(pinTrigger2, GPIO.OUT)
GPIO.setup(pinEcho2, GPIO.IN)

try:
    while True:
        GPIO.output(pinTrigger1, False)
        GPIO.output(pinTrigger2, False)

        time.sleep(0.5)

        GPIO.output(pinTrigger1, True)
        time.sleep(0.00001)
        GPIO.output(pinTrigger1, False)

        StartTime = time.time()

        while GPIO.input(pinEcho1) == 0:
            StartTime = time.time()

        while GPIO.input(pinEcho1) == 1:
            StopTime = time.time()
            if StopTime - StartTime >= 0.04:
                print("Too close 1!")
                StopTime = StartTime
                break

        ElapsedTime = StopTime - StartTime
        Distance = ElapsedTime * 34326 / 2

        print("Distance1 : %.1f cm" % Distance)

        GPIO.output(pinTrigger1, True)
        time.sleep(0.00001)
        GPIO.output(pinTrigger1, False)

        StartTime = time.time()

        while GPIO.input(pinEcho2) == 0:
            StartTime = time.time()

        while GPIO.input(pinEcho2) == 1:
            StopTime = time.time()
            if StopTime - StartTime >= 0.04:
                print("Too close 1!")
                StopTime = StartTime
                break

        ElapsedTime = StopTime - StartTime
        Distance = ElapsedTime * 34326 / 2

        print("Distance1 : %.1f cm" % Distance)

        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()


# Define a class containing 3 following sensors 
class FollowSensor:
    def __init__(self, pinTrigger1, pinEcho1, pinTrigger2, pinEcho2):
        self.pinTrigger1 = pinTrigger1
        self.pinEcho1 = pinEcho1
        self.pinTrigger2 = pinTrigger2
        self.pinEcho2 = pinEcho2

        GPIO.setup(self.pinTrigger1, GPIO.OUT)
        GPIO.setup(self.pinEcho1, GPIO.IN)
        GPIO.setup(self.pinTrigger2, GPIO.OUT)
        GPIO.setup(self.pinEcho2, GPIO.IN)

    def get_values(self):
        left_value = GPIO.input(self.pinTrigger1)
        middle_value = GPIO.input(self.pinEcho1)
        right_value = GPIO.input(self.pinTrigger2)

        return (left_value, middle_value, right_value)

# Define a class that will publish a Twist depending on values from following sensors
class Braitenberg(rclpy.node.Node):

    def __init__(self):
        super().__init__('braitenberg')
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        self.publisher

        self.sensors = Braitenberg(pinTrigger1, pinEcho1, pinTrigger2, pinEcho2)

        self._linear_velocity = 0
        self._angular_velocity = 0

    def run(self):
        


        while rclpy.ok():
            values = self.sensors.get_values()
            match values:
                case (1, 1, 1):
                    print('Je suis dans le noir !')
                    self._linear_velocity = 0.0
                    self._angular_velocity = 0.0
                case (1, 1, 0):
                    print('Je derive a fond vers la gauche, a droite toute !')
                    self._linear_velocity = 1.0
                    self._angular_velocity = 1.5
                case (1, 0, 0):
                    print('Je derive un peu vers la gauche, a droite !')
                    self._linear_velocity = 1.0
                    self._angular_velocity = 1.0
                case (0, 0, 0):
                    print('On va tout droit !')
                    self._linear_velocity = 1.5
                    self._angular_velocity = 0.0
                case (0, 0, 1):
                    print('Je derive un peu vers la droite, a gauche !')
                    self._linear_velocity = 1.0
                    self._angular_velocity = -1.0
                case (0, 1, 1):
                    print('Je derive a fond vers la droite, a gauche toute ! ')
                    self._linear_velocity = 1.0
                    self._angular_velocity = -1.5
                case _:
                    print('Cas bizarre, je ne sais pas quoi faire !')
                    self._linear_velocity = 1.0
                    self._angular_velocity = 0.0
            
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

    braitenberg = Braitenberg()

    thread = threading.Thread(target=rclpy.spin, args=(braitenberg, ), daemon=True)
    thread.start()

    # Run node. This will block
    braitenberg.run()

    thread.join()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
   main()
