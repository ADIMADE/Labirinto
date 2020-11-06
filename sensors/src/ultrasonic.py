#!/usr/bin/env python3
import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float64


# ------------------- Class ---------------------


# Class Ultrasonic : for every ultrasonic input
class Ultrasonic():

    # constructor of the class Ultrasonic
    def __init__(self, ultrasonicName):

        #Get the GPIO pin of the trigger and echo pin in the ROS parameter server
        #self.triggerPin = rospy.get_param(ultrasonicName)
        #self.echoPin = rospy.get_param(ultrasonicName)

        self.triggerPin =
        self.echoPin =

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.triggerPin)
        GPIO.setup(self.echoPin)

        #Publisher
        self.stringPubName = 'ultrasonic' +ultrasonicName
        self.pub = rospy.Publisher(self.stringPubName, Float64, queue_size=10)

    # Method for continuous distance measurement
    def run(ultrasonicArray):

        # run distance measurement until node is shut down
        while not rospy.is_shutdown():
            for i in range(len(ultrasonicArray)):
                # initialize input to triggerPin
                GPIO.output(ultrasonicArray.triggerPin, False)
                time.sleep(0.1)
                GPIO.output(ultrasonicArray.triggerPin, True)
                time.sleep(0.00001)
                GPIO.output(ultrasonicArray.triggerPin, False)

                # start timer while ultrasonic waves are sent
                while GPIO.input(ultrasonicArray.echoPin) == 0:
                    pulse_start = time.time()
                # stop timer when ultrasonic waves are received
                while GPIO.input(ultrasonicArray.echoPin) == 1:
                    pulse_end = time.time()

                # calculation of distance with half speed of sound (17'320cm/s) at 25 degree
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17320

    run = staticmethod(run)

# ------------------- Main ---------------------

if __name__ == '__main__':
    try:
        print("Ultrasonic Node")

        # initialization of the node
        rospy.init_node('encodersNode', anonymous=True)

        # creation of a list for all ultrasonic sensors
        ultrasonicArray = []

        #creating ultrasonic object front
        ultrasonicFront = Ultrasonic('ultrasonicFront')
        ultrasonicArray.append(ultrasonicFront)

        # creating ultrasonic object left
        ultrasonicLeft = Ultrasonic('ultrasonicLeft')
        ultrasonicArray.append(ultrasonicLeft)

        # creating ultrasonic object right
        ultrasonicRight = Ultrasonic('ultrasonicRight')
        ultrasonicArray.append(ultrasonicRight)

    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()



