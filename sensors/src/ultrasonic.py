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
        GPio.setup(self.echoPin)

        #Publisher
        self.stringPubName = 'ultrasonic' +ultrasonicName
        self.pub = rospy.Publisher(self.stringPubName, Float64, queue_size=10)

    #Method to calculate distance
    def get_distance(self):


