#!/usr/bin/env python3
import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Int64
from std_msgs.msg import Bool


# ------------------- Class ---------------------

# Class Encoder : for every encoder input
class Encoder():

    # constructor of Class Encoder
	def __init__(self, encoderName):

		# Attribute : counter for increments
		self.counter = 0

		# Get the GPIO Pin of Encoder in the ROS Parameter Serve
		self.encoderPin = int(rospy.get_param(encoderName))

		rospy.loginfo(self.encoderPin)

		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.encoderPin, GPIO.IN)

		# Publisher and Suscriber
		self.stringPubName = '/encoder/' + encoderName
		self.pub = rospy.Publisher(self.stringPubName, Int64, queue_size=10)
		self.resetSuscriber = rospy.Subscriber('encoderReset', Bool, self.reset)

	# Method : reset of the counter
	def reset(self, message):
		if message == 1:
			self.counter = 0

	# Method : loop for counter
	def run(encoderArray):
		bufferArray = [0] * len(encoderArray)

		# loop until node is shutting down
		while not rospy.is_shutdown():
			for i in range(len(encoderArray)):
				# verify if encoders change state
				if GPIO.input(encoderArray[i].encoderPin) != bufferArray[i]:
					encoderArray[i].counter += 1
					bufferArray[i] = GPIO.input(encoderArray[i].encoderPin)
					encoderArray[i].pub.publish(encoderArray[i].counter)
					rospy.loginfo(encoderArray[i].counter)

	run = staticmethod(run)


# ------------------- Main ---------------------


if __name__ == '__main__':
	try:
		print("Encoder Node")

		# initialization Node
		rospy.init_node('encodersNode', anonymous=True)

		# creatin of List for all encoders
		encoderArray = []

		# creating Encoder Object for left motor
		encoderHSA1 = Encoder('HSA1')
		encoderArray.append(encoderHSA1)

		# creating Encoder Object for right motor
		encoderHSB1 = Encoder('HSB1')
		encoderArray.append(encoderHSB1)

		Encoder.run(encoderArray)

	except rospy.ROSInterruptException:
		pass
	finally:
		GPIO.cleanup()
