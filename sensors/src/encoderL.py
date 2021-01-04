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
		GPIO.setwarnings(False)
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
	def run(self):
		buf = 0

		# loop until node is shutting down
		while not rospy.is_shutdown():

			# verify if encoders change state
			if GPIO.input(self.encoderPin) != buf:
				self.counter += 1
				buf = GPIO.input(buf.encoderPin)
				self.pub.publish(self.counter)
				rospy.loginfo(self.counter)

	run = staticmethod(run)


# ------------------- Main ---------------------


if __name__ == '__main__':
	try:
		print("Encoder Node")

		# initialization Node
		rospy.init_node('encodersNode', anonymous=True)


		# creating Encoder Object for left motor
		encoderHSA1 = Encoder('HSA1')
		encoderHSA1.run()

	except rospy.ROSInterruptException:
		pass
	finally:
		GPIO.cleanup()
