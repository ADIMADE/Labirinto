#! /usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import actionlib
import actions.msg
import time
from std_msgs.msg import Float64


class Turn(object):
	def __init__(self, name):
		# create messages that are used to publish feedback/result
		self._feedback = actions.msg.turnFeedback()
		self._result = actions.msg.turnResult()

		# create action server
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.turnAction,
		execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

		# RPi Pin setup
		self.AIN1_PIN = 29
		self.AIN2_PIN = 31
		self.BIN1_PIN = 21
		self.BIN2_PIN = 19
		self.STBY_PIN = 23

		# Initialize the variable for the angle speed of the mpu sensor
		self.degrPerSec = 0

		# Setup subscriber for the mpu sensor
		self.sub = rospy.Subscriber('mpu_gyroZ', Float64, self.mpu_callback)

	# Define a function to turn off all motors
	def all_motors_off(self):
		GPIO.output(self.AIN1_PIN, False)
		GPIO.output(self.AIN2_PIN, False)
		GPIO.output(self.BIN1_PIN, False)
		GPIO.output(self.BIN2_PIN, False)

	# When a new message appears from subscriber then the callback function is called
	def mpu_callback(self, message):
		self.degrPerSec = message.data

	# Execute function is automatically executed in action server
	def execute_cb(self, goal):

		GPIO.setmode(GPIO.BOARD)
		GPIO.setwarnings(False)
		# Setup GPIO's as output
		GPIO.setup(self.AIN1_PIN, GPIO.OUT)
		GPIO.setup(self.AIN2_PIN, GPIO.OUT)
		GPIO.setup(self.BIN1_PIN, GPIO.OUT)
		GPIO.setup(self.BIN2_PIN, GPIO.OUT)
		GPIO.setup(self.STBY_PIN, GPIO.OUT)

		# Set standby pin of motor controller high
		GPIO.output(self.STBY_PIN, True)


		self._feedback.angle = 0
		rate = 0.01
		speed = 100
		success = True

		a_in1 = GPIO.PWM(self.AIN1_PIN, 100)
		a_in2 = GPIO.PWM(self.AIN2_PIN, 100)
		b_in1 = GPIO.PWM(self.BIN1_PIN, 100)
		b_in2 = GPIO.PWM(self.BIN2_PIN, 100)

		# Turn on standby pin of the motor driver chip
		GPIO.output(self.STBY_PIN, True)

		# publish info to the console for the user
		#rospy.loginfo('%s: Executing Turn, goal: %i, status: %i'% (self._action_name, goal.turn_angle, self._feedback))

		# start executing the action
		if goal.turn_angle < 0:
			while self._feedback.angle >= goal.turn_angle:
				# Function is active when a new request is made from action client
				if self._as.is_preempt_requested():
					rospy.loginfo('%s: Preempted' % self._action_name)
					self._as.set_preempted()
					success = False
					self.all_motors_off()
					break

				# Calculate the total driven angle
				degree = self.degrPerSec * rate
				self._feedback.angle += degree

				rospy.loginfo(self._feedback.angle)
				rospy.loginfo(degree)

				# Turn on motors
				a_in1.start(False)
				a_in2.start(float(speed))
				b_in1.start(False)
				b_in2.start(float(speed))

				# Publish driven angle
				self._as.publish_feedback(self._feedback)

				# Sleep in the same rate as the sampling rate of the mpu
				time.sleep(rate)
		else : 
			while self._feedback.angle <= goal.turn_angle:
				# Function is active when a new request is made from action client
				if self._as.is_preempt_requested():
					rospy.loginfo('%s: Preempted' % self._action_name)
					self._as.set_preempted()
					success = False
					self.all_motors_off()
					break

				# Calculate the total driven angle
				degree = self.degrPerSec * rate
				self._feedback.angle += degree

				rospy.loginfo(self._feedback.angle)
				rospy.loginfo(degree)

				# Turn on motors
				a_in1.start(float(speed))
				a_in2.start(False)
				b_in1.start(float(speed))
				b_in2.start(False)

				# Publish driven angle
				self._as.publish_feedback(self._feedback)

				# Sleep in the same rate as the sampling rate of the mpu
				time.sleep(rate)

		# When while condition is true, success function turn off all motors an publish success
		if success:
			self.all_motors_off()

			a_in1.stop()
			a_in2.stop()
			b_in1.stop()
			b_in2.stop()

			del a_in1
			del a_in2
			del b_in1
			del b_in2

			self._result.turn_complete = self._feedback.angle
			rospy.loginfo('%s: Succeeded' % self._action_name)
			self._as.set_succeeded(self._result)



if __name__ == '__main__':
        try:
	          rospy.init_node('turn')
	          server = Turn(rospy.get_name())
	          rospy.spin()

        except rospy.ROSInterruptException:
               pass

        finally:
                  GPIO.cleanup()
