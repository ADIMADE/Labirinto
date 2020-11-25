#! /usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import actionlib
import action.msg
import time
from std_msgs.msg import Float64


class Turn(object):
    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = action.msg.TurnActionFeedback()
        self._result = action.msg.TurnActionResult()

        # create action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action.msg.TurnAction,
        execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # RPi Pin setup
        self.AIN1_PIN = 11
        self.AIN2_PIN = 13
        self.BIN1_PIN = 19
        self.BIN2_PIN = 21
        self.STBY_PIN = 7

        # Setup GPIO's as output
        GPIO.setup(self.AIN1_PIN, GPIO.OUT)
        GPIO.setup(self.AIN2_PIN, GPIO.OUT)
        GPIO.setup(self.BIN1_PIN, GPIO.OUT)
        GPIO.setup(self.BIN2_PIN, GPIO.OUT)
        GPIO.setup(self.STBY_PIN, GPIO.OUT)

        # Set standby pin of motor controller high
        GPIO.output(self.STBY_PIN, True)

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
        self.degrPerSec = message

    # Execute function is automatically executed in action server
    def execute_cb(self, goal):
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
        rospy.loginfo('%s: Executing Turn', self._action_name, goal.order, self._feedback)

        # start executing the action
        while self._feedback < goal.order:
            # Function is active when a new request is made from action client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.all_motors_off()
                break

            # Calculate the total driven angle
            degree = self.degrPerSec * rate
            self._feedback += degree

            # Turn on motors
            a_in1.start(float(speed))
            a_in2.start(False)
            b_in1.start(False)
            b_in2.start(float(speed))

            # Publish driven angle
            self._as.publish_feedback(self._feedback)

            # Sleep in the same rate as the sampling rate of the mpu
            time.sleep(rate)

        # When while condition is true, success function turn off all motors an publish success
        if success:
            self.all_motors_off()
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_suceeded(self._result)


if __name__ == '__main__':
    rospy.init_node('turn')
    server = Turn(rospy.get_name())
    rospy.spin()