#! /usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import actionlib
import actions.msg
from std_msgs.msg import Float64


class StraightDriveDist(object):
    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = actions.msg.straightDriveDistActionFeedback()
        self._result = actions.msg.straightDriveDistActionResult()

        # create action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.straightDriveDistAction,
        execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # RPi Pin setup
        self.AIN1_PIN = 11
        self.AIN2_PIN = 13
        self.BIN1_PIN = 19
        self.BIN2_PIN = 21
        self.STBY_PIN = 7

        # Select pin mode
        GPIO.setmode(GPIO.BOARD)

        # Setup GPIO's as output
        GPIO.setup(self.AIN1_PIN, GPIO.OUT)
        GPIO.setup(self.AIN2_PIN, GPIO.OUT)
        GPIO.setup(self.BIN1_PIN, GPIO.OUT)
        GPIO.setup(self.BIN2_PIN, GPIO.OUT)
        GPIO.setup(self.STBY_PIN, GPIO.OUT)

        # Set standby pin of motor controller high
        GPIO.output(self.STBY_PIN, True)

        # Initialize the variable for the sensors and motors
        self.distFront = 0
        self.encoderLeft = 0
        self.encoderRight = 0
        self.aSpeed = 70
        self.bSpeed = 70
        self._feedback = True

        # Setup subscriber for the ultrasonic sensor
        self.subUltraFront = rospy.Subscriber('/ultrasonic/Front', Float64, self.ultrasonic_front_callback)

        # Setup subscribers for encoder sensors
        self.subEncoderLeft = rospy.Subscriber('/encoder/HSA1', Float64, self.encoder_left_callback)

        self.subEncoderRight = rospy.Subscriber('/encoder/HSB1', Float64, self.encoder_right_callback)

    # Define a function to turn off all motors
    def all_motors_off(self):

        GPIO.output(self.AIN1_PIN, False)
        GPIO.output(self.AIN2_PIN, False)
        GPIO.output(self.BIN1_PIN, False)
        GPIO.output(self.BIN2_PIN, False)

    # When a new message appears from subscriber then the callback function is called
    def ultrasonic_front_callback(self, message):
        self._feedback = message

    def encoder_left_callback(self, message):
        self.encoderLeft = message

    def encoder_right_callback(self, message):
        self.encoderRight = message

    # Execute function is automatically executed in action server
    def execute_cb(self, goal):
        # action status
        success = True

        # variables: pid tuning factors
        kp = 0.2
        ki = 0.006
        kd = 0.002

        # variables: pid error memories
        prev_error = 0
        sum_error = 0

        # turn all motors on 100 percent speed
        a_in1 = GPIO.PWM(self.AIN1_PIN, 100)
        a_in2 = GPIO.PWM(self.AIN2_PIN, 100)
        b_in1 = GPIO.PWM(self.BIN1_PIN, 100)
        b_in2 = GPIO.PWM(self.BIN2_PIN, 100)

        # Turn on standby pin of the motor driver chip
        GPIO.output(self.STBY_PIN, True)

        # publish info to the console for the user
        rospy.loginfo('%s: Executing straightDrive, goal: %i, status: %i'% (self._action_name, goal.distance,
                                                                            self._feedback))
        # start executing the action
        while self._feedback > goal.distance:
            # Function is active when a new request is made from action client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.all_motors_off()
                break

            # calculating effective distance and expected distance
            error = self.encoderLeft - self.encoderRight

            # pid controller for speed regulation
            self.aSpeed += (error * kp) + (sum_error * ki) + \
                           (prev_error * kd)

            # speed limitations
            if self.aSpeed > 100:
                self.aSpeed = 100
            if self.aSpeed < 60:
                self.aSpeed = 60

            # Turn on motors
            a_in1.start(float(self.aSpeed))
            a_in2.start(False)
            b_in1.start(float(self.bSpeed))
            b_in2.start(False)

            # saving of errors
            prev_error = error
            sum_error += error

            # Publish that there ist a wall
            self._as.publish_feedback(self._feedback)

        # When while condition is true, success function turn off all motors an publish success
        if success:
            self.all_motors_off()
            self._result = self._feedback

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('straightDriveDist')
    server = StraightDriveDist(rospy.get_name())
    rospy.spin()