#! /usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import actionlib
import actions.msg
import math
import time
from std_msgs.msg import Float64
from std_msgs.msg import Int64

#Motor A is Right
#Motor B is Left

class StraightDriveDist(object):
    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = actions.msg.straightDriveDistFeedback()
        self._result = actions.msg.straightDriveDistResult()

        # create action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.straightDriveDistAction,
        execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # RPi Pin setup
        self.AIN1_PIN = 29
        self.AIN2_PIN = 31
        self.BIN1_PIN = 21
        self.BIN2_PIN = 19
        self.STBY_PIN = 23

        # Initialize the variable for the sensors and motors
        self.distFront = 0
        self.encoderLeft = 0
        self.encoderRight = 0
        self.aSpeed = 70
        self.bSpeed = 70

        # Setup subscriber for the ultrasonic sensor
        self.subUltraFront = rospy.Subscriber('/ultrasonic/Front', Float64, self.ultrasonic_front_callback)

        # Setup subscribers for encoder sensors
        self.subEncoderLeft = rospy.Subscriber('/encoder/HSA1', Int64, self.encoder_left_callback)

        self.subEncoderRight = rospy.Subscriber('/encoder/HSB1', Int64, self.encoder_right_callback)

    # Define a function to turn off all motors
    def all_motors_off(self):

        GPIO.output(self.AIN1_PIN, False)
        GPIO.output(self.AIN2_PIN, False)
        GPIO.output(self.BIN1_PIN, False)
        GPIO.output(self.BIN2_PIN, False)

    # When a new message appears from subscriber then the callback function is called
    def ultrasonic_front_callback(self, message):
        self.ultrasonicFront = message.data

    def encoder_left_callback(self, message):
        self.encoderLeft = message.data

    def encoder_right_callback(self, message):
        self.encoderRight = message.data

    # Execute function is automatically executed in action server
    def execute_cb(self, goal):

        # Setup GPIO's as output
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(self.AIN1_PIN, GPIO.OUT)
        GPIO.setup(self.AIN2_PIN, GPIO.OUT)
        GPIO.setup(self.BIN1_PIN, GPIO.OUT)
        GPIO.setup(self.BIN2_PIN, GPIO.OUT)
        GPIO.setup(self.STBY_PIN, GPIO.OUT)

        # turn all motors on 100 percent speed
        a_in1 = GPIO.PWM(self.AIN1_PIN, 100)
        a_in2 = GPIO.PWM(self.AIN2_PIN, 100)
        b_in1 = GPIO.PWM(self.BIN1_PIN, 100)
        b_in2 = GPIO.PWM(self.BIN2_PIN, 100)

        # Set standby pin of motor controller high
        GPIO.output(self.STBY_PIN, True)

        # action status
        success = True

        # variables: pid tuning factors
        kp = 0.2
        ki = 0
        kd = 0

        # variables: pid error memories
        prev_error = 0
        sum_error = 0

        # variables for distance calculation

        start_time = time.time()
        seconds = 0.5

        while True:
             current_time = time.time()
             elapsed_time = current_time - start_time

             a_in1.start(float(100))
             a_in2.start(False)
             b_in1.start(float(100))
             b_in2.start(False)

             if elapsed_time > seconds:
                  break

        wheel_diameter = 6.8
        wheel_scope = wheel_diameter * math.pi
        ticks_per_turn = 275

        ticks_per_centimeter = ticks_per_turn / wheel_scope

        ticks_goal = goal.distance * ticks_per_centimeter

        ticks_initial = self.encoderRight

        ticks_current = 0

        # publish info to the console for the user
        # rospy.loginfo('%s: Executing straightDrive, goal: %i, status: %i'% (self._action_name, goal.distance,
        #                                                                    self._feedback))
        # start executing the action
        rospy.loginfo(ticks_initial)
        rospy.loginfo(ticks_goal)
        while ticks_current < ticks_goal :
            # Function is active when a new request is made from action client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.all_motors_off()
                break

            # calculating effective distance and expected distance
            error = self.encoderRight - self.encoderLeft

            # pid controller for speed regulation
            self.aSpeed += (error * kp) + (sum_error * ki) + (prev_error * kd)

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

            ticks_current = self.encoderRight - ticks_initial

            rospy.loginfo(ticks_current)

            self._feedback.distanceCurrent = ticks_current / ticks_per_centimeter

            # Publish that there ist a wall
            self._as.publish_feedback(self._feedback)

        # When while condition is true, success function turn off all motors an publish success
        if success:

            a_in1.stop()
            a_in2.stop()
            b_in1.stop()
            b_in2.stop()

            del a_in1
            del a_in2
            del b_in1
            del b_in2

            self.all_motors_off()

            self._result.distanceCompleted = self._feedback.distanceCurrent

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':

        try:
                 rospy.init_node('straightDriveDist')
                 server = StraightDriveDist(rospy.get_name())
                 rospy.spin()

        except rospy.ROSInterruptExcpetion:
                 pass

        finally:
                 GPIO.cleanup()
