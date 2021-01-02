#! /usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import actionlib
import actions.msg
import time
from std_msgs.msg import Float64


class StraightDrive(object):
    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = actions.msg.straightDriveActionFeedback()
        self._result = actions.msg.straightDriveActionResult()

        # create action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.straightDriveAction,
        execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # RPi Pin setup
        self.AIN1_PIN = 29
        self.AIN2_PIN = 31
        self.BIN1_PIN = 21
        self.BIN2_PIN = 19
        self.STBY_PIN = 23

        # Initialize the motor and sensor variables
        self.distLeft = 0
        self.distRight = 0
        self.aSpeed = 70
        self.bSpeed = 70

        # Setup subscribers for the ultrasonic sensors
        self.subUltraLeft = rospy.Subscriber('/ultrasonic/Left', Float64, self.ultrasonic_left_callback)

        self.subUltraRight = rospy.Subscriber('/ultrasonic/Right', Float64, self.ultrasonic_right_callback)

    # Define a function to turn off all motors
    def all_motors_off(self):

        GPIO.output(self.AIN1_PIN, False)
        GPIO.output(self.AIN2_PIN, False)
        GPIO.output(self.BIN1_PIN, False)
        GPIO.output(self.BIN2_PIN, False)

    # When a new message appears from subscriber then the callback function is called
    def ultrasonic_left_callback(self, message):
        self.distLeft = message

    def ultrasonic_right_callback(self, message):
        self.distRight = message

    # Execute function is automatically executed in action server
    def execute_cb(self, goal):

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
        # action status
        success = True

        # variables: pid tuning factors
        kp = 0.2
        ki = 0.006
        kd = 0.002

        # variables: pid error memories
        prev_error_left = 0
        prev_error_right = 0
        sum_error_left = 0
        sum_error_right = 0

        # variables: target calculation
        track_width = 30
        sensor_offset = 7

        # turn all motors on 100 percent speed
        a_in1 = GPIO.PWM(self.AIN1_PIN, 100)
        a_in2 = GPIO.PWM(self.AIN2_PIN, 100)
        b_in1 = GPIO.PWM(self.BIN1_PIN, 100)
        b_in2 = GPIO.PWM(self.BIN2_PIN, 100)

        # Turn on standby pin of the motor driver chip
        GPIO.output(self.STBY_PIN, True)

        # publish info to the console for the user
        # rospy.loginfo('%s: Executing straightDrive, goal: %i, status: %i'% (self._action_name, goal.drive_until_passage,
        #                                                                    self._feedback))
        # calculated pid controller target
        target = (track_width / 2) - sensor_offset

        rospy.loginfo(type(self.distLeft))
        rospy.loginfo(type(self.distLeft.data))

        # start executing the action
        while self.distLeft.data < 10.0  or self.distRight.data < 10.0:
            # Function is active when a new request is made from action client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self.all_motors_off()
                break

            # feedback: wall is present
            self._feedback = True

            # calculating effective distance and expected distance
            error_left = target - self.distLeft.data
            error_right = target - self.distRight.data

            # pid controller for speed regulation
            self.aSpeed += (error_left * kp) + (sum_error_left * ki) + \
                           (prev_error_left * kd)

            self.bSpeed += (error_right * kp) + (sum_error_right * ki) + \
                           (prev_error_right * kd)

            # speed limitations
            if self.aSpeed > 100:
                self.aSpeed = 100
            if self.aSpeed < 60:
                self.aSpeed = 60
            if self.bSpeed > 100:
                self.bSpeed = 100
            if self.bSpeed < 60:
                self.bSpeed = 60

            # Turn on motors
            a_in1.start(float(self.aSpeed))
            a_in2.start(False)
            b_in1.start(float(self.bSpeed))
            b_in2.start(False)

            # saving of errors
            prev_error_left = error_left
            prev_error_right = error_right

            sum_error_left += error_left
            sum_error_right += error_right

            # Publish that there ist a wall
            self._as.publish_feedback(self._feedback)

        # When while condition is true, success function turn off all motors an publish success
        if success:
            self.all_motors_off()

            del a_in1
            del a_in2
            del b_in1
            del b_in2
            GPIO.cleanup()

            # feedback: wall_present
            self._feedback = False
            # result: wall_not_present
            self._result = True

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':

        try:
                rospy.init_node('straightDrive')
                server = StraightDrive(rospy.get_name())
                rospy.spin()

        except  rospy.ROSInterruptException:
                pass

        finally:
                GPIO.cleanup()
