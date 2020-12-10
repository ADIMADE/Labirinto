#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time


# GPIO Mode (BOARD / BCM)

GPIO.setmode(GPIO.BOARD)

#Ultrasonic
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.IN)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(24, GPIO.IN)

#Motors
AIN1_PIN = 11
AIN2_PIN = 13
BIN1_PIN = 19
BIN2_PIN = 21
STBY_PIN = 7
GPIO.setup(AIN1_PIN, GPIO.OUT)
GPIO.setup(AIN2_PIN, GPIO.OUT)
GPIO.setup(BIN1_PIN, GPIO.OUT)
GPIO.setup(BIN2_PIN, GPIO.OUT)
GPIO.setup(STBY_PIN, GPIO.OUT)
GPIO.output(STBY_PIN, True)
a_in1 = GPIO.PWM(AIN1_PIN, 100)
a_in2 = GPIO.PWM(AIN2_PIN, 100)
b_in1 = GPIO.PWM(BIN1_PIN, 100)
b_in2 = GPIO.PWM(BIN2_PIN, 100)


def all_motors_off():
    GPIO.output(AIN1_PIN, False)
    GPIO.output(AIN2_PIN, False)
    GPIO.output(BIN1_PIN, False)
    GPIO.output(BIN2_PIN, False)


def distance(side):
    if side == 'right':
        GPIO_TRIGGER = 16
        GPIO_ECHO = 18
    elif side == 'left':
        GPIO_TRIGGER = 22
        GPIO_ECHO = 24

    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    startTime = time.time()
    stopTime = time.time()
    # save StartTime

    while GPIO.input(GPIO_ECHO) == 0:
        startTime = time.time()

    # save time of arrival

    while GPIO.input(GPIO_ECHO) == 1:
        stopTime = time.time()

    # time difference between start and arrival

    timeElapsed = stopTime - startTime
    distance = (timeElapsed * 3430) / 2

    return distance


if __name__ == '__main__':

    try:
        speed = 100

        rightRangeList = [distance('right')]
        leftRangeList = [distance('left')]

        wall = True

        while wall:
            rightRangeList.append(distance('right'))
            if len(rightRangeList) > 5:
                del rightRangeList[0]
            leftRangeList.append(distance('left'))
            if len(leftRangeList) > 5:
                del leftRangeList[0]

            distR = sum(rightRangeList) / len(rightRangeList)
            distL = sum(leftRangeList) / len(leftRangeList)

            if distR > 30 or distL > 30:
                wall = False

            diff = distR - distL

            print("Measured Distance Right = %.1f mm" % distR)
            print("Measured Distance Left = %.1f mm" % distL)

            a_in1.start(float(speed-diff))
            a_in2.start(False)
            b_in1.start(False)
            b_in2.start(float(speed+diff))

            time.sleep(0.01)

        all_motors_off()


    except KeyboardInterrupt:

        print("Measurement stopped by User")
        GPIO.cleanup()
