
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
    distance = (timeElapsed * 34300) / 2

    return distance


if __name__ == '__main__':

    try:
        aSpeed = 70
        bSpeed = 70
        target_diff = 0
        KP = 0.2
        KI = 0.002
        KD = 0
        prev_errorL = 0
        prev_errorR = 0
        sum_errorL = 0
        sum_errorR = 0
        track_width = 30
        sensor_offset = 7

        target = (track_width/2)-7

        rightRangeList = [distance('right')]
        leftRangeList = [distance('left')]

        print("KP = ", KP, "KI = ", KI,"KD = ", KD)

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

            if distR >50 or distL > 50:
                wall = False

            errorR = target - distL
            errorL = target - distR

            aSpeed += (errorL * KP) + (sum_errorL * KI) + (prev_errorL * KD)
            bSpeed += (errorR * KP) + (sum_errorR * KI) + (prev_errorR * KD)

            if aSpeed > 100 :
                aSpeed = 100
            if aSpeed < 60:
                aSpeed = 60
            if bSpeed > 100 :
                bSpeed = 100
            if bSpeed < 60:
                bSpeed = 60

            print("%.1f;%.1f;%.1f;%.1f;%.1f;%.1f" %(distL,distR,errorL,errorR,aSpeed,bSpeed))

            a_in1.start(False)
            a_in2.start(float(aSpeed))
            b_in1.start(float(bSpeed))
            b_in2.start(False)

            prev_errorL = errorL
            prev_errorR = errorR

            sum_errorL += errorL
            sum_errorR += errorR

            time.sleep(0.01)

        all_motors_off()


    except KeyboardInterrupt:

        print("Measurement stopped by User")
        GPIO.cleanup()
