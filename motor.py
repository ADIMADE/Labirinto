import time
import RPi.GPIO as GPIO
import threading
GPIO.setmode(GPIO.BOARD)

#Set pins for Motors
AIN1_PIN = 7
AIN2_PIN = 11
BIN1_PIN = 13
BIN2_PIN = 15

#Set pins for Motor Standby
STDBY_PIN = 19

#Set pins for Hall Sensors
HSA1 = 23
HSA2 = 24
HSB1 = 23
HSB2 = 24

#Defining Motor Outputs
GPIO.setup(AIN1_PIN, GPIO.OUT)
GPIO.setup(AIN2_PIN, GPIO.OUT)
GPIO.setup(BIN1_PIN, GPIO.OUT)
GPIO.setup(BIN2_PIN, GPIO.OUT)

#Defining Motor Standby Output
GPIO.setup(STDBY_PIN, GPIO.OUT)

#Defining Hall Sensors Inputs
GPIO.setup(HSA1, GPIO.IN)
GPIO.setup(HSA2, GPIO.IN)
GPIO.setup(HSB1, GPIO.IN)
GPIO.setup(HSB2, GPIO.IN)

#Turn On Motors Outputs
GPIO.output(STDBY_PIN, True)

#Creating PWM Objects
AIN1 = GPIO.PWM(AIN1_PIN, 100)
AIN2 = GPIO.PWM(AIN2_PIN, 100)
BIN1 = GPIO.PWM(BIN1_PIN, 100)
BIN2 = GPIO.PWM(BIN2_PIN, 100)

#Function to stop all Motoros
def all_off():
    GPIO.output(AIN1_PIN, False)
    GPIO.output(AIN2_PIN, False)
    GPIO.output(BIN1_PIN, False)
    GPIO.output(BIN2_PIN, False)




#Function to rotate MotorA
def moveMotorA(speed, increments):
    #Counter for encoders
    counterA = 0

    #If given speed positiv turn forward
    if speed > 0:
        while counterA <= increments:
            if GPIO.input(HSA1) != bufferHSA1:
                counterA += GPIO.input(HSA1)
            AIN1.start(float(speed))
            AIN2.start(float(0))
    #If given speed negativ turn backward
    else:
        while counterA <= increments:
            if GPIO.input(HSA1) != bufferHSA1:
                counterA += GPIO.input(HSA1)
            AIN2.start(float(speed)*(-1))
            AIN1.start(float(0))


#Function to rotate MotorB
def moveMotorB(speed, increments):
    # Counter for encoders
    counterB = 0
    # If given speed positiv turn forward
    if speed > 0:
        while counterB <= increments:
            if GPIO.input(HSB1) != bufferHSA1:
                counterB += GPIO.input(HSA1)
            BIN1.start(float(speed))
            BIN2.start(float(0))
    # If given speed negativ turn backward
    else:
        while counterB <= increments:
            if GPIO.input(HSB1) != bufferHSA1:
                counterB += GPIO.input(HSA1)
            BIN2.start(float(speed)*(-1))
            BIN1.start(float(0))




def moveForward(speed, distance):
    #Calculation of encoders increments needed for reaching the distance
    increments = distance

    #defining the threads to move each motor
    threadA = threading.Thread(name="threadMotorA", target=moveMotorA(speed, increments))
    threadB = threading.Thread(name="threadMotorB", target=moveMotorB(speed, increments))

    #start the threads
    threadA.start()
    threadB.start()
    #Wait on each thread : wait that each motor reach is increments
    threadA.join()
    threadB.join()


def moveBackward(speed, distance):
    # Calculation of encoders increments needed for reaching the distance
    increments = distance

    # defining the threads to move each motor
    threadA = threading.Thread(name="threadMotorA", target=moveMotorA(speed*(-1), increments))
    threadB = threading.Thread(name="threadMotorB", target=moveMotorB(speed*(-1), increments))

    # start the threads
    threadA.start()
    threadB.start()
    # Wait on each thread : wait that each motor reach is increments
    threadA.join()
    threadB.join()


def moveRight(speed, degree):
    # Calculation of encoders increments needed for reaching the distance
    incrementsA = degree
    incrementsB = degree*(-1)

    # defining the threads to move each motor
    threadA = threading.Thread(name="threadMotorA", target=moveMotorA(speed, incrementsA))
    threadB = threading.Thread(name="threadMotorB", target=moveMotorB(speed, incrementsB))

    # start the threads
    threadA.start()
    threadB.start()
    # Wait on each thread : wait that each motor reach is increments
    threadA.join()
    threadB.join()

def moveLeft(speed, degree):
    # Calculation of encoders increments needed for reaching the distance
    incrementsA = degree*(-1)
    incrementsB = degree

    # defining the threads to move each motor
    threadA = threading.Thread(name="threadMotorA", target=moveMotorA(speed, incrementsA))
    threadB = threading.Thread(name="threadMotorB", target=moveMotorB(speed, incrementsB))

    # start the threads
    threadA.start()
    threadB.start()
    # Wait on each thread : wait that each motor reach is increments
    threadA.join()
    threadB.join()