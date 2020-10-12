import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

AIN1_PIN = 7
AIN2_PIN = 11
BIN1_PIN = 13
BIN2_PIN = 15
STDBY_PIN = 19

GPIO.setup(AIN1_PIN, GPIO.OUT)
GPIO.setup(AIN2_PIN, GPIO.OUT)
GPIO.setup(BIN1_PIN, GPIO.OUT)
GPIO.setup(BIN2_PIN, GPIO.OUT)
GPIO.setup(STDBY_PIN, GPIO.OUT)

GPIO.output(STDBY_PIN, True)

def all_off():
    GPIO.output(AIN1_PIN, False)
    GPIO.output(AIN2_PIN, False)
    GPIO.output(BIN1_PIN, False)
    GPIO.output(BIN2_PIN, False)

AIN1 = GPIO.PWM(AIN1_PIN, 100)
AIN2 = GPIO.PWM(AIN2_PIN, 100)
BIN1 = GPIO.PWM(BIN1_PIN, 100)
BIN2 = GPIO.PWM(BIN2_PIN, 100)

try:
    while True:

        BIN1.start(float(30))
        GPIO.output(BIN2_PIN, False)
        time.sleep(5)
        BIN1.stop()
        #AIN2.stop()
        time.sleep(2)

except KeyboardInterrupt:
    all_off()

finally:
    GPIO.cleanup()
