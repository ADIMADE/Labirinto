import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

AIN1_PIN = 7
AIN2_PIN = 11
BIN1_PIN = 13
BIN2_PIN = 15

GPIO.setup(AIN1_PIN, GPIO.OUT)
GPIO.setup(AIN2_PIN, GPIO.OUT)
GPIO.setup(BIN1_PIN, GPIO.OUT)
GPIO.setup(BIN2_PIN, GPIO.OUT)

def all_off():
	GPIO.output(AIN1_PIN, False)
	GPIO.output(AIN2_PIN, False)
	GPIO.output(BIN1_PIN, False)
	GPIO.output(BIN2_PIN, False)

AIN1 = GPIO.pwm(AIN1_PIN, 100)
AIN2 = GPIO.pwm(AIN2_PIN, 100)
BIN1 = GPIO.pwm(BIN1_PIN, 100)
BIN2 = GPIO.pwm(BIN2_PIN, 100)

try:
	while True:

		AIN1.start(float(50))
		AIN2.start(False)
		sleep(5)
		AIN1.stop()
		AIN2.stop()

except KeyboardInterrupt:
	all_off()

finally:
	GPIO.cleanup()