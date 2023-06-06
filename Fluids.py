import time
import RPi.GPIO as GPIO
import math

DESIRED_DISPLACEMENT = 50  # ml
DIR_PIN = 17  # GPIO pin for direction signal
STEP_PIN = 27  # GPIO pin for step signal


class Stepper_Driver(object):
    pinDir = 17
    pinStep = 27
    pulseDuration = 1 / 100
    maxSpeed = 500
    minSpeed = 0

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinStep, GPIO.OUT)
        GPIO.setup(self.pinDir, GPIO.OUT)

    def setSpeed(self, desired_speed):
        if desired_speed > self.maxSpeed:
            desired_speed = self.maxSpeed
        elif desired_speed < self.maxSpeed:
            desired_speed = self.minSpeed
        self.pulseDuration = 1 / desired_speed


temp = Stepper_Driver()
try:
    while True:
        GPIO.output(temp.pinStep, GPIO.HIGH)
        time.sleep(temp.pulseDuration / 2)
        GPIO.output(temp.pinStep, GPIO.LOW)
        time.sleep(temp.pulseDuration / 2)
except KeyboardInterrupt:
    GPIO.cleanup()
