import time
import RPi.GPIO as GPIO
import math

DESIRED_DISPLACEMENT = 50  # ml
DIR_PIN = 17  # GPIO pin for direction signal
STEP_PIN = 27  # GPIO pin for step signal


class Stepper_Driver(object):
    def __init__(self):
        self.pinStep = STEP_PIN
        self.pinDir = DIR_PIN
        self.pulseSpeed = 100  # Steps per second
        self.pulseDuration = 1 / 100
        self.maxSpeed = 500
        self.minSpeed = 0
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinStep, GPIO.OUT)
        GPIO.setup(self.pinDir, GPIO.OUT)

    def setSpeed(self, desired_speed):
        if desired_speed > self.maxSpeed:
            desired_speed = self.maxSpeed
        elif desired_speed < self.maxSpeed:
            desired_speed = self.minSpeed
        self.pulseSpeed = desired_speed
        self.pulseDuration = 1 / self.pulseSpeed


temp = Stepper_Driver()
try:
    while True:
        GPIO.output(temp.pinStep, GPIO.HIGH)
        time.sleep(temp.pulseDuration / 2)
        GPIO.output(temp.pinStep, GPIO.LOW)
        time.sleep(temp.pulseDuration / 2)
except KeyboardInterrupt:
    GPIO.cleanup()
