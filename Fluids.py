import time
import RPi.GPIO as GPIO
import math
TUBE_AREA = math.pi * (0.4 * 2.56) ** 2  # Area of Pipe
DESIRED_DISPLACEMENT = 50  # ml
DIR_PIN = 17  # GPIO pin for direction signal
STEP_PIN = 27  # GPIO pin for step signal
PULSE_PER_STEP = 256
STEPS_PER_REV = 1000  # 1.8 degree
CURRENT_CHANGE = 0
P = 0.1
I = 0.5
D = 0.1
class Stepper_Driver(object):
    def __init__(self):
        self.pinStep = STEP_PIN
        self.pinDir = DIR_PIN
        self.pulseSpeed = 100  # Steps per second
        self.pulseDuration = 1 / self.pulseSpeed
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
    		GPIO.output(self.pinStep, GPIO.HIGH)
                time.sleep(self.pulseDuration / 2)
                GPIO.output(self.pinStep, GPIO.LOW)
                time.sleep(self.pulseDuration / 2)
        except KeyboardInterrupt:
            GPIO.cleanup()

