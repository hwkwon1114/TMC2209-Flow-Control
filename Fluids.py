import time
import RPi.GPIO as GPIO

DESIRED_SPEED = 50  # Desired speed iFluids.pyn steps per second
DIR_PIN = 22  # GPIO pin for direction signal
STEP_PIN = 27  # GPIO pin for step signal
MIN_PULSE_DURATION = 1.9e-6  # Minimum pulse duration in seconds (1.9us)


class Stepper_Driver(object):
    def __init__(self, pinDir, pinStep):
        self.pinDir = pinDir
        self.pinStep = pinStep
        self.pulseDuration = MIN_PULSE_DURATION
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinStep, GPIO.OUT)
        GPIO.setup(self.pinDir, GPIO.OUT)

    def setSpeed(self, desired_speed):
        # Calculate pulse duration based on desired speed
        desired_pulse_duration = 1.0 / desired_speed

        # Ensure pulse duration is not less than the minimum
        if desired_pulse_duration < MIN_PULSE_DURATION:
            print("Warning: Desired speed is too high, setting to maximum speed.")
            self.pulseDuration = MIN_PULSE_DURATION
        else:
            self.pulseDuration = desired_pulse_duration

    def step(self, direction):
        # Set direction
        GPIO.output(self.pinDir, direction)

        # Pulse the step pin
        GPIO.output(self.pinStep, GPIO.HIGH)
        time.sleep(self.pulseDuration / 2)
        GPIO.output(self.pinStep, GPIO.LOW)
        time.sleep(self.pulseDuration / 2)


temp = Stepper_Driver(DIR_PIN, STEP_PIN)

try:
    while True:
        temp.step(GPIO.LOW)  # Replace with GPIO.LOW for the other direction
except KeyboardInterrupt:
    GPIO.cleanup()
