import time
import RPi.GPIO as GPIO
from sensirion_i2c_driver import LinuxI2cTransceiver, I2cConnection, CrcCalculator
from sensirion_driver_adapters.i2c_adapter.i2c_channel import I2cChannel
from sensirion_i2c_sf06_lf.device import Sf06LfDevice
from sensirion_i2c_sf06_lf.commands import InvFlowScaleFactors
import math

DESIRED_SPEED = 500  # Desired speed in steps per second
DIR_PIN = 27  # GPIO pin for direction signal
STEP_PIN = 22  # GPIO pin for step signal
MIN_PULSE_DURATION = 1.9e-6  # Minimum pulse duration in seconds (1.9us)
TUBE_AREA = math.pi * (0.4 * 2.56) ** 2  # Area of Pipe
DESIRED_DISPLACEMENT = 0.5  # ml


class Stepper_Driver(object):
    def __init__(self, pinDir, pinStep):
        self.pinDir = pinDir
        self.pinStep = pinStep
        self.pulseDuration = MIN_PULSE_DURATION
        self.direction = GPIO.HIGH
        GPIO.setwarnings(False)
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

    def setDirection(self, Direction):
        self.direction = Direction

    def step(self):
        # Set direction
        GPIO.output(self.pinDir, self.direction)
        # Pulse the step pin
        GPIO.output(self.pinStep, GPIO.HIGH)
        time.sleep(self.pulseDuration / 2)
        GPIO.output(self.pinStep, GPIO.LOW)
        time.sleep(self.pulseDuration / 2)

    def run(self):
        try:
            while True:
                self.step(GPIO.HIGH)  # Replace with GPIO.LOW for the other direction
        except KeyboardInterrupt:
            GPIO.cleanup()


class FlowSensor:
    def __init__(self):
        # Transceiver has to be initialized. Can be done with "with" but would close after initialization
        self.i2c_transceiver = LinuxI2cTransceiver("/dev/i2c-1")
        channel = I2cChannel(
            I2cConnection(self.i2c_transceiver),
            slave_address=0x08,
            crc=CrcCalculator(8, 0x31, 0xFF, 0x0),
        )
        self.sensor = Sf06LfDevice(channel)
        self.volume = 0
        try:
            self.sensor.stop_continuous_measurement()  # Check if Open
            time.sleep(0.1)
        except BaseException:
            print("Ready to Start")
        self.prevtime = time.time()

    def close(self):  # Needs to close before ending
        self.i2c_transceiver.close()

    def start_measurement(self):
        self.sensor.start_h2o_continuous_measurement()
        self.prevtime = time.time()
        while self.volume < DESIRED_DISPLACEMENT:
            time.sleep(0.02)
            self.read_measurement()
            print(self.volume)

    def stop_measurement(self):
        self.sensor.stop_continuous_measurement()

    def read_measurement(self):
        try:
            flow, temperature, signaling_flags = self.sensor.read_measurement_data(
                InvFlowScaleFactors.SLF3C_1300F
            )
        except BaseException:
            return
        curTime = time.time()
        self.volume += flow * (curTime - self.prevtime) * TUBE_AREA  # in ml
        self.prevtime = curTime


temp = Stepper_Driver(DIR_PIN, STEP_PIN)
temp.setSpeed(DESIRED_SPEED)
temp.setDirection(GPIO.HIGH)
Sensor = FlowSensor()
Sensor.start_measurement()
temp.run()
