#!/usr/bin/env python

########################
# Imports
########################
import time
import RPi.GPIO as GPIO
import math
from sensirion_i2c_driver import LinuxI2cTransceiver, I2cConnection, CrcCalculator
from sensirion_driver_adapters.i2c_adapter.i2c_channel import I2cChannel
from sensirion_i2c_sf06_lf.device import Sf06LfDevice
from sensirion_i2c_sf06_lf.commands import InvFlowScaleFactors

# Constants
TUBE_AREA = math.pi * (0.4 * 2.56) ** 2  # Area of Pipe
DESIRED_DISPLACEMENT = 50  # ml
STEP_PIN = 18  # GPIO pin for step signal
DIR_PIN = 17  # GPIO pin for direction signal
FLOW_SENSOR_PIN = 21  # GPIO pin for flow sensor input
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
        self.pulseSpeed = 500  # Steps per second
        self.pulseDuration = 1 / self.pulseSpeed
        self.maxSpeed = 1500
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

    def begin(self):
        try:
            while DESIRED_DISPLACEMENT > CURRENT_CHANGE:
                temp = self.pulseDuration
                GPIO.output(self.pinStep, GPIO.HIGH)
                time.sleep(temp / 2)
                GPIO.output(self.pinStep, GPIO.LOW)
                time.sleep(temp / 2)
        except KeyboardInterrupt:
            GPIO.cleanup()


class FlowSensor:
    def __init__(self, channel):
        self.device = Sf06LfDevice(channel=channel)
        self.prev_time = 0
        self.integrated_error = 0
        self.timediff = 0
        self.prev_voldiff = 0

    def start_measurement(self):
        self.device.start_h2o_continuous_measurement()
        self.timediff = time.time()

    def stop_measurement(self):
        self.device.stop_continuous_measurement()

    def read_measurement(self):
        global CURRENT_CHANGE
        flow, _, _ = self.device.read_measurement_data(InvFlowScaleFactors.SLF3C_1300F)
        currenttime = time.time()
        self.timediff = currenttime - self.prev_time
        CURRENT_CHANGE += flow * self.timediff * TUBE_AREA  # in ml
        self.prev_time = currenttime


# Initialize the devices
stepper_driver = Stepper_Driver()
flow_sensor = FlowSensor(channel=1)  # replace with your I2C channel
# Start continuous measurement
flow_sensor.start_measurement()
stepper_driver.begin()
try:
    while True:
        # Read measurement data
        flow_sensor.read_measurement()
        # Print the current flow, temperature, and total volume
        error = DESIRED_DISPLACEMENT - CURRENT_CHANGE
        flow_sensor.integrated_error += error * flow_sensor.timediff
        derivative = (error - flow_sensor.prev_voldiff) / flow_sensor.timediff
        speed = P * error + I * flow_sensor.integrated_error + D * derivative
        stepper_driver.setSpeed(speed)
        flow_sensor.prev_voldiff = error
          # Wait for 1 second
        time.sleep(1)
except KeyboardInterrupt:
    # Stop continuous measurement
    flow_sensor.stop_measurement()
    GPIO.cleanup()
