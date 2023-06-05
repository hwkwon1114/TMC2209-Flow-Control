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

TUBE_AREA = math.pi * (0.4 * 2.56)**2 #Area of Pipe
DESIRED_DISPLACEMENT = 0.5 #ml
CURRENT_CHANGE = 0 #ml

# Constants
STEP_PIN = 18        # GPIO pin for step signal
DIR_PIN = 17          # GPIO pin for direction signal
FLOW_SENSOR_PIN = 17   # GPIO pin for flow sensor input
PULSES_PER_REV = 256 * 200   # Number of pulses for one full revolution of the motor
P = 0.1
I = 0.5
D = 0.1

import RPi.GPIO as GPIO

class Stepper_Driver(object):
    def __init__(self, pulsesPerStep, stepsPerRev):
        self._pinStep = STEP_PIN
        self._pinDir = DIR_PIN
        self._pulsesPerStep = pulsesPerStep
        self._stepsPerRev = stepsPerRev
        self._pulseCountAbs = 0
        self._pulseSpeed = -1
        self._pulseAcceleration = -1

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pinStep, GPIO.OUT)
        GPIO.setup(self._pinDir, GPIO.OUT)


    def setSpeed(self, desired_speed, acceleration_rate):
        time_to_reach_speed = (desired_speed - self._pulseSpeed) / acceleration_rate
        steps_to_reach_speed = 0.5 * acceleration_rate * (time_to_reach_speed ** 2)
        self._pulseSpeedTarget = desired_speed
        self._pulseAcceleration = acceleration_rate
        self._pulseCountAbsTarget = self._pulseCountAbs + steps_to_reach_speed

class FlowSensor:
    def __init__(self, channel):
        self.device = Sf06LfDevice(channel=channel)
        self.volume = 0.0

    def start_measurement(self):
        self.device.start_h2o_continuous_measurement()

    def stop_measurement(self):
        self.device.stop_continuous_measurement()

    def read_measurement(self):
        flow, temperature, signaling_flags = self.device.read_measurement_data(InvFlowScaleFactors.SLF3S_1300F)
        self.volume += flow * time. # in ml
        return flow, temperature, signaling_flags, self.volume

# Initialize the device
flow_sensor = FlowSensor(channel=1)  # replace with your I2C channel

# Start continuous measurement
flow_sensor.start_measurement()

try:
    while True:
        # Read measurement data
        flow, temperature, signaling_flags, volume = flow_sensor.read_measurement()

        # Print the current flow, temperature, and total volume
        print(f"Flow: {flow} ml/min, Temperature: {temperature} °C, Total Volume: {volume} ml")

        # Wait for 1 second
        time.sleep(1)
except KeyboardInterrupt:
    # Stop continuous measurement
    flow_sensor.stop_measurement()
