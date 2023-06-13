import time
import RPi.GPIO as GPIO
from sensirion_i2c_driver import LinuxI2cTransceiver, I2cConnection, CrcCalculator
from sensirion_driver_adapters.i2c_adapter.i2c_channel import I2cChannel
from sensirion_i2c_sf06_lf.device import Sf06LfDevice
from sensirion_i2c_sf06_lf.commands import InvFlowScaleFactors
import math
import threading

DESIRED_STEP_SPEED = 10000  # Desired speed iFluids.pyn steps per second
DIR_PIN = 22  # GPIO pin for direction signal
STEP_PIN = 27  # GPIO pin for step signal
MIN_PULSE_DURATION = 1.9e-6  # Minimum pulse duration in seconds (1.9us)
DESIRED_DISPLACEMENT = 0.5  # ml
StopFlag = threading.Event()


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
        time.sleep(self.pulseDuration / 2)  # Split delay to half
        GPIO.output(self.pinStep, GPIO.LOW)
        time.sleep(self.pulseDuration / 2)

    def run(self):
        try:
            while not StopFlag.is_set():
                self.step()  # Replace with GPIO.LOW for the other direction
        except KeyboardInterrupt:
            StopFlag.set()


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
        self.volume = 0.0
        try:
            self.sensor.stop_continuous_measurement()  # Check if Open
            time.sleep(0.1)
        except BaseException:
            print("Ready to Start")
        (product_identifier, serial_number) = self.sensor.read_product_identifier()
        print(
            f"product_identifier: {product_identifier}; "
            f"serial_number: {serial_number}; "
        )
        self.prevtime = time.time()

    def close(self):  # Needs to close before ending
        self.i2c_transceiver.close()

    def start_measurement(self):
        self.sensor.start_h2o_continuous_measurement()
        self.prevtime = time.time()
        while self.volume < DESIRED_DISPLACEMENT and not StopFlag.is_set():
            time.sleep(0.02)
            self.read_measurement()
            print(self.volume)
        self.stop_measurement()

    def stop_measurement(self):
        self.sensor.stop_continuous_measurement()

    def read_measurement(self):
        try:
            (
                rawFlow,
                a_temperature,
                a_signaling_flags,
            ) = self.sensor.read_measurement_data(InvFlowScaleFactors.SLF3C_1300F)
            curTime = time.time()
            flow = abs(rawFlow.value)  # ml/min
            self.volume += flow * (curTime - self.prevtime) / 60.0  # in ml
            self.prevtime = curTime
        except BaseException:
            print(a_signaling_flags)


temp = Stepper_Driver(DIR_PIN, STEP_PIN)
temp.setSpeed(DESIRED_STEP_SPEED)
temp.setDirection(GPIO.LOW)
Sensor = FlowSensor()
thread1 = threading.Thread(target=Sensor.start_measurement)
thread2 = threading.Thread(target=temp.run)
thread1.start()
# Give some time for the measurement to start before starting the motor control
time.sleep(0.5)  # Adjust the delay as needed
# Start the motor control thread
thread2.start()
# Wait for both threads to complete
thread1.join()
thread2.join()
Sensor.close()
GPIO.cleanup()
