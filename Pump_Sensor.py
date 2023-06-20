import time
import RPi.GPIO as GPIO
from sensirion_i2c_driver import LinuxI2cTransceiver, I2cConnection, CrcCalculator
from sensirion_driver_adapters.i2c_adapter.i2c_channel import I2cChannel
from sensirion_i2c_sf06_lf.device import Sf06LfDevice
from sensirion_i2c_sf06_lf.commands import InvFlowScaleFactors
import math
from threading import Event, Thread

DESIRED_STEP_SPEED = 10000  # Desired steps per second
DIR_PIN = 22  # GPIO pin for direction signal
STEP_PIN = 27  # GPIO pin for step signal
MIN_PULSE_DURATION = 1.9e-6  # Minimum pulse duration in seconds (1.9us)

DESIRED_FLOW_RATE = 0.5  # Desired flow rate in ml/sec
FLOW_RATE_TOLERANCE = 0.01  # Tolerance for flow rate control
DESIRED_DISPLACEMENT = 5.0  # Desired total displacement in ml
ACCELERATION_STEPS = 1000  # The number of steps over which to accelerate
MICROSTEPS = 16  # Define the number of microsteps per full step

StopFlag = Event()


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
        desired_pulse_duration = 1.0 / (desired_speed * MICROSTEPS)

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
            step_count = 0
            while not StopFlag.is_set():
                step_count += 1
                # Acceleration
                if step_count < ACCELERATION_STEPS:
                    self.setSpeed(
                        (DESIRED_STEP_SPEED / ACCELERATION_STEPS) * step_count
                    )
                else:
                    self.setSpeed(DESIRED_STEP_SPEED)
                self.step()
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
        self.flow = 0.0
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
            time.sleep(0.0005)
            self.read_measurement()
        self.stop_measurement()

    def stop_measurement(self):
        StopFlag.set()
        self.sensor.stop_continuous_measurement()

    def read_measurement(self):
        try:
            (
                rawFlow,
                a_temperature,
                a_signaling_flags,
            ) = self.sensor.read_measurement_data(InvFlowScaleFactors.SLF3C_1300F)
            curTime = time.time()
            self.flow = abs(rawFlow.value)  # ml/min
            self.volume += self.flow * (curTime - self.prevtime) / 60.0  # in ml
            self.prevtime = curTime
        except BaseException:
            print(a_signaling_flags)


# Give some time for the measurement to start before starting the motor control
class FluidController:
    def __init__(self):
        self.driver = Stepper_Driver(DIR_PIN, STEP_PIN)
        self.sensor = FlowSensor()

    def control_loop(self):
        while self.sensor.volume < DESIRED_DISPLACEMENT and not StopFlag.is_set():
            if self.sensor.flow < DESIRED_FLOW_RATE - FLOW_RATE_TOLERANCE:
                self.driver.setSpeed(
                    self.driver.pulseDuration * 1.1
                )  # Increase speed by 10%
            elif self.sensor.flow > DESIRED_FLOW_RATE + FLOW_RATE_TOLERANCE:
                self.driver.setSpeed(
                    self.driver.pulseDuration * 0.9
                )  # Decrease speed by 10%
            time.sleep(0.1)  # Adjust the delay as needed

    def start(self):
        thread1 = Thread(target=self.sensor.start_measurement)
        thread2 = Thread(target=self.driver.run)
        thread3 = Thread(target=self.control_loop)

        thread1.start()
        time.sleep(0.5)  # Give some time for the measurement
        thread2.start()
        thread3.start()

        thread1.join()
        thread2.join()
        thread3.join()

        self.sensor.close()
        GPIO.cleanup()


controller = FluidController()
controller.start()
