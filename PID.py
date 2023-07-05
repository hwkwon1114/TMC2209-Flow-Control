import time
import RPi.GPIO as GPIO
from sensirion_i2c_driver import LinuxI2cTransceiver, I2cConnection, CrcCalculator
from sensirion_driver_adapters.i2c_adapter.i2c_channel import I2cChannel
from sensirion_i2c_sf06_lf.device import Sf06LfDevice
from sensirion_i2c_sf06_lf.commands import InvFlowScaleFactors
import math
from threading import Event, Thread

DESIRED_STEP_SPEED = 20000 / 60  # Desired steps per second
DIR_PIN = 22  # GPIO pin for direction signal
STEP_PIN = 27  # GPIO pin for step signal
MIN_PULSE_DURATION = 1.9e-6  # Minimum pulse duration in seconds (1.9us)
MEASUREMENT_DELAY = 0.0005  # Has to be less than 100 ms (0.1s)
DESIRED_FLOW_RATE = 10  # Desired flow rate in ml/min
FLOW_RATE_TOLERANCE = 0.01  # Tolerance for flow rate control
DESIRED_DISPLACEMENT = 5.0  # Desired total displacement in ml
ACCELERATION_STEPS = 1100  # The number of steps over which to accelerate
STEP_TO_RAD = 360 / 200
MICROSTEPS = 16  # Define the number of microsteps per full step

StopFlag = Event()


class Stepper_Driver(object):
    def __init__(self, pinDir, pinStep):
        self.pinDir = pinDir
        self.pinStep = pinStep
        self.size = (
            90 / (DESIRED_STEP_SPEED * STEP_TO_RAD * MEASUREMENT_DELAY) * 1.1
        )  # Seconds divided by delay
        self.pulseDuration = MIN_PULSE_DURATION
        self.StepSpeed = DESIRED_STEP_SPEED
        self.direction = GPIO.LOW
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinStep, GPIO.OUT)
        GPIO.setup(self.pinDir, GPIO.OUT)
        self.changing = False

    def setSpeed(self, desired_speed):
        # Calculate pulse duration based on desired speed
        desired_pulse_duration = 1.0 / (desired_speed * MICROSTEPS)
        # Ensure pulse duration is not less than the minimum
        if desired_pulse_duration < MIN_PULSE_DURATION:
            print("Desired speed is too high, setting to maximum speed.")
            self.pulseDuration = MIN_PULSE_DURATION
        else:
            self.pulseDuration = desired_pulse_duration

    def accelerateSpeed(self, Newspeed):
        stepcount = 0
        self.changing = True
        diff = Newspeed - self.StepSpeed
        while stepcount < ACCELERATION_STEPS:
            stepcount += 1
            self.setSpeed(self.StepSpeed + diff * (stepcount / ACCELERATION_STEPS))
            self.step()
        self.StepSpeed = Newspeed
        self.changing = False

    def setDirection(self, Direction):
        self.direction = Direction

    def updateSize(self):
        self.size = 90 / (
            self.StepSpeed * STEP_TO_RAD * MEASUREMENT_DELAY
        )  # Update to the new size

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
                if self.changing == False:
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
        self.circularBuffer = CircularBuffer(
            90 / (DESIRED_STEP_SPEED * STEP_TO_RAD * MEASUREMENT_DELAY)
        )

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
        self.prevtime = time.perf_counter_ns()

    def close(self):  # Needs to close before ending
        self.i2c_transceiver.close()

    def start_measurement(self):
        self.sensor.start_h2o_continuous_measurement()
        self.prevtime = time.perf_counter_ns()
        while self.volume < DESIRED_DISPLACEMENT and not StopFlag.is_set():
            time.sleep(MEASUREMENT_DELAY)
            self.read_measurement()
            # print(self.volume)
        self.stop_measurement()

    def stop_measurement(self):
        StopFlag.set()
        self.sensor.stop_continuous_measurement()

    def read_measurement(self):
        try:
            (
                rawFlow,
                _,
                a_signaling_flags,
            ) = self.sensor.read_measurement_data(InvFlowScaleFactors.SLF3C_1300F)
            # print(rawFlow,"-", a_signaling_flags)
            curTime = time.perf_counter_ns()
            self.volume += (
                rawFlow.value * (curTime - self.prevtime) / 1000000000.0 / 60.0
            )  # in ml
            self.circularBuffer.addMeasurement(rawFlow.value)
            self.prevtime = curTime
        except BaseException:
            print(a_signaling_flags)

    def returnAverage(self):
        return self.circularBuffer.findAverage()


class CircularBuffer:
    def __init__(self, size):
        self.size = round(size)
        self.buffer = [None] * self.size
        self.index = 0
        self.is_full = False

    def addMeasurement(self, measurement):
        self.buffer[self.index] = measurement
        self.index = (self.index + 1) % self.size
        if not self.is_full and self.index == 0:
            self.is_full = True

    def findAverage(self, count):
        count1 = round(count)
        if count1 > self.size:
            count1 = self.size
        elif count1 == 0:
            return 0
        start_index = self.index - count1 if self.is_full else 0
        return sum(self.buffer[start_index : self.index]) / count1


# Give some time for the measurement to start before starting the motor control
class FluidController:
    def __init__(self):
        self.driver = Stepper_Driver(DIR_PIN, STEP_PIN)
        self.sensor = FlowSensor()
        self.desiredFlowrate = 20  # ml/min
        self.kP = 0.5
        self.kI = 0.5
        self.kD = 0.1
        self.lasterror = 0
        self.integrated_error = 0
        self.time = 0

    def control_loop(self):
        while self.sensor.volume < DESIRED_DISPLACEMENT and not StopFlag.is_set():
            averageFlow = self.sensor.returnAverage(
                90 / (self.driver.StepSpeed * STEP_TO_RAD * MEASUREMENT_DELAY)
            )
            timenow = time.perf_counter_ns() / 1000000000.0
            error = DESIRED_FLOW_RATE - averageFlow
            if self.time != 0:
                self.integrated_error += (timenow - self.time) * error
            derivTerm = error - self.lasterror
            self.lasterror = error
            self.time = timenow
            self.driver.accelerateSpeed(
                self.kP * error + self.kI * self.integrated_error + self.kD * derivTerm
            )  # Decrease the speed
            time.sleep(0.5)  # Adjust the delay as needed

    def start(self):
        thread1 = Thread(target=self.sensor.start_measurement)
        thread2 = Thread(target=self.driver.run)
        thread3 = Thread(target=self.control_loop)

        thread1.start()
        time.sleep(0.5)  # Give some time for the measurement
        thread2.start()
        time.sleep(0.1)
        thread3.start()

        thread1.join()
        thread2.join()
        thread3.join()

        self.sensor.close()
        GPIO.cleanup()


controller = FluidController()
controller.start()
