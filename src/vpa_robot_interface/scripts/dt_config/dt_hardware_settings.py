import dataclasses
import RPi.GPIO as GPIO

from typing import Dict
from abc import abstractmethod, ABC
from enum import IntEnum
from adafruit_drivers.Adafruit_PWM_Servo_Driver import PWM

'''Motor'''

LOW         = 0
HIGH        = 1

GPIO_LOW    = LOW
GPIO_HIGH   = HIGH

PWM_LOW     = 0
PWM_HIGH    = 4096

class MotorDirection(IntEnum):
    RELEASE     = 0
    FORWARD     = 1
    BACKWARD    = -1

class MotorDirectionControl(IntEnum):
    PWM = 0
    GPIO = 1

@dataclasses.dataclass
class MotorPins:
    in1: int
    in2: int
    pwm: int
    control: MotorDirectionControl

class AbsMotorDirectionController(ABC):
    def __init__(self, in1_pin: int, in2_pin: int, *args, **kwargs):
        self._in1_pin = in1_pin
        self._in2_pin = in2_pin
        self.setup()

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def set(self, direction: MotorDirection):
        pass

    def __str__(self):
        return f"{self.__class__.__name__}[in1={self._in1_pin}, in2={self._in2_pin}]"

class PWMMotorDirectionController(AbsMotorDirectionController):
    _DIRECTION_TO_SIGNALS = {
        MotorDirection.RELEASE: (LOW, HIGH),
        MotorDirection.FORWARD: (HIGH, LOW),
        MotorDirection.BACKWARD: (LOW, HIGH),
    }
    _PWM_VALUES = {
        LOW: (PWM_LOW, PWM_HIGH),
        HIGH: (PWM_HIGH, PWM_LOW),
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not isinstance(kwargs.get("pwm", None), PWM):
            raise ValueError(
                "You cannot instantiate `PWMMotorDirectionController` " "without passing a `PWM` object."
            )
        self._pwm = kwargs["pwm"]

    def setup(self):
        pass

    def set(self, direction: MotorDirection):
        in1_signal, in2_signal = self._DIRECTION_TO_SIGNALS[direction]
        in1_value, in2_value = self._PWM_VALUES[in1_signal], self._PWM_VALUES[in2_signal]
        self._pwm.setPWM(self._in1_pin, *in1_value)
        self._pwm.setPWM(self._in2_pin, *in2_value)

class GPIOMotorDirectionController(AbsMotorDirectionController):
    _DIRECTION_TO_SIGNALS = {
        MotorDirection.RELEASE: (HIGH, HIGH),
        MotorDirection.FORWARD: (HIGH, LOW),
        MotorDirection.BACKWARD: (LOW, HIGH),
    }
    _GPIO_VALUES = {LOW: GPIO_LOW, HIGH: GPIO_HIGH}

    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._in1_pin, GPIO.OUT)
        GPIO.setup(self._in2_pin, GPIO.OUT)

    def set(self, direction: MotorDirection):
        in1_signal, in2_signal = self._DIRECTION_TO_SIGNALS[direction]
        in1_value, in2_value = self._GPIO_VALUES[in1_signal], self._GPIO_VALUES[in2_signal]
        GPIO.output(self._in1_pin, in1_value)
        GPIO.output(self._in2_pin, in2_value)

class Motor:
    _K = 16
    _CONTROLLER = {
        MotorDirectionControl.PWM: PWMMotorDirectionController,
        MotorDirectionControl.GPIO: GPIOMotorDirectionController,
    }

    def __init__(
        self, name: str, pwm: PWM, in1_pin: int, in2_pin: int, pwm_pin: int, control: MotorDirectionControl
    ):
        self._pwm = pwm
        self._name = name
        self._in1_pin = in1_pin
        self._in2_pin = in2_pin
        self._pwm_pin = pwm_pin
        self._control = control
        self._controller = self._CONTROLLER[control](in1_pin, in2_pin, pwm=self._pwm)

    def set(self, direction: MotorDirection, speed: int = 0):
        speed = max(0, min(speed, 255))
        self._controller.set(direction)
        self._pwm.setPWM(self._pwm_pin, 0, speed * self._K)

    def __str__(self):
        return (
            f"Motor[name={self._name}, in1={self._in1_pin}, in2={self._in2_pin}, "
            f"pwm={self._pwm_pin}, controller={self._controller}]"
        )


'''HAT'''

class AbsHAT(ABC):
    def __init__(self, address=0x60, frequency=1600):
        # default I2C address of the HAT
        self._i2caddr = address
        # default @1600Hz PWM frequency
        self._frequency = frequency
        # configure PWM
        self._pwm = PWM(self._i2caddr, debug=False)
        self._pwm.setPWMFreq(self._frequency)

    @abstractmethod
    def get_motor(self, num: int, name: str) -> Motor:
        pass

class HATv1(AbsHAT):
    _MOTOR_NUM_TO_PINS: Dict[int, MotorPins] = {
        1: MotorPins(10, 9, 8, MotorDirectionControl.PWM),
        2: MotorPins(11, 12, 13, MotorDirectionControl.PWM),
        3: MotorPins(4, 3, 2, MotorDirectionControl.PWM),
        4: MotorPins(5, 6, 7, MotorDirectionControl.PWM),
    }

    def get_motor(self, num: int, name: str) -> Motor:
        if num not in self._MOTOR_NUM_TO_PINS:
            raise ValueError(
                f"Motor num `{num}` not supported. "
                f"Possible choices are `{self._MOTOR_NUM_TO_PINS.keys()}`."
            )
        pins = self._MOTOR_NUM_TO_PINS[num]
        return Motor(name, self._pwm, pins.in1, pins.in2, pins.pwm, control=pins.control)

class HATv2(HATv1):
    pass