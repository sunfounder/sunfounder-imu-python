from ._i2c import I2C
from .spl06_001 import SPL06_001
from .qmc6310 import QMC6310
from .sh3001 import SH3001

from .data_type import AccelDate, GyroDate, MagDate, BarometerDate

class IMU:
    BAROMETER_ADDRESS_MAP = {
        0x76: SPL06_001,
        0x77: SPL06_001,
    }

    MAGNETOMETER_ADDRESS_MAP = {
        0x1C: QMC6310,
        0x3C: QMC6310,
    }

    ACCEL_GYRO_ADDRESS_MAP = {
        0x36: SH3001,
        0x37: SH3001,
    }

    def __init__(self):
        self._temperature = None
        self.acceleration = AccelDate(0, 0, 0)
        self.gyroscrope = GyroDate(0, 0, 0)
        self.magenatic = MagDate(0, 0, 0)
        self.barometer = BarometerDate(0, 0, 0)

        self.barometer_sensor = None
        self.accel_gyro_sensor = None
        self.magenatic_sensor = None

        self.temperature = None
        self.pressure = None
        self.altitude = None

        addresses = I2C.scan()
        for address in addresses:
            if address in self.BAROMETER_ADDRESS_MAP:
                self.barometer_sensor = self.BAROMETER_ADDRESS_MAP[address]()
            if address in self.ACCEL_GYRO_ADDRESS_MAP:
                self.accel_gyro_sensor = self.ACCEL_GYRO_ADDRESS_MAP[address]()
            if address in self.MAGNETOMETER_ADDRESS_MAP:
                self.magenatic_sensor = self.MAGNETOMETER_ADDRESS_MAP[address]()


    def read(self):
        """ Get all measurements in one call
            
        Returns:
            dict: Dictionary containing temperature, pressure, and altitude
        """
        if self.barometer_sensor is not None:
            self.temperature, self.pressure, self.altitude = self.barometer_sensor.read()
            self.barometer.update(self.pressure, self.temperature, self.altitude)
        if self.accel_gyro_sensor is not None:
            self.acceleration, self.gyroscrope = self.accel_gyro_sensor.read()
        if self.magenatic_sensor is not None:
            self.magenatic = self.magenatic_sensor.read()
