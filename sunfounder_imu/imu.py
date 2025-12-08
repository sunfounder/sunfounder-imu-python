from ._i2c import I2C
from .spl06_001 import SPL06_001
from .data_type import AccelDate, GyroDate, MagDate, BarometerDate

class IMU:
    BAROMETER_ADDRESS_MAP = {
        0x76: SPL06_001,
        0x77: SPL06_001,
    }

    def __init__(self):
        self._temperature = None
        self.acceleration = AccelDate(0, 0, 0)
        self.gyroscrope = GyroDate(0, 0, 0)
        self.magenatic = MagDate(0, 0, 0)
        self.barometer = BarometerDate(0, 0, 0)

        self.barometer_sensor = None
        self.accelerometer_sensor = None
        self.gyroscrope_sensor = None
        self.magenatic_sensor = None

        self.temperature = None
        self.pressure = None
        self.altitude = None

        addresses = I2C.scan()
        for address in addresses:
            if address in self.BAROMETER_ADDRESS_MAP:
                self.barometer_sensor = self.BAROMETER_ADDRESS_MAP[address]()
                break

    def read(self):
        """ Get all measurements in one call
            
        Returns:
            dict: Dictionary containing temperature, pressure, and altitude
        """
        if self.barometer_sensor is not None:
            self.temperature, self.pressure, self.altitude = self.barometer_sensor.read()
            self.barometer.update(self.pressure, self.temperature, self.altitude)
