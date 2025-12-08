
class ThreeAxisData:
    def __init__(self, x, y, z):
        self.update(x, y, z)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    def list(self):
        return [self.x, self.y, self.z]

    def __str__(self):
        return f"{self.__class__.__name__}(x={self.x}, y={self.y}, z={self.z})"

    def update(self, x, y, z):
        self._x = x
        self._y = y
        self._z = z

class AccelDate(ThreeAxisData):
    pass

class GyroDate(ThreeAxisData):
    pass

class MagDate(ThreeAxisData):
    pass

class BarometerDate():
    def __init__(self, pressure, temperature, altitude):
        self.update(pressure, temperature, altitude)

    @property
    def pressure(self):
        return self._pressure

    @property
    def temperature(self):
        return self._temperature

    @property
    def altitude(self):
        return self._altitude

    def __str__(self):
        return f"{self.__class__.__name__}(pressure={self.pressure:.2f} hPa, temperature={self.temperature:.2f} °C, altitude={self.altitude:.2f} m)"

    def update(self, pressure, temperature, altitude):
        self._pressure = pressure
        self._temperature = temperature
        self._altitude = altitude
