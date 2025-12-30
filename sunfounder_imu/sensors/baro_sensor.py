
from .._base import _Base

class BaroSensor(_Base):
    """
    Barometer class to interface with barometer sensor.

    Args:
        address (int): I2C address of the barometer sensor.
        sea_level_pressure (float, optional): Sea level pressure in Pascals. Default is 1013.25 hPa.
        offset (float, optional): Offset value in Pascals. Default is 0.0.
    """
    P0 = 1013.25  # Standard sea level pressure in hPa


    def __init__(self, address: int, *args, sea_level_pressure: float = P0, offset: float = 0.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.address = address

        self.sea_level_pressure = sea_level_pressure
        self.offset = offset

        self.temperature = None
        self.pressure = None
        self.altitude = None

    def read_temperature(self) -> float:
        raise NotImplementedError("Subclasses must implement get_temperature method")

    def read_pressure(self) -> float:
        raise NotImplementedError("Subclasses must implement read_pressure method")

    def read_altitude(self, pressure=None) -> float:
        """ Calculate altitude based on pressure
        
        Args:
            pressure (float): Pressure in Pascals (default: current pressure)

        Returns:
            float: Altitude in meters
        """
        # Use provided sea level pressure P0 or the stored one
        if pressure is None:
            pressure = self.read_pressure()
        
        # Barometric formula for altitude calculation
        altitude = 44330.0 * (1.0 - ((pressure / self.sea_level_pressure) ** 0.1903))
        
        return altitude
    
    def read(self) -> (float, float, float):
        """ Get all measurements in one call
            
        Returns:
            tuple: (temperature, pressure, altitude)
        """
        self.temperature = self.read_temperature()
        self.pressure = self.read_pressure()
        self.altitude = self.read_altitude(pressure=self.pressure)
        
        return (self.temperature, self.pressure, self.altitude)

    def set_offset(self, offset: float) -> None:
        """ Set the offset for pressure calculations
        
        Args:
            offset (float): Offset value in Pascals
        """
        self.offset = offset

    def set_sea_level_pressure(self, pressure: float) -> None:
        """ Set the sea level pressure for altitude calculations
        
        Args:
            pressure (float): Sea level pressure in Pascals
        """
        self.sea_level_pressure = pressure

    def set_calibration_data(self, sea_level_pressure: float, offset: float) -> None:
        """ Set both sea level pressure and offset
        
        Args:
            sea_level_pressure (float): Sea level pressure in Pascals
            offset (float): Offset value in Pascals
        """
        self.set_sea_level_pressure(sea_level_pressure)
        self.set_offset(offset)