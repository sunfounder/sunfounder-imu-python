"""SPL06-01 Barometer Sensor Library

This module provides a Python interface for the SPL06-01 barometer sensor,
allowing for accurate pressure and temperature measurements.
"""

from ._base import _Base
from ._i2c import I2C
from ._utils import twos_complement
import time

class SPL06_001(_Base):
    """ SPL06-001 Barometer Sensor Class
    
    This class provides an interface to the SPL06-01 barometer sensor,
    allowing for pressure and temperature measurements with configurable
    precision and sampling rates.

    Args:
        *args: Variable length argument list for base class
        address (int): I2C address of the sensor (default: None)
        mode (MODE): Measurement mode (default: ContinuousPressureAndTemperatureMeasurement)
        press_osr (OVERSAMPLING_RATE): Pressure oversampling rate (default: RATE_1024)
        temp_osr (OVERSAMPLING_RATE): Temperature oversampling rate (default: RATE_1024)
        **kwargs: Additional keyword arguments for base class
    """
    # I2C Address
    I2C_ADDRESSES = [0x76, 0x77]

    REG_PRESSURE = 0x00
    REG_TEMPERATURE = 0x03
    REG_PRESSURE_CONFIGURATION = 0x06
    REG_TEMPERATURE_CONFIGURATION = 0x07
    REG_MEASUREMENT_CONFIGURATION = 0x08
    REG_CONFIGURATION_REGISTER = 0x09
    REG_INTERNAL_STATUS = 0x0A
    REG_FIFO_STATUS = 0x0B
    REG_RESET = 0x0C
    REG_ID = 0x0D
    REG_C0 = 0x10
    REG_C1 = 0x11
    REG_C00 = 0x13
    REG_C10 = 0x15
    REG_C01 = 0x18
    REG_C11 = 0x1A
    REG_C20 = 0x1C
    REG_C21 = 0x1E
    REG_C30 = 0x20

    MODE_IDLE = 0
    MODE_PRESSURE = 1
    MODE_TEMPERATURE = 2
    MODE_CONTINUE_PRESSURE = 5
    MODE_CONTINUE_TEMPERATURE = 6
    MODE_CONTINUE_PRESSURE_AND_TEMPERATURE = 7

    RATE_1 = 0
    RATE_2 = 1
    RATE_4 = 2
    RATE_8 = 3
    RATE_16 = 4
    RATE_32 = 5
    RATE_64 = 6
    RATE_128 = 7

    TEMPERATURE_EXTERNAL = 1

    SCALE_FACTORS = [
        524288,    # RATE_1 (single)
        1572864,   # RATE_2 (2 times Low Power)
        3670016,   # RATE_4 (4 times)
        7864320,   # RATE_8 (8 times)
        253952,    # RATE_16 (16 times Standard)
        516096,    # RATE_32 (32 times)
        1040384,   # RATE_64 (64 times High Precision)
        2088960    # RATE_128 (128 times)
    ]

    P0 = 1013.25  # Standard sea level pressure in hPa

    def __init__(self,
        *args,
        address=None,
        mode=MODE_CONTINUE_PRESSURE_AND_TEMPERATURE,
        pressure_rate=RATE_128,
        pressure_precision=RATE_128,
        temperature_rate=RATE_128,
        temperature_precision=RATE_128,
        **kwargs
        ):
        super().__init__(*args, **kwargs)
        
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES)
            if addresses:
                address = addresses[0]
        self.i2c = I2C(address=address)
        self.address = address

        self._init(mode, pressure_rate, pressure_precision, temperature_rate, temperature_precision)

    def set_pressure_configuration(self, rate, precision):
        """ Set the pressure oversampling rate
        
        Args:
            rate (OVERSAMPLING_RATE): Pressure oversampling rate
        """
        self.pressure_rate = rate
        self.pressure_precision = precision
        self.pressure_scale_factor = self.SCALE_FACTORS[precision]
        # Update pressure configuration
        pressure_config = (self.pressure_rate << 4) | self.pressure_precision
        self.i2c.write_byte_data(self.REG_PRESSURE_CONFIGURATION, pressure_config)

    def get_pressure_configuration(self):
        """ Get the current pressure configuration
        
        Returns:
            tuple: (rate, precision)
        """
        data = self.i2c.read_byte_data(self.REG_PRESSURE_CONFIGURATION)
        rate = data >> 4 & 0x07
        precision = data & 0x07
        return rate, precision

    def set_temperature_configuration(self, rate, precision):
        """ Set the temperature oversampling rate
        
        Args:
            rate (OVERSAMPLING_RATE): Temperature oversampling rate
        """
        self.temperature_external = self.TEMPERATURE_EXTERNAL
        self.temperature_rate = rate
        self.temperature_precision = precision
        self.temperature_scale_factor = self.SCALE_FACTORS[precision]
        # Update temperature configuration
        temperature_config = self.temperature_external << 7 | (self.temperature_rate << 4) | self.temperature_precision
        self.i2c.write_byte_data(self.REG_TEMPERATURE_CONFIGURATION, temperature_config)

    def get_temperature_configuration(self):
        """ Get the current temperature configuration
        
        Returns:
            tuple: (rate, precision)
        """
        data = self.i2c.read_byte_data(self.REG_TEMPERATURE_CONFIGURATION)
        external = data >> 7 & 0x01
        rate = data >> 4 & 0x07
        precision = data & 0x07
        return external, rate, precision

    def wait_ready(self, timeout = 2000):
        """ Wait for the sensor to be ready
        
        Args:
            timeout (int): Maximum wait time in milliseconds (default: 2000)
        
        Returns:
            tuple: (coefficients_ready, sensor_ready, temperature_ready, pressure_ready)
        """
        # Wait for coefficients to be ready
        start_time = time.time()
        while True:
            data = self.i2c.read_byte_data(self.REG_MEASUREMENT_CONFIGURATION)
            coefficients_ready = data & 0x80 > 7
            sensor_ready = data & 0x40 > 6
            temperature_ready = data & 0x20 > 5
            pressure_ready = data & 0x10 > 4
            if coefficients_ready and sensor_ready and temperature_ready and pressure_ready:
                break
            if (time.time() - start_time) * 1000 > timeout:
                string_not_ready = ""
                if not coefficients_ready:
                    string_not_ready += "coefficients "
                if not sensor_ready:
                    string_not_ready += "sensor "
                if not temperature_ready:
                    string_not_ready += "temperature "
                if not pressure_ready:
                    string_not_ready += "pressure "
                raise TimeoutError(f"Sensor readiness check timed out, not ready: {string_not_ready}")
            time.sleep(0.01)
        return coefficients_ready, sensor_ready, temperature_ready, pressure_ready

    def _init(self, mode, pressure_rate, pressure_precision, temperature_rate, temperature_precision):
        """ Initialize the barometer sensor
        
        This method resets the sensor, reads calibration coefficients,
        and configures the measurement parameters.

        Args:
            mode (MODE): Measurement mode
            pressure_rate (OVERSAMPLING_RATE): Pressure oversampling rate
            pressure_precision (OVERSAMPLING_RATE): Pressure precision
            temperature_rate (OVERSAMPLING_RATE): Temperature oversampling rate
            temperature_precision (OVERSAMPLING_RATE): Temperature precision
        """
        # Reset the sensor
        self.reset()
        # Wait for reset to complete
        time.sleep(0.01)

        # set mode
        self.mode = mode
        self.i2c.write_byte_data(self.REG_MEASUREMENT_CONFIGURATION, self.mode)
        # 当使用大于8倍的过采样率时，必须设置P_SHIFT和T_SHIFT
        
        cfg_reg = 0
        # If pressure precision is greater than 8 times, set P_SHIFT 1
        if pressure_precision > 3:
            cfg_reg |= (1 << 2)
        # If temperature precision is greater than 8 times, set T_SHIFT 1
        if temperature_precision > 3:
            cfg_reg |= (1 << 3)
        self.i2c.write_byte_data(self.REG_CONFIGURATION_REGISTER, cfg_reg)

        # set pressure configuration
        self.set_pressure_configuration(pressure_rate, pressure_precision)
        # set temperature configuration
        self.set_temperature_configuration(temperature_rate, temperature_precision)

        # Read sensor ID
        self._read_id()
        
        # Wait for sensor readiness
        self.wait_ready()

        # Read calibration coefficients
        self._read_calibration_coefficients()

    def reset(self):
        """ Reset the sensor to default settings
        """
        self.i2c.write_byte_data(self.REG_RESET, 0b1001)

    def _read_id(self):
        """ Get the sensor ID
        """
        self.id = self.i2c.read_byte_data(self.REG_ID)

    def _read_calibration_coefficients(self):
        """ Read calibration coefficients from the sensor
        """
        # Read all calibration coefficients
        data = self.i2c.read_i2c_block_data(self.REG_C0, 18)

        self.calibration_coefficients = {
            "c0": twos_complement(data[0] << 4 | data[1] >> 4, 12),
            "c1": twos_complement((data[1] & 0x0F) << 8 | data[2], 12),
            "c00": twos_complement((data[3] << 12) | (data[4] << 4) | (data[5] >> 4), 20),
            "c10": twos_complement((data[5] & 0x0F) << 16 | (data[6] << 8) | data[7], 20),
            "c01": twos_complement((data[8] << 8) | data[9], 16),
            "c11": twos_complement((data[10] << 8) | data[11], 16),
            "c20": twos_complement((data[12] << 8) | data[13], 16),
            "c21": twos_complement((data[14] << 8) | data[15], 16),
            "c30": twos_complement((data[16] << 8) | data[17], 16),
        }

    def _get_raw_temperature(self) -> int:
        """ Read raw temperature data from the sensor
        
        Returns:
            int: Raw temperature value
        """
        # Read 24-bit temperature data (3 bytes)
        data = self.i2c.read_i2c_block_data(self.REG_TEMPERATURE, 3)
        value = (data[0] << 16) | (data[1] << 8) | data[2]
        value = twos_complement(value, 24)
        return value
    
    def _get_raw_pressure(self) -> int:
        """ Read raw pressure data from the sensor
        
        Returns:
            int: Raw pressure value
        """
        # Read 24-bit pressure data (3 bytes)
        data = self.i2c.read_i2c_block_data(self.REG_PRESSURE, 3)
        value = (data[0] << 16) | (data[1] << 8) | data[2]
        value = twos_complement(value, 24)
        return value
    
    def get_temperature(self) -> float:
        """ Get the current temperature in Celsius
        
        Returns:
            float: Temperature in degrees Celsius
        """
        # Read raw temperature
        t_raw = self._get_raw_temperature()
        
        # Calculate scaled temperature
        t_raw_sc = float(t_raw) / self.temperature_scale_factor
        
        # Apply calibration using coefficients
        c0 = self.calibration_coefficients['c0']
        c1 = self.calibration_coefficients['c1']
        
        # Temperature compensation formula from datasheet
        temperature = (c0 * 0.5) + (c1 * t_raw_sc)
        
        return temperature
    
    def get_pressure(self) -> float:
        """ Get the current pressure in Pascals
        
        Returns:
            float: Pressure in hPascals
        """
        # Read raw values
        t_raw = self._get_raw_temperature()
        p_raw = self._get_raw_pressure()
        
        # Calculate scaled values
        t_raw_sc = float(t_raw) / self.temperature_scale_factor
        p_raw_sc = float(p_raw) / self.pressure_scale_factor
        
        # Get calibration coefficients
        c00 = self.calibration_coefficients['c00']
        c10 = self.calibration_coefficients['c10']
        c20 = self.calibration_coefficients['c20']
        c30 = self.calibration_coefficients['c30']
        c01 = self.calibration_coefficients['c01']
        c11 = self.calibration_coefficients['c11']
        c21 = self.calibration_coefficients['c21']
        
        # Apply pressure compensation formula from datasheet
        pressure = c00 + p_raw_sc * (c10 + p_raw_sc * (c20 + p_raw_sc * c30))
        pressure += t_raw_sc * c01 + t_raw_sc * p_raw_sc * (c11 + p_raw_sc * c21)
        
        return pressure / 100.0
    
    def get_altitude(self, p0=None, pressure=None) -> float:
        """ Calculate altitude based on pressure
        
        Args:
            p0 (float): Sea level pressure in Pascals (default: stored value)

        Returns:
            float: Altitude in meters
        """
        # Use provided sea level pressure P0 or the stored one
        p0 = p0 if p0 is not None else self.P0
        
        pressure = pressure if pressure is not None else self.get_pressure()
        
        # Barometric formula for altitude calculation
        altitude = 44330.0 * (1.0 - ((pressure / p0) ** 0.1903))
        
        return altitude
    
    def read(self):
        """ Get all measurements in one call
            
        Returns:
            dict: Dictionary containing temperature, pressure, and altitude
        """
        temperature = self.get_temperature()
        pressure = self.get_pressure()
        altitude = self.get_altitude(pressure=pressure)
        
        return (temperature, pressure, altitude)
