from typing import Optional

from .._i2c import I2C

from .baro_sensor import BaroSensor
from .accel_gyro_sensor import AccelGyroSensor
from .mag_sensor import MagSensor

from .spl06_001 import SPL06_001
from .qmc6310 import QMC6310
from .sh3001 import SH3001

BAROS = {
    "SPL06_001": SPL06_001,
}

ACCEL_GYROS = {
    "SH3001": SH3001,
}

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

def get_baro_sensor(addresses: Optional[list]=None) -> BaroSensor:
    """ Get barometer sensor object
    
    Args:
        addresses (list, optional): List of I2C addresses of the barometer sensor. Defaults to None.
        
    Returns:
        BaroSensor: Barometer sensor object
    """
    addresses = addresses or I2C.scan()
    sensor = None

    for address in addresses:
        if address in BAROMETER_ADDRESS_MAP:
            sensor = BAROMETER_ADDRESS_MAP[address](address)
            break
    return sensor

def get_accel_gyro_sensor(addresses: Optional[list]=None) -> AccelGyroSensor:
    """ Get accel_gyro sensor object
    
    Args:
        addresses (list, optional): List of I2C addresses of the accel_gyro sensor. Defaults to None.
        
    Returns:
        AccelGyroSensor: Accelerometer-gyroscope sensor object
    """
    addresses = addresses or I2C.scan()
    sensor = None

    for address in addresses:
        if address in ACCEL_GYRO_ADDRESS_MAP:
            sensor = ACCEL_GYRO_ADDRESS_MAP[address](address)
            break
    return sensor
        
def get_mag_sensor(addresses: Optional[list]=None) -> MagSensor:
    """ Get magnetometer sensor object
    
    Args:
        addresses (list, optional): List of I2C addresses of the magnetometer sensor. Defaults to None.
        
    Returns:
        MagSensor: Magnetometer sensor object
    """
    addresses = addresses or I2C.scan()
    sensor = None

    for address in addresses:
        if address in MAGNETOMETER_ADDRESS_MAP:
            sensor = MAGNETOMETER_ADDRESS_MAP[address](address)
            break
    return sensor
