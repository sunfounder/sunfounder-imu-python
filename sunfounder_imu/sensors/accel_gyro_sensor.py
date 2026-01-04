from typing import Optional

from .._base import _Base
from .._utils import remove_outliers_3d_and_mean
# from .ellipsoid_calibrator import EllipsoidCalibrator, DEFAULT_BIAS, DEFAULT_S_INV
from .linear_calibrator import LinearCalibrator, DEFAULT_BIAS, DEFAULT_SCALE

class AccelGyroSensor(_Base):
    """
    Accelerometer and Gyroscope class to interface with accel_gyro sensor.

    Args:
        address (int): I2C address of the accel_gyro sensor.
        accel_bias (list, optional): initial accel bias. Defaults to DEFAULT_BIAS.
        accel_scale (list, optional): initial accel scale. Defaults to DEFAULT_SCALE.
        gyro_bias (list, optional): initial gyro bias. Defaults to DEFAULT_BIAS.
        gyro_scale (list, optional): initial gyro scale. Defaults to DEFAULT_SCALE.
    """
    
    G = 9.80665

    def __init__(self,
            address: int,
            *args,
            accel_bias: list=DEFAULT_BIAS,
            accel_scale: list=DEFAULT_SCALE,
            gyro_bias: list=DEFAULT_BIAS,
            gyro_scale: list=DEFAULT_SCALE,
            **kwargs):
        super().__init__(address, *args, **kwargs)
        self.address = address

        self.accel_calibrator = LinearCalibrator(bias=accel_bias, scale=accel_scale)
        self.gyro_calibrator = LinearCalibrator(bias=gyro_bias, scale=gyro_scale)

        self.accel_cali_means = None
        self.accel_cali_raw = None
        self.gyro_cali_means = None
        self.gyro_cali_raw = None
        

    def read_raw(self) -> [float]:
        raise NotImplementedError("_read method not implemented")

    def read_raw_accel(self) -> [float]:
        raise NotImplementedError("_read_accel method not implemented")
        
    def read_raw_gyro(self) -> [float]:
        raise NotImplementedError("_read_gyro method not implemented")

    def read_raw_temperature(self) -> float:
        raise NotImplementedError("_read_temp method not implemented")

    def read(self) -> tuple:
        ''' Read all data
        
        Returns:
            tuple: (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), temperature
        '''
        accel_data, gyro_data, temperature = self.read_raw()
        
        # Accel data
        # apply calibration
        accel_data = self.accel_calibrator.calibrate(accel_data)

        # convert to m/s^2
        accel_data = [v * self.G for v in accel_data]
        # round to 2 decimal places
        accel_data = [round(v, 2) for v in accel_data]

        # Gyro data
        # apply calibration
        gyro_data = self.gyro_calibrator.calibrate(gyro_data)
        # round to 2 decimal places
        gyro_data = [round(v, 2) for v in gyro_data]

        return accel_data, gyro_data, temperature

    def calibrate_prepare(self) -> None:
        ''' Prepare accelerometer calibration, clear temp datas
        '''
        self.accel_cali_means = []
        self.accel_cali_raw = []
        self.gyro_cali_means = []
        self.gyro_cali_raw = []

    def calibrate_read(self) -> None:
        ''' Calibration read, read and store raw data
        '''
        if self.accel_cali_raw is None or self.gyro_cali_raw is None:
            self.calibrate_prepare()
        
        accel_data = list(self.read_raw_accel())
        gyro_data = list(self.read_raw_gyro())
        self.accel_cali_raw.append(accel_data)
        self.gyro_cali_raw.append(gyro_data)
        return accel_data, gyro_data

    def calibrate_step(self) -> None:
        ''' Calibration step, calculate mean of raw data
        '''
        if self.accel_cali_raw is None:
            raise ValueError('Calibration data is empty')
        if self.gyro_cali_raw is None:
            raise ValueError('Calibration data is empty')
        accel_meaned = remove_outliers_3d_and_mean(self.accel_cali_raw)
        gyro_meaned = remove_outliers_3d_and_mean(self.gyro_cali_raw)
        
        self.accel_cali_means.append(accel_meaned)
        self.gyro_cali_means.append(gyro_meaned)
        self.accel_cali_raw = []
        self.gyro_cali_raw = []
        return accel_meaned, gyro_meaned

    def calibrate_finish(self) -> None:
        ''' Calibration finish, calculate offset
        '''
        if self.accel_cali_means is None:
            raise ValueError('Calibration data is empty')
        if self.gyro_cali_means is None:
            raise ValueError('Calibration data is empty')
        
        # Calculate offset
        accel_bias, accel_scale = self.accel_calibrator.fit(self.accel_cali_means)
        gyro_bias, gyro_scale = self.gyro_calibrator.fit(self.gyro_cali_means)
        self.set_calibration_data(accel_bias, accel_scale, gyro_bias, gyro_scale)

        return accel_bias, accel_scale, gyro_bias, gyro_scale

    def set_calibration_data(self,
            accel_bias: Optional[list]=None,
            accel_scale: Optional[list]=None,
            gyro_bias: Optional[list]=None,
            gyro_scale: Optional[list]=None,
            ) -> None:
        ''' Set calibration data
        
        Args:
            accel_bias (list, optional): Acceleration bias. Defaults to [0, 0, 0].
            accel_scale (list, optional): Acceleration scale. Defaults to [1, 1, 1].
            gyro_bias (list, optional): Gyroscope bias. Defaults to [0, 0, 0].
            gyro_scale (list, optional): Gyroscope scale. Defaults to [1, 1, 1].
        '''
        if accel_bias is not None:
            self.accel_calibrator.set_bias(accel_bias)
        if accel_scale is not None:
            self.accel_calibrator.set_scale(accel_scale)
        if gyro_bias is not None:
            self.gyro_calibrator.set_bias(gyro_bias)
        if gyro_scale is not None:
            self.gyro_calibrator.set_scale(gyro_scale)
