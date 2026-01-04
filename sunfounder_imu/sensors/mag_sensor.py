
import math
from typing import Optional

from .._base import _Base
from .._utils import remove_outliers_3d_and_mean
from .linear_calibrator import LinearCalibrator, DEFAULT_BIAS, DEFAULT_SCALE

class MagSensor(_Base):
    """
    Magnetometer class to interface with mag sensor.

    Args:
        address (int): I2C address of the mag sensor.
        bias (list, optional): initial bias. Defaults to None.
        scale (list, optional): initial scale. Defaults to None.
    """
    def __init__(self,
            address: int,
            *args,
            bias: Optional[list[float]]=DEFAULT_BIAS,
            scale: Optional[list[float]]=DEFAULT_SCALE,
            **kwargs):
        super().__init__(address, *args, **kwargs)
        self.address = address

        self.calibrator = LinearCalibrator(bias=bias, scale=scale)

        self.mag_cali_means = None
        self.mag_cali_raw = None

    def read_raw_mag(self) -> tuple[float, float, float]:
        raise NotImplementedError("read_raw method not implemented")

    def read_magnetometer(self) -> tuple[float, float, float]:
        ''' Get the magnetometer data.

        Returns:
            tuple[float, float, float]: Magnetometer data in gauss.
        '''
        mag_data = self.read_raw_mag()
        return mag_data

    def read_azimuth(self, data: Optional[list[float]]=None, plane: str='xy') -> float:
        ''' Get the azimuth angle.

        Args:
            data (Optional[list[float]], optional): Magnetometer data. Defaults to None.
            plane (str, optional): Plane to calculate azimuth. Defaults to 'xy'.

        Returns:
            float: Azimuth angle in degrees. Range is [0, 360).
        '''
        if data is None:
            data = self.read_magnetometer()
        if isinstance(data, list):
            x, y, z = data
        else:
            raise ValueError(f"Invalid data type: {type(data)}")
        if plane == 'xy':
            azimuth = math.atan2(x, y) * 180.0 / math.pi
            azimuth -= 90.0
        elif plane == 'yz':
            azimuth = math.atan2(z, y) * 180.0 / math.pi
        elif plane == 'xz':
            azimuth = math.atan2(z, x) * 180.0 / math.pi
        else:
            raise ValueError(f"Invalid plane: {plane}")
        if azimuth < 0:
            azimuth += 360.0
        return azimuth

    def read(self) -> tuple[tuple[float, float, float], float]:
        ''' Read the magnetometer data.

        Returns:
            tuple[tuple[float, float, float], float]: Magnetometer data in gauss and azimuth angle in degrees.
        '''
        mag_data = self.read_magnetometer()
        
        mag_data = self.calibrator.calibrate(mag_data)
        # round to 3 decimal places
        mag_data = [round(v, 3) for v in mag_data]

        azimuth = self.read_azimuth(data=mag_data)
        return mag_data, azimuth

    def set_calibration_data(self, bias: Optional[list[float]]=None, scale: Optional[list[float]]=None) -> None:
        ''' Set the calibration bias and scale.

        Args:
            bias (Optional[list[float]], optional): Bias. Defaults to None.
            scale (Optional[list[float]], optional): Scale. Defaults to None.
        '''
        if bias is not None:
            self.calibrator.set_bias(bias)
        if scale is not None:
            self.calibrator.set_scale(scale)

    def calibrate_prepare(self) -> None:
        ''' Prepare the device for calibration.
        '''
        self.mag_cali_raw = []
        self.mag_cali_means = []

    def calibrate_read(self) -> None:
        ''' Read the magnetometer data and add it to the calibration data.
        '''
        if self.mag_cali_raw is None:
            self.calibrate_prepare()
        data = list(self.read_magnetometer())
        self.mag_cali_raw.append(data)
        return data

    def calibrate_step(self) -> None:
        ''' Add the magnetometer data to the calibration data.
        '''
        if self.mag_cali_raw is None:
            raise ValueError("No calibration data. Please call calibrate_prepare() first.")
        meaned = remove_outliers_3d_and_mean(self.mag_cali_raw)
        self.mag_cali_means.append(meaned)
        self.mag_cali_raw = []
        return meaned

    def calibrate_finish(self) -> None:
        ''' Finish the calibration and set the offset.

        Returns:
            tuple[list[float], list[float]]: Offsets and scales.
        '''
        if not self.mag_cali_means:
            raise ValueError("No calibration data. Please call calibrate_prepare() and calibrate_step() first.")
        biases, scales = self.calibrator.fit(self.mag_cali_means)
        self.set_calibration_data(biases, scales)
        return biases, scales
