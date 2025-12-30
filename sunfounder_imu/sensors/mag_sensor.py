
import math
from typing import Optional
from .._base import _Base

class MagSensor(_Base):
    """
    Magnetometer class to interface with mag sensor.

    Args:
        address (int): I2C address of the mag sensor.
    """
    def __init__(self, address: int, *args, offset: Optional[list[float]]=[0.0, 0.0, 0.0], scale: Optional[list[float]]=[1.0, 1.0, 1.0], **kwargs):
        super().__init__(address, *args, **kwargs)
        self.address = address

        self.azimuth = None
        self.calibrate_data_temp = None
        self.offsets = offset
        self.scales = scale

    def read_raw_mag(self) -> tuple[float, float, float]:
        raise NotImplementedError("read_raw method not implemented")

    def read_magnetometer(self, raw: bool=False) -> tuple[float, float, float]:
        ''' Get the magnetometer data.

        Args:
            raw (bool, optional): Whether to return the raw magnetometer data. Defaults to False.

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
        
        # apply offset
        mag_data = [v - self.offsets[i] for i, v in enumerate(mag_data)]
        # apply scale
        mag_data = [v * self.scales[i] for i, v in enumerate(mag_data)]
        # round to 3 decimal places
        mag_data = [round(v, 3) for v in mag_data]

        azimuth = self.read_azimuth(data=mag_data)
        return mag_data, azimuth

    def set_calibration_data(self, offsets: list[float], scales: list[float]) -> None:
        ''' Set the calibration offset and scale.

        Args:
            offsets (list[float]): Offsets.
            scales (list[float]): Scales.
        '''
        self.offsets = offsets
        self.scales = scales

    def calibrate_prepare(self) -> None:
        ''' Prepare the device for calibration.
        '''
        self.calibrate_data_temp = []

    def calibrate_step(self, data: Optional[tuple[float, float, float]]=None) -> None:
        ''' Add the magnetometer data to the calibration data.

        Args:
            data (Optional[tuple[float, float, float]], optional): Magnetometer data in gauss. Defaults to None.
        '''
        if self.calibrate_data_temp is None:
            self.calibrate_data_temp = []
        if data is None:
            data = list(self.read_magnetometer(raw=True))
        self.calibrate_data_temp.append(data)
        return data

    def calibrate_finish(self) -> None:
        ''' Finish the calibration and set the offset.

        Returns:
            tuple[list[float], list[float]]: Offsets and scales.
        '''
        if not self.calibrate_data_temp:
            raise ValueError("No calibration data. Please call calibrate_prepare() and calibrate_step() first.")
        data_min = list(map(min, zip(*self.calibrate_data_temp)))
        data_max = list(map(max, zip(*self.calibrate_data_temp)))
        # Bottom calculation is from zeus-car
        offsets = list(map(lambda x, y: (x + y) / 2, data_min, data_max))
        scales = list(map(lambda x, y: 2 / (y - x), data_min, data_max))
        avg_scale = sum(scales) / len(scales)
        scales = list(map(lambda x: avg_scale / x, scales))

        self.offsets = [round(offset, 2) for offset in offsets]
        self.scales = [round(scale, 2) for scale in scales]

        return self.offsets, self.scales
