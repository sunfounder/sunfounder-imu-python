from ._i2c import I2C
from ._base import _Base
from ._utils import twos_complement, mapping
from .data_type import MagDate

import math
from typing import Optional



class QMC6310(_Base):
    ''' QMC6310 magnetometer

    Args:
        *args: Pass through arguments to the parent class.
        address (int, optional): I2C address of the device. Default is 0x0D.
        **kwargs: Pass through keyword arguments to the parent class.

    '''

    I2C_ADDRESSES = [0x01C, 0x3C]

    REG_DATA_X = 0x01
    REG_DATA_Y = 0x03
    REG_DATA_Z = 0x05
    REG_STATUS = 0x09
    REG_CTL_1  = 0x0A
    REG_CTL_2  = 0x0B
    REG_SIGN   = 0x29

    MODE_SUSPEND    = 0b00 << 0
    MODE_NORMAL     = 0b01 << 0
    MODE_SINGLE     = 0b10 << 0
    MODE_CONTINUOUS = 0b11 << 0

    ODR_10_HZ   = 0b00 << 2
    ODR_50_HZ   = 0b01 << 2
    ODR_100_HZ  = 0b10 << 2
    ODR_200_HZ  = 0b11 << 2

    OSR1_8 = 0b00 << 4
    OSR2_4 = 0b01 << 4
    OSR4_2 = 0b10 << 4
    OSR8_1 = 0b11 << 4

    OSR2_1 = 0b00 << 6
    OSR2_2 = 0b01 << 6
    OSR2_4 = 0b10 << 6
    OSR2_8 = 0b11 << 6

    SET_RESET_ON   = 0b00 << 0
    SET_ONLY_ON    = 0b01 << 0
    SET_RESET_OFF  = 0b10 << 0

    RANGE_30_GAUSS = 0b00 << 2
    RANGE_12_GAUSS = 0b01 << 2
    RANGE_8_GAUSS  = 0b10 << 2
    RANGE_2_GAUSS  = 0b11 << 2

    SELF_TEST_ON  = 0b1 << 6
    SELF_TEST_OFF = 0b0 << 6

    SOFT_RST_ON   = 0b1 << 7
    SOFT_RST_OFF  = 0b0 << 7

    DATA_REGS = [REG_DATA_X, REG_DATA_Y, REG_DATA_Z]
    RANGE_GAUSS = {
        RANGE_30_GAUSS: 30.0,
        RANGE_12_GAUSS: 12.0,
        RANGE_8_GAUSS: 8.0,
        RANGE_2_GAUSS: 2.0,
    }

    def __init__(self, *args, address: Optional[int]=None, **kwargs):
        super().__init__(*args, **kwargs)
        
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES)
            if addresses:
                address = addresses[0]
        self.i2c = I2C(address=address)
        self.address = address
        self.offsets = self.config.get("qmc6310_offsets", [0, 0, 0])
        self.scales = self.config.get("qmc6310_scales", [1.0, 1.0, 1.0])
        self.calibrate_data_temp = None

        self.init(
            set_reset_mode=self.SET_RESET_ON,
            mode=self.MODE_NORMAL,
            odr=self.ODR_200_HZ,
            osr1=self.OSR1_8,
            osr2=self.OSR2_8,
            range=self.RANGE_8_GAUSS,
        )
    
    def init(self,
        set_reset_mode: int=SET_RESET_ON,
        mode: int=MODE_NORMAL,
        odr: int=ODR_200_HZ,
        osr1: int=OSR1_8,
        osr2: int=OSR2_8,
        range: int=RANGE_30_GAUSS,
        ) -> None:
        ''' Initialize the device.

        Args:
            set_reset_mode (int, optional): Set reset mode. Defaults to SET_RESET_ON.
            mode (int, optional): Operation mode. Defaults to MODE_NORMAL.
            odr (int, optional): Output data rate. Defaults to ODR_200_HZ.
            osr1 (int, optional): OSR1. Defaults to OSR1_8.
            osr2 (int, optional): OSR2. Defaults to OSR2_8.
            range (int, optional): Range. Defaults to RANGE_30_GAUSS.
        '''
        
        self.range = self.RANGE_GAUSS[range]

        # Define the sign for X Y and Z axis ( Don't know why, and how, but datasheet says so in Normal Mode Setup Example)
        self.i2c.write_byte_data(self.REG_SIGN, 0x06)

        # Reset
        self.i2c.write_byte_data(self.REG_CTL_2, self.SOFT_RST_ON)

        # Set control register 2
        self.i2c.write_byte_data(self.REG_CTL_2, set_reset_mode | range)

        # Set control register 1
        self.i2c.write_byte_data(self.REG_CTL_1, mode | odr | osr1 | osr2)

    def get_magnetometer_data(self, raw: bool=False) -> int:
        ''' Get the magnetometer data.

        Returns:
            tuple[float, float, float]: Magnetometer data in uT.
            raw (bool, optional): Whether to return the raw azimuth angle. Defaults to False.
        '''
        data = [self.i2c.read_word_data(reg) for reg in self.DATA_REGS]
        data = [twos_complement(d, 16) for d in data]
        data = [mapping(d, -32768, 32767, -self.range, self.range) for d in data]
        if not raw:
            data = [(value - self.offsets[i]) * self.scales[i] for i, value in enumerate(data)]
        return MagDate(*data)

    def get_azimuth(self, data: Optional[MagDate|tuple[float, float, float]]=None, plane: str='xy') -> float:
        ''' Get the azimuth angle.

        Args:
            data (Optional[tuple[float, float, float]], optional): Magnetometer data. Defaults to None.
            plane (str, optional): Plane to calculate azimuth. Defaults to 'xy'.

        Returns:
            float: Azimuth angle in degrees. Range is [0, 360).
        '''
        if data is None:
            data = self.get_magnetometer_data()
        if isinstance(data, MagDate):
            data = data.list()
        x, y, z = data
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

    def read(self) -> tuple[float, float, float]:
        ''' Read the magnetometer data.

        Returns:
            tuple[MagDate, float]: Magnetometer data in uT and azimuth angle in degrees.
        '''
        mag_data = self.get_magnetometer_data()
        azimuth = self.get_azimuth(data=mag_data)
        return mag_data, azimuth

    def set_calibration(self, offsets: list[float], scales: list[float]) -> None:
        ''' Set the calibration offset and scale.

        Args:
            offsets (list[float]): Offsets.
            scales (list[float]): Scales.
        '''
        self.offsets = offsets
        self.scales = scales
        self.config.set("qmc6310_offsets", self.offsets)
        self.config.set("qmc6310_scales", self.scales)

    def calibrate_prepare(self) -> None:
        ''' Prepare the device for calibration.
        '''
        self.calibrate_data_temp = []

    def calibrate_step(self, data: Optional[tuple[float, float, float]]=None) -> None:
        ''' Add the magnetometer data to the calibration data.

        Args:
            data (Optional[tuple[float, float, float]], optional): Magnetometer data in uT. Defaults to None.
        '''
        if data is None:
            data = self.get_magnetometer_data(raw=True).list()
        if self.calibrate_data_temp is None:
            self.calibrate_data_temp = []
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
        scales = list(map(lambda x, y: (y - x) / 2, data_min, data_max))
        avg_scale = sum(scales) / len(scales)
        scales = list(map(lambda x: avg_scale / x, scales))

        self.offsets = [round(offset, 2) for offset in offsets]
        self.scales = [round(scale, 2) for scale in scales]
        self.config.set("qmc6310_offsets", self.offsets)
        self.config.set("qmc6310_scales", self.scales)

        return self.offsets, self.scales
