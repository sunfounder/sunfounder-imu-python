
from typing import Optional
from .._i2c import I2C
from .mag_sensor import MagSensor
from .._utils import twos_complement, mapping


class QMC6310(MagSensor):
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

    MODE_SUSPEND    = 0b00
    MODE_NORMAL     = 0b01
    MODE_SINGLE     = 0b10
    MODE_CONTINUOUS = 0b11

    ODR_10_HZ   = 0b00
    ODR_50_HZ   = 0b01
    ODR_100_HZ  = 0b10
    ODR_200_HZ  = 0b11

    OSR1_8 = 0b00
    OSR2_4 = 0b01
    OSR4_2 = 0b10
    OSR8_1 = 0b11

    OSR2_1 = 0b00
    OSR2_2 = 0b01
    OSR2_4 = 0b10
    OSR2_8 = 0b11

    SET_RESET_ON   = 0b00
    SET_ONLY_ON    = 0b01
    SET_RESET_OFF  = 0b10

    RANGE_30_GAUSS = 0b00
    RANGE_12_GAUSS = 0b01
    RANGE_8_GAUSS  = 0b10
    RANGE_2_GAUSS  = 0b11

    SELF_TEST_ON  = 0b1 << 6
    SELF_TEST_OFF = 0b0 << 6

    SOFT_RST_ON   = 0b1 << 7
    SOFT_RST_OFF  = 0b0 << 7

    DATA_REGS = [REG_DATA_X, REG_DATA_Y, REG_DATA_Z]

    RANGES = {
        RANGE_30_GAUSS: 30.0,
        RANGE_12_GAUSS: 12.0,
        RANGE_8_GAUSS: 8.0,
        RANGE_2_GAUSS: 2.0,
    }

    def __init__(self, *args, address: Optional[int]=None, **kwargs):
        
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES)
            if addresses:
                address = addresses[0]
        super().__init__(address, *args, **kwargs)

        self.i2c = I2C(address=address)
        self.range = None

        self.init(
            set_reset_mode=self.SET_RESET_ON,
            mode=self.MODE_NORMAL,
            odr=self.ODR_200_HZ,
            osr1=self.OSR1_8,
            osr2=self.OSR2_8,
            range=self.RANGE_8_GAUSS,
        )
    
    def init(self,
        set_reset_mode: Optional[int]=None,
        mode: Optional[int]=None,
        odr: Optional[int]=None,
        osr1: Optional[int]=None,
        osr2: Optional[int]=None,
        range: Optional[int]=None,
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
        
        self.range = self.RANGES[range]
        # Define the sign for X Y and Z axis ( Don't know why, and how, but datasheet says so in Normal Mode Setup Example)
        self.set_sign(True, True, False)
        # Reset
        self.reset()
        self.set_ctrl2(set_reset_mode=set_reset_mode, range=range, self_test=self.SELF_TEST_OFF)
        self.set_ctrl1(mode=mode, odr=odr, osr1=osr1, osr2=osr2)

    def set_sign(self, x: bool, y: bool, z: bool) -> None:
        ''' Set the magnetometer sign.

        Args:
            x (bool): Sign for X axis.
            y (bool): Sign for Y axis.
            z (bool): Sign for Z axis.
        '''
        sign = (x << 2) | (y << 1) | z
        self.i2c.write_byte_data(self.REG_SIGN, sign)

    def reset(self) -> None:
        ''' Reset the device.
        '''
        self.i2c.write_byte_data(self.REG_CTL_2, self.SOFT_RST_ON)

    def set_ctrl1(self, mode: Optional[int]=None, odr: Optional[int]=None, osr1: Optional[int]=None, osr2: Optional[int]=None) -> None:
        ''' Set the control register 1.

        Args:
            mode (int, optional): Operation mode. Defaults to None.
            odr (int, optional): Output data rate. Defaults to None.
            osr1 (int, optional): OSR1. Defaults to None.
            osr2 (int, optional): OSR2. Defaults to None.
        '''
        ctrl1 = self.i2c.read_byte_data(self.REG_CTL_1)
        if mode is not None:
            ctrl1 &= ~(0b11 << 0)
            ctrl1 |= mode << 0
        if odr is not None:
            ctrl1 &= ~(0b11 << 2)
            ctrl1 |= odr << 2
        if osr1 is not None:
            ctrl1 &= ~(0b11 << 4)
            ctrl1 |= osr1 << 4
        if osr2 is not None:
            ctrl1 &= ~(0b11 << 6)
            ctrl1 |= osr2 << 6
        self.i2c.write_byte_data(self.REG_CTL_1, ctrl1)

    def set_ctrl2(self, set_reset_mode: Optional[int]=None, range: Optional[int]=None) -> None:
        ''' Set the control register 2.

        Args:
            set_reset_mode (int, optional): Set reset mode. Defaults to None.
            range (int, optional): Magnetometer range. Defaults to None.
        '''
        ctrl2 = self.i2c.read_byte_data(self.REG_CTL_2)
        if set_reset_mode is not None:
            ctrl2 &= ~(0b11 << 0)
            ctrl2 |= set_reset_mode << 0
        if range is not None:
            ctrl2 &= ~(0b11 << 2)
            ctrl2 |= range << 2
            self.range = self.RANGES[range]
        self.i2c.write_byte_data(self.REG_CTL_2, ctrl2)

    def self_test(self) -> None:
        ''' Set self test mode.
        '''
        self.i2c.write_byte_data(self.REG_CTL_2, self.SELF_TEST_ON)

    def set_range(self, range: int) -> None:
        ''' Set the magnetometer range.

        Args:
            range (int): Magnetometer range in gauss.
        '''
        self.set_ctrl2(range=range)

    def read_raw_mag(self) -> tuple[float, float, float]:
        ''' Get raw magnetometer data.

        Returns:
            tuple[float, float, float]: Magnetometer data in gauss.
        '''
        data = [self.i2c.read_word_data(reg) for reg in self.DATA_REGS]
        data = [twos_complement(d, 16) for d in data]
        data = [mapping(d, -32768, 32767, -self.range, self.range) for d in data]
        return tuple(data)
