
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
        
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES)
            if addresses:
                address = addresses[0]
        super().__init__(address, *args, **kwargs)

        self.i2c = I2C(address=address)

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

    def read_raw_mag(self) -> tuple[float, float, float]:
        ''' Get raw magnetometer data.

        Returns:
            tuple[float, float, float]: Magnetometer data in gauss.
        '''
        data = [self.i2c.read_word_data(reg) for reg in self.DATA_REGS]
        data = [twos_complement(d, 16) for d in data]
        data = [mapping(d, -32768, 32767, -self.range, self.range) for d in data]
        return tuple(data)
