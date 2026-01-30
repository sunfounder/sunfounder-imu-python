
from typing import Optional
import time

from .._i2c import I2C
from .mag_sensor import MagSensor
from .._utils import twos_complement, mapping

from .._logger import Logger

class QMC6309(MagSensor):
    ''' QMC6309 magnetometer

    Args:
        *args: Pass through arguments to the parent class.
        address (int, optional): I2C address of the device. Default is 0x0D.
        **kwargs: Pass through keyword arguments to the parent class.

    '''

    I2C_ADDRESSES = [0x7C, 0x0C]

    RED_CHIP_ID = 0x00
    REG_DATA_X  = 0x01
    REG_DATA_Y  = 0x03
    REG_DATA_Z  = 0x05
    REG_STATUS  = 0x09
    REG_CTRL_1  = 0x0A
    REG_CTRL_2  = 0x0B
    REG_CTRL_3  = 0x0E
    REG_TEST_X  = 0x13
    REG_TEST_Y  = 0x14
    REG_TEST_Z  = 0x15

    MODE_SUSPEND    = 0b00
    MODE_NORMAL     = 0b01
    MODE_SINGLE     = 0b10
    MODE_CONTINUOUS = 0b11

    ODR_1_HZ   = 0b000
    ODR_10_HZ  = 0b001
    ODR_50_HZ  = 0b010
    ODR_100_HZ = 0b011
    ODR_200_HZ = 0b100

    OSR1_8 = 0b00
    OSR2_4 = 0b01
    OSR4_2 = 0b10
    OSR8_1 = 0b11

    OSR2_1 = 0b000
    OSR2_2 = 0b001
    OSR2_4 = 0b010
    OSR2_8 = 0b011
    OSR2_16 = 0b100

    SET_RESET_ON   = 0b00
    SET_ONLY_ON    = 0b01
    SET_RESET_OFF  = 0b11

    RANGE_32_GAUSS = 0b00
    RANGE_16_GAUSS = 0b01
    RANGE_8_GAUSS  = 0b10

    SELF_TEST_ON  = 0b1 << 6
    SELF_TEST_OFF = 0b0 << 6

    SOFT_RST_ON   = 0b1 << 7
    SOFT_RST_OFF  = 0b0 << 7

    DATA_REGS = [REG_DATA_X, REG_DATA_Y, REG_DATA_Z]

    RANGES = {
        RANGE_32_GAUSS: 32.0,
        RANGE_16_GAUSS: 16.0,
        RANGE_8_GAUSS: 8.0,
    }

    def __init__(self, *args, address: Optional[int]=None, **kwargs):
        
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES, all=True)
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
        # Reset
        self.reset()
        self.set_ctrl2(set_reset_mode=set_reset_mode, range=range, self_test=self.SELF_TEST_OFF)
        self.set_ctrl1(mode=mode, osr1=osr1, osr2=osr2)

    def reset(self) -> None:
        ''' Reset the device.
        '''
        self.i2c.write_byte_data(self.REG_CTRL_2, self.SOFT_RST_ON)
        time.sleep(0.1)
        self.i2c.write_byte_data(self.REG_CTRL_2, self.SOFT_RST_OFF)

    def set_ctrl1(self, mode: Optional[int]=None, osr1: Optional[int]=None, osr2: Optional[int]=None) -> None:
        ''' Set the control register 1.

        Args:
            mode (int, optional): Operation mode. Defaults to None.
            osr1 (int, optional): OSR1. Defaults to None.
            osr2 (int, optional): OSR2. Defaults to None.
        '''
        ctrl1 = self.i2c.read_byte_data(self.REG_CTRL_1)
        if mode is not None:
            ctrl1 &= ~(0b11 << 0)
            ctrl1 |= mode << 0
        if osr1 is not None:
            ctrl1 &= ~(0b11 << 3)
            ctrl1 |= osr1 << 3
        if osr2 is not None:
            ctrl1 &= ~(0b111 << 5)
            ctrl1 |= osr2 << 5
        self.i2c.write_byte_data(self.REG_CTRL_1, ctrl1)

    def set_ctrl2(self, set_reset_mode: Optional[int]=None, odr: Optional[int]=None, range: Optional[int]=None) -> None:
        ''' Set the control register 2.

        Args:
            set_reset_mode (int, optional): Set reset mode. Defaults to None.
            odr (int, optional): Output data rate. Defaults to None.
            range (int, optional): Magnetometer range. Defaults to None.
        '''
        ctrl2 = self.i2c.read_byte_data(self.REG_CTRL_2)
        if set_reset_mode is not None:
            ctrl2 &= ~(0b11 << 0)
            ctrl2 |= set_reset_mode << 0
        if range is not None:
            ctrl2 &= ~(0b11 << 2)
            ctrl2 |= range << 2
            self.range = self.RANGES[range]
        if odr is not None:
            ctrl2 &= ~(0b111 << 4)
            ctrl2 |= odr << 4
        self.i2c.write_byte_data(self.REG_CTRL_2, ctrl2)

    def set_ctrl3(self, self_test: Optional[int]=None) -> None:
        ''' Set the control register 3.

        Args:
            self_test (int, optional): Self test mode. Defaults to None.
        '''
        ctrl3 = self.i2c.read_byte_data(self.REG_CTRL_3)
        if self_test is not None:
            ctrl3 &= ~(0b1 << 7)
            ctrl3 |= self_test << 7
        self.i2c.write_byte_data(self.REG_CTRL_3, ctrl3)

    def self_test(self) -> bool:
        ''' Set self test mode.

        Returns:
            bool: True if self test passed, False otherwise.
        '''
        self.set_ctrl1(mode=self.MODE_SUSPEND)
        self.set_ctrl1(mode=self.MODE_CONTINUOUS)
        time.sleep(0.02)
        self.set_ctrl3(self_test=self.SELF_TEST_ON)
        time.sleep(0.15)
        ready = self.read_status()["self_test_ready"]
        if not ready:
            self.logger.error("Self test not ready")
            return None
        datas = self.i2c.read_i2c_block_data(self.REG_TEST_X, 3)
        x = twos_complement(datas[0], 8)
        y = twos_complement(datas[1], 8)
        z = twos_complement(datas[2], 8)
        if x > -50 and x < -1 and y > -50 and y < -1 and z > -50 and z < -1:
            self.logger.info("Self test passed")
            return True
        else:
            self.logger.error("Self test failed")
            return False

    def set_range(self, range: int) -> None:
        ''' Set the magnetometer range.

        Args:
            range (int): Magnetometer range in gauss.
        '''
        self.set_ctrl2(range=range)

    def read_status(self) -> dict:
        ''' Read the status register.

        Returns:
            dict: Status register value.
        '''
        data = self.i2c.read_byte_data(self.REG_STATUS)
        nvm_load_done = bool((data >> 4) & 0b1)
        nvm_ready = bool((data >> 3) & 0b1)
        self_test_ready = bool((data >> 2) & 0b1)
        overflow = bool((data >> 1) & 0b1)
        data_ready = bool(data & 0b1)

        return {
            "nvm_load_done": nvm_load_done,
            "nvm_ready": nvm_ready,
            "self_test_ready": self_test_ready,
            "overflow": overflow,
            "data_ready": data_ready,
        }

    def suspend(self) -> None:
        ''' Suspend the device.
        '''
        self.set_ctrl1(mode=self.MODE_SUSPEND)

    def is_data_ready(self) -> bool:
        ''' Check if data is ready.

        Returns:
            bool: True if data is ready, False otherwise.
        '''
        status = self.read_status()
        return status["data_ready"]

    def read_raw_mag(self) -> tuple[float, float, float]:
        ''' Get raw magnetometer data.

        Returns:
            tuple[float, float, float]: Magnetometer data in gauss.
        '''
        if not self.is_data_ready():
            self.logger.warning("Data not ready")
            return None
        data = [self.i2c.read_word_data(reg) for reg in self.DATA_REGS]
        data = [twos_complement(d, 16) for d in data]
        data = [mapping(d, -32768, 32767, -self.range, self.range) for d in data]
        return tuple(data)
